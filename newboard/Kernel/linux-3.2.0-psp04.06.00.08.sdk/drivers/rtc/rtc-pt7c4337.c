/****************************************************************************************
 * driver/rtc/rtc-PT7C4337.c
 *Copyright     :
 *Author        :   Jeremy YU
 *Date      : 2016-11-14
 *This driver use for AM3352 extern rtc. Use i2c0 ,the chip is PT7C4337
 ********************************************************************************************/
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/bcd.h>
#include <linux/rtc.h>
#include <linux/delay.h>
#ifdef CONFIG_ANDROID_POWER
#include <linux/android_power.h>
#endif

#if 1
#define DBG(x...)   printk(KERN_DEBUG x)
#else
#define DBG(x...)
#endif

#define DRV_NAME "rtc_PT7C4337"

#define   PT7C4337_ADDR 0x68
#define   RTC_SEC       0x00
#define   RTC_MIN       0x01
#define   RTC_HOUR      0x02
#define   RTC_WEEK      0x03
#define   RTC_DAY       0x04
#define   RTC_MON       0x05
#define   RTC_YEAR      0x06
#define   RTC_A1_SEC    0x07
#define   RTC_A1_MIN    0x08
#define   RTC_A1_HOUR   0x09
#define   RTC_A1_DAY    0x0A
#define   RTC_A2_MIN    0x0B
#define   RTC_A2_HOUR   0x0C
#define   RTC_A2_DAY    0x0D
#define   RTC_CTR       0x0E
#define   RTC_STA       0x0F
#define   CENTURY       0x80
#define   E_TIME        0x80
#define   INTCN         0x04
#define   A2IE          0x02
#define   A1IE          0x01
#define   A2F           0x02
#define   A1F           0x01
#define   OSF           0x80

static int PT7C4337_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int PT7C4337_remove(struct i2c_client *client);

static int PT7C4337_i2c_read_regs(struct i2c_client *client, u8 reg, u8 buf[], unsigned len) {
    int ret;
    unsigned char offset = 0;
    struct i2c_msg msgs[] = {
        { client->addr, 0, 1, &offset },
        { client->addr, I2C_M_RD, len, buf },
    };
    buf[0] = reg;
    ret = i2c_transfer(client->adapter, msgs, 2);
    DBG("*************PT7C4337_i2c_read_regs:reg=%d,value=%d ret: %d offset: %d\n", reg, buf[0], ret, offset);
    if (ret > 0) {
        ret = 0;
    }
    return ret;
}

static int PT7C4337_i2c_set_regs(struct i2c_client *client, u8 reg, u8 const buf[], __u16 len) {
    int ret = 0;
    uint8_t buffer[len + 1];

    buffer[0] = reg;
    memcpy(&buffer[1], buf, len);
    ret = i2c_master_send(client, buffer, len + 1);
    DBG("*************PT7C4337_i2c_set_regs:reg=%d,value=%d, ret=%d\n", reg, buf[0], ret);
    if (ret == len + 1) {
        return 0;
    }

    return -EIO;
}

static int PT7C4337_i2c_read_time(struct i2c_client *client, struct rtc_time *tm) {
    u8 regs[7] = { 0, };
    int ret;
    ret = PT7C4337_i2c_read_regs(client, RTC_SEC, regs, 7);
    tm->tm_year = 70 + (regs[6] & 0x0f) + (regs[6] >> 4) * 10;
    tm->tm_mon = (regs[5] & 0x0f) + ((regs[5] >> 4) & 0x01) * 10;
    tm->tm_mday = (regs[4] & 0x0f) + (regs[4] >> 4) * 10;
    tm->tm_wday = regs[3];
    tm->tm_hour = (regs[2] & 0x0f) + ((regs[2] >> 4) & 0x03) * 10;
    tm->tm_min = (regs[1] & 0x0f) + (regs[1] >> 4) * 10;
    tm->tm_sec = (regs[0] & 0x0f) + (regs[0] >> 4) * 10;
    DBG("-----------rtc_sec=%d", tm->tm_sec);
    DBG("-----------rtc_min=%d", tm->tm_min);
    DBG("-----------rtc_hour=%d", tm->tm_hour);
    DBG("-----------rtc_wday=%d", tm->tm_wday);
    DBG("-----------rtc_mday=%d", tm->tm_mday);
    DBG("-----------rtc_mon=%d", tm->tm_mon);
    DBG("-----------rtc_year=%d", tm->tm_year);
    DBG("-----------rtc_yday=%d", tm->tm_yday);
    DBG("-----------rtc_isdst=%d\n", tm->tm_isdst);
    return ret;
}

static int PT7C4337_i2c_set_time(struct i2c_client *client, struct rtc_time  *tm) {
    u8 regs[7] = { 0, };
    int ret = 0;

    DBG("-----------rtc_sec=%d", tm->tm_sec);
    DBG("-----------rtc_min=%d", tm->tm_min);
    DBG("-----------rtc_hour=%d\n", tm->tm_hour);

    DBG("-----------rtc_wday=%d\n", tm->tm_wday);

    DBG("-----------rtc_mday=%d", tm->tm_mday);
    DBG("-----------rtc_mon=%d", tm->tm_mon);
    DBG("-----------rtc_year=%d\n", tm->tm_year);

    DBG("-----------rtc_yday=%d", tm->tm_yday);
    DBG("-----------rtc_isdst=%d\n", tm->tm_isdst);
    tm->tm_year -= 70;
    regs[6] = ((((tm->tm_year) / 10) % 10) << 4) + ((tm->tm_year) % 10);
    regs[5] = (((tm->tm_mon) / 10) << 4) + ((tm->tm_mon) % 10);
    regs[4] = (((tm->tm_mday) / 10) << 4) + ((tm->tm_mday) % 10);
    regs[3] = tm->tm_wday;
    regs[2] = (((tm->tm_hour) / 10) << 4) + ((tm->tm_hour) % 10);
    regs[1] = (((tm->tm_min) / 10) << 4) + ((tm->tm_min) % 10);
    regs[0] = (((tm->tm_sec) / 10) << 4) + ((tm->tm_sec) % 10);


    ret = PT7C4337_i2c_set_regs(client, RTC_SEC, regs, 7);
    /*enable oscillator and time count chain*/
    regs[0] = 0;
    PT7C4337_i2c_read_regs(client, RTC_CTR, regs, 1);
    regs[0]  = regs[0] & (~ E_TIME);
    PT7C4337_i2c_set_regs(client, RTC_CTR, regs, 1);
    /*enable oscillator flag*/
    regs[0] = 0;
    PT7C4337_i2c_read_regs(client, RTC_STA, regs, 1);
    regs[0]  = regs[0] & (~ OSF);
    PT7C4337_i2c_set_regs(client, RTC_STA, regs, 1);

    regs[0] = 0;
    PT7C4337_i2c_read_regs(client, RTC_CTR, regs, 1);
    DBG("----------- RTC_CTR=0x%x\n", regs[0]);
    regs[0] = 0;
    PT7C4337_i2c_read_regs(client, RTC_STA, regs, 1);
    DBG("----------- RTC_STA=0x%x\n", regs[0]);

    return ret;
}

static int PT7C4337_rtc_read_time(struct device *dev, struct rtc_time *tm) {
    return PT7C4337_i2c_read_time(to_i2c_client(dev), tm);
}

static int PT7C4337_rtc_set_time(struct device *dev, struct rtc_time *tm) {
    return PT7C4337_i2c_set_time(to_i2c_client(dev), tm);
}

static const struct i2c_device_id PT7C4337_id[] = {
    { "PT7C4337", 0 },
    { }
};

static struct i2c_driver PT7C4337_driver = {
    .driver     = {
        .name   = DRV_NAME,
        .owner  = THIS_MODULE,
    },
    .probe      = PT7C4337_probe,
    .remove     = PT7C4337_remove,
    .id_table   = PT7C4337_id,
};

static const struct rtc_class_ops PT7C4337_rtc_ops = {
    .read_time  = PT7C4337_rtc_read_time,
    .set_time   = PT7C4337_rtc_set_time,
};

static int PT7C4337_probe(struct i2c_client *client,
                          const struct i2c_device_id *id) {
    int rc = 0;
    struct rtc_device *rtc = NULL;
    u8 regs[RTC_STA+1] = { 0 };

    if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
        rc = -ENODEV;
        goto failout;
    }
    rtc = rtc_device_register(client->name, &client->dev, &PT7C4337_rtc_ops, THIS_MODULE);
    if (IS_ERR(rtc)) {
        rc = PTR_ERR(rtc);
        goto failout;
    }
    i2c_set_clientdata(client, rtc);

    PT7C4337_i2c_read_regs(client, RTC_STA, regs, sizeof regs);
    DBG("PT7C4337_probe: RTC_CTR=0x%x  RTC_STA=0x%0x\n", regs[RTC_CTR], regs[RTC_STA]);
    regs[0] = 0x04; // set INTCN to 1
    PT7C4337_i2c_set_regs(client, RTC_CTR, regs, 1);

    if((regs[RTC_STA] & 0x80) != 0) {
        DBG("RTC_STA OSF is 1, the oscillator has stopped working. reset it\n");
        regs[0] = (regs[RTC_STA] & 0x7f);
        PT7C4337_i2c_set_regs(client, RTC_STA, regs, 1);
    }
    PT7C4337_i2c_read_regs(client, RTC_STA, regs, sizeof regs);
    DBG("PT7C4337_probe: RTC_CTR=0x%x RTC_STA=0x%0x after set\n", regs[RTC_CTR], regs[RTC_STA]);
    return 0;

failout:
    if(rtc) {
        rtc_device_unregister(rtc);
    }
    return rc;
}

static int PT7C4337_remove(struct i2c_client *client) {
    struct rtc_device *rtc = i2c_get_clientdata(client);
    if(rtc) {
        rtc_device_unregister(rtc);
    }
    return 0;
}

static int __init PT7C4337_init(void) {
    i2c_add_driver(&PT7C4337_driver);
    return 0;
}

static void __exit PT7C4337_exit(void) {
    i2c_del_driver(&PT7C4337_driver);
}

MODULE_DESCRIPTION("Pericom PT7C4337 RTC driver");
MODULE_LICENSE("GPL");

module_init(PT7C4337_init);
module_exit(PT7C4337_exit);
