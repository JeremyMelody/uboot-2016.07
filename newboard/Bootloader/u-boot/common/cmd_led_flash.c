#include <common.h>
#include <net.h>
#include <command.h>
#include <asm/arch/hardware.h>
#include <asm/io.h>
#include <config_cmd_default.h>
#include <asm/io.h>

#define GPIO1_OE *((volatile unsigned int *)0x4804C134)
#define GPIO1_DATAOUT *((volatile unsigned int *)0x4804C13C)
#define GPIO1_SETDATAOUT *((volatile unsigned int *)0x4804C194)
#define GPIO1_CLEARDATAOUT *((volatile unsigned int *)0x4804C190)


/* Added by MYIR, for flashing LED GPIO3_18 */
#define GPIO3_BASE		0x481AE000
//#define GPIO3_OE		*((volatile unsigned int *)(GPIO3_BASE + 0x134))
//#define GPIO3_DATAOUT		*((volatile unsigned int *)(GPIO3_BASE + 0x13c))
//#define GPIO3_SETDATAOUT 	*((volatile unsigned int *)(GPIO3_BASE + 0x194))
//#define GPIO3_CLEARDATAOUT	*((volatile unsigned int *)(GPIO3_BASE + 0x190))

#define GPIO3_OE                (GPIO3_BASE + 0x134)
#define GPIO3_DATAOUT           (GPIO3_BASE + 0x13c)
#define GPIO3_SETDATAOUT        (GPIO3_BASE + 0x194)
#define GPIO3_CLEARDATAOUT      (GPIO3_BASE + 0x190)

#define LED_BIT			(1 << 18)

#define PRINT_LINE		printf("FUNC:%s(), LINE:%d\n", __func__, __LINE__)

static init_gpio3(void)
{
	/* enable gpio3 clock */
	__raw_writel(0x2, 0x44E00000 + 0xb4);

	/* wait for stable */
	while( 0x2 != (__raw_readl(0x44E00000 + 0xb4)&0x3));

	/* write clk */
	__raw_writel(__raw_readl(0x44E00000 + 0xb4)|0x00040000, 0x44E00000 + 0xb4);

	/* wait for stable */
	while( 0x00040000 != (__raw_readl(0x44E00000 + 0xb4) & 0x00040000));

	/* enable module */
	__raw_writel(__raw_readl(0x481AE000 + 0x130) &~ 0x00000001, 0x481AE000 + 0x130);

	/* reset module */
	__raw_writel(__raw_readl(0x481AE000 + 0x10) | 0x00000002, 0x481AE000 + 0x10);

	/* wait for reset complete */
	while(!(__raw_readl(0x481AE000 + 0x114)&0x00000001));

	/* set to gpio mode, receive disable, pull disable */
	__raw_writel(0x00000008|7, 0x44E10000 + 0x09a0);
	 
}

int do_led(cmd_tbl_t *cmdtp, int flag, int argc, char *argv[])  
{
	/* MYIR */
	init_gpio3();

	switch(argc) { 
		case 3:
			//__raw_writel(0x0, 0x4804C134);
			__raw_writel(__raw_readl(GPIO3_OE) & ~LED_BIT, GPIO3_OE);

			if(strcmp(argv[1], "flash") == 0) {
				if(strcmp(argv[2], "all") == 0) {
					/* Added by MYIR */
					printf("\r\nSystem update complete. \r\nPlease select NAND boot mode by setting the jumper, and then re-power the board.\r\n");
					while(1) {
						//__raw_writel(0x0C000000,0x4804C194);
						__raw_writel(LED_BIT, GPIO3_SETDATAOUT);
						udelay(500000);
						//__raw_writel(0x0C000000, 0x4804C190);
						__raw_writel(LED_BIT, GPIO3_CLEARDATAOUT);
						udelay(500000);
					}
				} 
			}	 
			break;
		case 2:
			if(strcmp(argv[1], "on") == 0) {
				//__raw_writel(0x0, 0x4804C134);
				//__raw_writel(0x0C000000,0x4804C194);
				__raw_writel(__raw_readl(GPIO3_OE)&~LED_BIT, GPIO3_OE);
				__raw_writel(LED_BIT, GPIO3_SETDATAOUT);

			} else if(strcmp(argv[1], "off") == 0) {
				//__raw_writel(0x0, 0x4804C134);
				//__raw_writel(0x0C000000, 0x4804C190);
                                __raw_writel(__raw_readl(GPIO3_OE)&~LED_BIT, GPIO3_OE);
                                __raw_writel(LED_BIT, GPIO3_CLEARDATAOUT);
			}
			break;
		case 1:          
		default:  
			printf ("wrong argv, see help led!\n");
	}    
}

U_BOOT_CMD(
        led ,      3,      1,      do_led,
        "    led control\n",
        "flash all\n"
	"on/off\n"
);
