#include <stdio.h>
#include <linux/types.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/ioctl.h>
#include <errno.h>
#define I2C_RETRIES 0x0701
#define I2C_TIMEOUT 0x0702
#define I2C_RDWR 0x0707 

#define ADDR_SGTL5K 0x0A

struct i2c_msg{
	unsigned short addr;
	unsigned short flags;
#define I2C_M_TEN 0x0010
#define I2C_M_RD 0x0001
	unsigned short len;
	unsigned char *buf;
};

struct i2c_rdwr_ioctl_data{
	struct i2c_msg *msgs;
	int nmsgs; 

};

int main()
{
	int fd,ret;
	struct i2c_rdwr_ioctl_data i2c_data;
	fd=open("/dev/i2c-0",O_RDWR);

	if(fd<0){
		perror("open error");
	}
	i2c_data.nmsgs=2; 
	i2c_data.msgs=(struct i2c_msg*)malloc(i2c_data.nmsgs*sizeof(struct i2c_msg));
	if(!i2c_data.msgs){
		perror("malloc error");
		exit(1);
	}
	ioctl(fd,I2C_TIMEOUT,1);
	ioctl(fd,I2C_RETRIES,2);

	i2c_data.nmsgs=2;
	(i2c_data.msgs[0]).len=2; 
	(i2c_data.msgs[0]).addr=ADDR_SGTL5K; 
	(i2c_data.msgs[0]).flags=0;//write
	(i2c_data.msgs[0]).buf=(unsigned char*)malloc(2);
	(i2c_data.msgs[0]).buf[0]=0x00;
	(i2c_data.msgs[0]).buf[1]=0x00;

	(i2c_data.msgs[1]).len=2;
	(i2c_data.msgs[1]).addr=ADDR_SGTL5K;
	(i2c_data.msgs[1]).flags=I2C_M_RD;//read
	(i2c_data.msgs[1]).buf=(unsigned char*)malloc(2);
	(i2c_data.msgs[1]).buf[0]=0;
	(i2c_data.msgs[1]).buf[1]=0;

        ret=ioctl(fd,I2C_RDWR,(unsigned long)&i2c_data);
	if(ret<0){
		perror("ioctl error2");
	}

	unsigned short value;
	value = ((i2c_data.msgs[1]).buf[0])<< 8 | ((i2c_data.msgs[1]).buf[1]);
	printf("\n\rSGTL5000 CHIP_ID:%04X\n\r",value);
	close(fd);
	return 0;
}
