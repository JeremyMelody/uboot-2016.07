/********************************************************************
* 		copyright (C) 2014 all rights reserved
*			 @file: key_led.c
* 		  @Created: 2014-7-28 14:00
* 	  	   @Author: conway chen
* 	  @Description: test user keys and leds 
*	  @Modify Date: 2014-7-28 14:00
*********************************************************************/

#include<stdio.h>
#include<stdlib.h>
#include<unistd.h>
#include<sys/ioctl.h>
#include<sys/types.h>
#include<sys/stat.h>
#include<sys/select.h>
#include<sys/time.h>
#include<fcntl.h>
#include<errno.h>
#include<linux/input.h>

/**
 * @brief: main function  
 * @Param: argc: number of parameters
 * @Param: argv: parameters list
 */
int main(int argc, char *argv[])
{
	int fd,fd2;
	int key_value, i = 0, count;
	struct input_event ev_key;
	if (argc < 2) {
		fprintf(stdout, "Usage: [ key device ] \n");
		exit(0);	
	}
	fd = open(argv[1], O_RDWR);
	if (fd < 0) {
		perror("open device1 erro!\n");
		exit(1);
	}
	fd2 = open("/sys/class/leds/user_led0/brightness", O_RDWR);	
	if (fd2 < 0) {
		perror("open device2 error!\n");
		close(fd);
		exit(1);
	}
	for ( ; ; ) {
		count = read(fd, &ev_key, sizeof(struct input_event));		
		if (count > 0)
		if (EV_KEY == ev_key.type) {
			printf("type:%d, code:%d, value:%d\n", ev_key.type, ev_key.code, ev_key.value);
			if (0 == ev_key.value) {
				count = write(fd2, "1", 1);
				if (count < 0) 
					printf("write file1 fail!\n");
			} else {
				count = write(fd2, "0", 1);
				if (count < 0)
					printf("write file2 fail!\n");
			}	  
		}		
	}
	close(fd);
	close(fd2);
	return 0;
}

