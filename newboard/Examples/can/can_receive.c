/********************************************************************
* 		copyright (C) 2014 all rights reserved
*			 @file: can_r.c
* 		  @Created: 2014-8-6 16:30
* 	  	   @Author: conway chen
* 	  @Description: test CAN receiving datagram 
*	  @Modify Date: 2014-8-6 16:30
*********************************************************************/
#include <stdio.h> 
#include <stdlib.h> 
#include <string.h> 
#include <unistd.h> 
#include <net/if.h> 
#include <sys/ioctl.h> 
#include <sys/socket.h> 
#include <linux/can.h> 
#include <linux/can/raw.h> 
#include <getopt.h>

/**
 * @brief: print usage message 
 * @Param: stream: output device 
 * @Param: exit_code: error code which want to exit 
 */
void print_usage (FILE *stream, int exit_code)
{
    fprintf(stream, "Usage: option [ dev... ] \n");
    fprintf(stream,
            "\t-h  --help     Display this usage information.\n"
            "\t-d  --device   The device can[0-1]\n"
	    	"\t-i  --id		  Set the can id that want to receive\n");
    exit(exit_code);
}

/**
 * @brief: main function  
 * @Param: argc: number of parameters
 * @Param: argv: parameters list
 */
int main(int argc, char *argv[])  
{  
	int s, nbytes, i;
	char *device;
	int id, next_option, device_flag=0, id_flag=0;  
	struct sockaddr_can addr;  
	struct ifreq ifr;  
	struct can_frame frame;  
	struct can_filter rfilter[1];
	const char *const short_options = "hd:i:";
	const struct option long_options[] = {
		{ "help",   0, NULL, 'h'},
		{ "device", 1, NULL, 'd'},
		{ "id", 1, NULL, 'i'},
		{ NULL,     0, NULL, 0  }
	};
	
	while (1) {
		next_option = getopt_long (argc, argv, short_options, long_options, NULL);
		if (next_option < 0)
			break;
		switch (next_option) {
			case 'h':
				print_usage (stdout, 0);
				break;
			case 'd':
				device = optarg;
				device_flag = 1;
				break;
			case 'i':
				id = atoi(optarg);
				id_flag = 1;
				break;
			case '?':
				print_usage (stderr, 1);
				break;
			default:
				abort ();
		}
	}
	
	if (!device_flag) {
		print_usage (stdout, 0);
		exit(0);	
	}
	/* create a socket */  
	s = socket(PF_CAN, SOCK_RAW, CAN_RAW);
	strcpy(ifr.ifr_name, device);
	/* determine the interface index */  
	ioctl(s, SIOCGIFINDEX, &ifr);                    
	addr.can_family = AF_CAN;  
	addr.can_ifindex = ifr.ifr_ifindex;
	/* bind the socket to a CAN interface */    
	bind(s, (struct sockaddr *)&addr, sizeof(addr));
	
	if (id_flag) {     
		/* define the filter rules */   
		rfilter[0].can_id   = id;  
		rfilter[0].can_mask = CAN_SFF_MASK;  
		/* Set the filter rules */	  
		setsockopt(s, SOL_CAN_RAW, CAN_RAW_FILTER, &rfilter, sizeof(rfilter)); 
	}  
	while(1) {
		/* receive frame */  
		nbytes = read(s, &frame, sizeof(frame));            
		/* printf the received frame */  
		if (nbytes > 0) { 
			printf("%s  %#x  [%d]  ", ifr.ifr_name, frame.can_id, frame.can_dlc);
			for (i = 0; i < frame.can_dlc; i++)
				printf("%#x ", frame.data[i]); 
			printf("\n"); 
		}  
	}  
	close(s);  
	return 0;  
}  
