#include <unistd.h>
#include <stdio.h>
#include<string.h>
#include <fcntl.h>
#include <linux/fb.h>
#include <sys/mman.h>
#include <stdlib.h>

unsigned short rgb2lcd(unsigned char red, unsigned char green, unsigned char blue);
unsigned short change_rgb565(unsigned short rgb);
unsigned char rgb[3]={0,0,255};

int main()
{
	int fbfd = 0;
unsigned short lcd_data;
	struct fb_var_screeninfo vinfo;
	struct fb_fix_screeninfo finfo;
	long int screensize = 0;
	char *fbp = 0;
	int x = 0, y = 0;
	long int location = 0;
	int sav=0;

	/* open device*/
printf("1.\n");
	fbfd = open("/dev/fb0", O_RDWR);
printf("2.\n");
	if (!fbfd) {
		printf("Error: cannot open framebuffer device.\n");
		exit(1);
	}

	printf("The framebuffer device was opened successfully.\n");
 
	/* Get fixed screen information */
	if (ioctl(fbfd, FBIOGET_FSCREENINFO, &finfo)) {
		printf("Error reading fixed information.\n");
		exit(2);
	}

	/* Get variable screen information */
	
	if (ioctl(fbfd, FBIOGET_VSCREENINFO, &vinfo)) {
		printf("Error reading variable information.\n");
		exit(3);
	}
	
	/* show these information*/
	printf("vinfo.xres=%d\n",vinfo.xres);
	printf("vinfo.yres=%d\n",vinfo.yres);
	printf("vinfo.bits_per_bits=%d\n",vinfo.bits_per_pixel);//24
	printf("vinfo.xoffset=%d\n",vinfo.xoffset);
	printf("vinfo.yoffset=%d\n",vinfo.yoffset);
	printf("finfo.line_length=%d\n",finfo.line_length);
 
	/* Figure out the size of the screen in bytes */
	screensize = vinfo.xres * vinfo.yres * vinfo.bits_per_pixel/8;
 
 	/* Map the device to memory */
	fbp = (char *)mmap(0, screensize, PROT_READ | PROT_WRITE, MAP_SHARED, fbfd, 0); 
 	if ((int)fbp == -1) { 
		printf("Error: failed to map framebuffer device to memory.\n");
		exit(4);
	}

	printf("The framebuffer device was mapped to memory successfully.\n");
	memset(fbp,0,screensize);
	
	/* Where we are going to put the pixel */

	
	//lcd_data = rgb2lcd( rgb[0], rgb[1], rgb[2]);
	unsigned short color[8];

	int i;

	for(i=0;i<8;i++){
		color[i]=rgb2lcd((i&0x04)?255:0,(i&0x02)?255:0,(i&0x01)?255:0);

		for(x=0;x<vinfo.xres;x++){
		for(y=0;y<vinfo.yres;y++){
			location = (x+vinfo.xoffset)*(vinfo.bits_per_pixel/8) + (y+vinfo.yoffset)*finfo.line_length;
	 		//*(fbp+location) = 0xff; /* blue */
			//*(fbp+location+1) = 0x00;
			//*(fbp+location)= lcd_data&0x00FF;
			//*(fbp+location+1)=(lcd_data&0xFF00)>>8;
			*(fbp+location)= color[i]&0x00FF;
			*(fbp+location+1)=(color[i]&0xFF00)>>8;
			}
		}
	printf("tttttttt.\n");
		sleep(1);
	}

#if 0
	for(i=0;i<8;i++){
		color[i]=rgb2lcd((i&0x04)?255:0,(i&0x02)?255:0,(i&0x01)?255:0);
		for(x=i*(vinfo.xres/8);x<(i*(vinfo.xres/8)+vinfo.xres/8);x++){
		for(y=0;y<vinfo.yres;y++){
			location = (x+vinfo.xoffset)*(vinfo.bits_per_pixel/8) + (y+vinfo.yoffset)*finfo.line_length;
	 	unsigned short	//*(fbp+location) = 0xff; /* blue */
			//*(fbp+location+1) = 0x00;
			//*(fbp+location)= lcd_data&0x00FF;
			//*(fbp+location+1)=(lcd_data&0xFF00)>>8;

			//*(fbp+location)= color[i]&0x00FF;
			//*(fbp+location+1)=(color[i]&0xFF00)>>8;
			*(fbp+location)= j&0x00FF;
			*(fbp+location+1)=(j&0xFF00)>>8;
			j++;
			}
		}
	printf("tttttttt.\n");
	}
#endif

	int j=0;
	unsigned short jj=0;
	while(1){
		for(x=0;x<vinfo.xres;x++){
		for(y=0;y<vinfo.yres;y++){
			location = (x+vinfo.xoffset)*(vinfo.bits_per_pixel/8) + (y+vinfo.yoffset)*finfo.line_length;
			j=(0xFFFF/vinfo.xres)*x;
			jj=(unsigned short)change_rgb565(j);
			*(fbp+location)= jj&0x00FF;
			*(fbp+location+1)=(jj&0xFF00)>>8;
			
			}
		}
		sleep(1);
	}





	munmap(fbp, screensize); /* release the memory */
	
	while(1);
	close(fbfd);
	
	return 0;
}

unsigned short rgb2lcd(unsigned char red, unsigned char green, unsigned char blue)
{
	/*  R[0:4],G[5:10]B[11:15]  RGB565*/
	return ((((blue>>3)&0x001F)<<11) | (((green>>2)&0x003F)<<5) | ((red>>3)&0x001F) );
}

unsigned short change_rgb565(unsigned short rgb)
{
	unsigned short c=0;
	int i;

	//const int j[16]={5,0,6,11,1,7,12,2,8,13,3,9,14,4,10,15};
	const int j[16]={1,4,7,10,13,0,2,5,8,11,14,3,6,9,12,15};
	for(i=0;i<16;i++){
		c |=( ((rgb & (0x0001<<j[i]))?1:0)&0x0001)<<i;
	}
return c;





}
