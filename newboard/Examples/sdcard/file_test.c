#include<unistd.h>
#include<sys/types.h>
#include<sys/stat.h>
#include<fcntl.h>
#include<stdio.h>

int main(){

	int fdsrc,fddes,nbytes;
	int flags=O_CREAT | O_TRUNC | O_WRONLY;
	int z;
	//char buf[20], des[20];
	char buf[]="Make Your idea Real!";
	char des[]="MYiR.txt";
	char file_path[20];
	printf("Enter SD path:");
	scanf("%s", file_path);                 		/*读入目标文件名*/

	//fdsrc=open("/etc/passwd",O_RDONLY);
	fdsrc=open(file_path, O_RDONLY);
	if(fdsrc<0){
		printf("open source file error!");
		exit(1);
	}
	fddes=open(des,flags,0644);
	if(fddes<0){
		printf("open destination file error!");
		exit(1);
	}
	//while((nbytes=read(fdsrc,buf,20))>0)
	{
		z=write(fddes,buf,strlen(buf));
		if(z<0){
			perror("write file error");
		}
	}
	close(fddes);
	close(fdsrc);
	printf("Write OK\n");
	exit(0);
}
