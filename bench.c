#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdlib.h>
extern unsigned usleep(unsigned usec);
#include <sys/ioctl.h>
#include "common.h"

void my_list(){
	for(int i=0;i<COUNT(Devices);i++)
		printf("%2i:%s%i\n",i,NAME(i));
}
int my_open (int*fd,int*dev,int num){
	if(*fd>=0)
		return fprintf(stderr,"%s:Already opened\n",__FUNCTION__);
	if(num>COUNT(Devices))
		return fprintf(stderr,"%s:Out of range\n",__FUNCTION__);
	
	char buffer[32];
	sprintf(buffer,MY_PATH"%s%d",NAME(num));
	if((*fd=open(buffer,O_RDWR))>0)
		*dev=num;
	
	return fprintf(stderr,"%s(%s) : %s\n",__FUNCTION__,buffer,*fd>0?"ok":"fail");
}
int my_close(int*fd,int*dev){
	if(*fd<0)
		return fprintf(stderr,"%s:nothing openend\n",__FUNCTION__) ;
	if(close(*fd)>=0)//close ok
		*fd=*dev=-1;
	return fprintf(stderr,"%s : %s\n",__FUNCTION__,*fd<0?"ok":"fail");
}
int my_read (int*fd,int*dev){
	if(*fd<0)
		return fprintf(stderr,"%s:nothing openend\n",__FUNCTION__) ;
	
	int ret;unsigned char val;
	if((ret=read(*fd,&val,sizeof(val)))!=sizeof(val))
		return fprintf( stderr, "%s(%s%i):error %i\n",__FUNCTION__,NAME(*dev),ret);
		
	fprintf( stdout, "%s(%s%i) => %i (%#08x)\n",__FUNCTION__,NAME(*dev),ret,val);
	if(Devices[*dev].type == ANL)
		fprintf(stdout,"En Volt : %.3fV\n",(5*(float)val)/1023);//recu:[0-1023] reel:[0V - 5V]
	return val;
}
int my_write(int*fd,int*dev,DeviceState state){
	if(*fd<0)
		return fprintf(stderr,"%s:nothing openend\n",__FUNCTION__) ;

	return fprintf(stderr,"%s(%s%i) => %s\n",__FUNCTION__,NAME(*dev),write(*fd,&state,sizeof(char))!=sizeof(char)?"error":"ok");
}
int my_ioctl(int*fd,int*dev,int cmd,int val){
	if(*fd<0)
		return fprintf(stderr,"%s:nothing openend\n",__FUNCTION__) ;
	return fprintf(stdout,"%s(%s%i,%i,%i) => %i\n",__FUNCTION__, NAME(*dev),cmd,val, ioctl(*fd,cmd,val) );
}

int main(int argc,char** argv){
	int fd=-1,dev=-1;
	for(int i=1;i<argc;i++){
		switch(argv[i][0]){
			case 'l':my_list ();break;
			case 'o':my_open (&fd,&dev,atoi(argv[++i]));break;
			case 'w':my_write(&fd,&dev,atoi(argv[++i])?ON:OFF);break;
			case 'r':my_read (&fd,&dev);break;
			case 'c':my_close(&fd,&dev);break;
			case 'i':my_ioctl(&fd,&dev,atoi(argv[++i]),atoi(argv[++i]));break;
			case 'd':case 's':usleep(1000*atoi(argv[++i]));break;
			default :break;
		}
	}
	return 0;
}