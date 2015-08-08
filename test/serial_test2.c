#include <sys/time.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include <string.h>
#include <errno.h>
#include <wiringPi.h>
#include <wiringSerial.h>
#include <softPwm.h>
#include <math.h>

int main (){
    int fd,i,Flag,FirstBuf,t;
    int Num_Avail;
    char buffer[80];
    unsigned int TimeNow,TimeStart;
    unsigned char counter;
    int all_count=0;
    unsigned char ucStra[6],ucStrw[6],ucStrAngle[6];
    float Value[3];
    if ((fd = serialOpen ("/dev/ttyAMA0", 115200)) < 0)
    {
        fprintf (stderr, "Unable to open serial device: %s\n", strerror (errno)) ;
        return 1 ;
    }
    softPwmCreate(7,0,20);
    TimeStart = millis();
    for(;;)
    {   
        if (Flag == 0)
        {
            Flag = 1;
            while(1)
            {
                FirstBuf = serialGetchar(fd);
                if(FirstBuf == 0x55)
                {
                    for(t = 0;t < 10;t++)
                    {
                        FirstBuf = serialGetchar(fd);
                    }
                    break;
                }
            }
        }
        serialFlush(fd);
        delay(10);
        Num_Avail = serialDataAvail (fd);
        read(fd,buffer,Num_Avail);
        
        for(counter;counter < strlen(buffer) - 7;counter++)
        {
            if(buffer[counter]==0x55)
            {    
                switch(buffer [counter + 1])
                {
                    case 0x51:
                    ucStra[0]=buffer[counter + 2];
                    ucStra[1]=buffer[counter + 3];
                    ucStra[2]=buffer[counter + 4];
                    ucStra[3]=buffer[counter + 5];
                    ucStra[4]=buffer[counter + 6];
                    ucStra[5]=buffer[counter + 7];
                    break;
                    case 0x52:     
                    ucStrw[0]=buffer[counter + 2];
                    ucStrw[1]=buffer[counter + 3];
                    ucStrw[2]=buffer[counter + 4];
                    ucStrw[3]=buffer[counter + 5];
                    ucStrw[4]=buffer[counter + 6];
                    ucStrw[5]=buffer[counter + 7];
                    break;
                    case 0x53: 
                    ucStrAngle[0]=buffer[counter + 2];
                    ucStrAngle[1]=buffer[counter + 3];
                    ucStrAngle[2]=buffer[counter + 4];
                    ucStrAngle[3]=buffer[counter + 5];
                    ucStrAngle[4]=buffer[counter + 6];
                    ucStrAngle[5]=buffer[counter + 7];
                    TimeNow = millis();
                    Value[0] = ((short)(ucStra[1]<<8| ucStra[0]))/32768.0*16;
                    Value[1] = ((short)(ucStra[3]<<8| ucStra[2]))/32768.0*16;
                    Value[2] = ((short)(ucStra[5]<<8| ucStra[4]))/32768.0*16;
                    system("clear");
                    
                    printf("Num_Avail; %d",Num_Avail);
                    printf("a:%.3f %.3f %.3f  ",Value[0],Value[1],Value[2]); 
                    
                    Value[0] = ((short)(ucStrw[1]<<8| ucStrw[0]))/32768.0*2000;
                    Value[1] = ((short)(ucStrw[3]<<8| ucStrw[2]))/32768.0*2000;
                    Value[2] = ((short)(ucStrw[5]<<8| ucStrw[4]))/32768.0*2000;
                    printf("w:%.3f %.3f %.3f  \n",Value[0],Value[1],Value[2]); 

                    Value[0] = ((short)(ucStrAngle[1]<<8| ucStrAngle[0]))/32768.0*180;
                    Value[1] = ((short)(ucStrAngle[3]<<8| ucStrAngle[2]))/32768.0*180;
                    Value[2] = ((short)(ucStrAngle[5]<<8| ucStrAngle[4]))/32768.0*180;
                    printf("A:%.2f %.2f %.2f\r\n",Value[0],Value[1],Value[2]); 
                    all_count++;
                    printf("count: %d time: %d\n",all_count,TimeNow - TimeStart);
                    break;
                }
            }
        }
    }
}