#include <pthread.h>
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

float Acceleration[3],AngleSpeed[3],Angle[3],Roll,Pitch,Yaw,DutyCycle[3];

void gyro_acc()
{
    int fd,i;
    int Num_Avail;
    unsigned int TimeNow,TimeStart;
    unsigned char Re_buf[11],counter;
    int all_count;
    unsigned char ucStra[6],ucStrw[6],ucStrAngle[6];
    if ((fd = serialOpen ("/dev/ttyAMA0", 115200)) < 0)
    {
        fprintf (stderr, "Unable to open serial device: %s\n", strerror (errno)) ;
        return  ;
    }
    TimeStart = millis();
    for(;;)
    {
        Re_buf[counter]=serialGetchar(fd);
        
        if(counter==0&&Re_buf[0]!=0x55) {return;}//第0号数据不是帧头
        counter++;
        if(counter==11)             //接收到11个数据
        {    
            counter=0;               //重新赋值，准备下一帧数据的接收        
            switch(Re_buf [1])
            {
            case 0x51:
            ucStra[0]=Re_buf[2];
            ucStra[1]=Re_buf[3];
            ucStra[2]=Re_buf[4];
            ucStra[3]=Re_buf[5];
            ucStra[4]=Re_buf[6];
            ucStra[5]=Re_buf[7];
            break;
            case 0x52:     
            ucStrw[0]=Re_buf[2];
            ucStrw[1]=Re_buf[3];
            ucStrw[2]=Re_buf[4];
            ucStrw[3]=Re_buf[5];
            ucStrw[4]=Re_buf[6];
            ucStrw[5]=Re_buf[7];
            break;
            case 0x53: 
            ucStrAngle[0]=Re_buf[2];
            ucStrAngle[1]=Re_buf[3];
            ucStrAngle[2]=Re_buf[4];
            ucStrAngle[3]=Re_buf[5];
            ucStrAngle[4]=Re_buf[6];
            ucStrAngle[5]=Re_buf[7];
            TimeNow = millis();
            Acceleration[0] = ((short)(ucStra[1]<<8| ucStra[0]))/32768.0*16;
            Acceleration[1] = ((short)(ucStra[3]<<8| ucStra[2]))/32768.0*16;
            Acceleration[2] = ((short)(ucStra[5]<<8| ucStra[4]))/32768.0*16;
            system("clear");
            Num_Avail = serialDataAvail (fd);
            printf("Num_Avail; %d",Num_Avail);
            printf("a:%.3f %.3f %.3f  ",Acceleration[0],Acceleration[1],Acceleration[2]); 
            
            AngleSpeed[0] = ((short)(ucStrw[1]<<8| ucStrw[0]))/32768.0*2000;
            AngleSpeed[1] = ((short)(ucStrw[3]<<8| ucStrw[2]))/32768.0*2000;
            AngleSpeed[2] = ((short)(ucStrw[5]<<8| ucStrw[4]))/32768.0*2000;
            printf("w:%.3f %.3f %.3f  \n",AngleSpeed[0],AngleSpeed[1],AngleSpeed[2]); 

            Angle[0] = ((short)(ucStrAngle[1]<<8| ucStrAngle[0]))/32768.0*180;
            Angle[1] = ((short)(ucStrAngle[3]<<8| ucStrAngle[2]))/32768.0*180;
            Angle[2] = ((short)(ucStrAngle[5]<<8| ucStrAngle[4]))/32768.0*180;
            printf("A:%.2f %.2f %.2f\r\n",Angle[0],Angle[1],Angle[2]); 
            all_count++;
            printf("count: %d time: %d\n",all_count,TimeNow - TimeStart);
            printf("DutyCycle[0]: %.2f\n",DutyCycle[0]);
            serialFlush(fd);
            break;
            }
        }
    }
}

void Calc_Dutycycle()
{
    if (Angle[0] <= 180&& Angle[0]>=0)
    {
        Roll = Angle[0];
    }
    else 
    {
        Roll = abs(Angle[0]);
    }
    DutyCycle[0] = (Roll/180)*8 + 11;
}

int main()
{
    pthread_t mpu6050;
    int PinNumber = 7,ret;
    if (-1 == wiringPiSetup())
    {
        printf("Setup WiringPi failed!");
        return 1;
    }
    softPwmCreate(PinNumber,0,20);//20*100=2000microseconds=500hz
    softPwmWrite(PinNumber,19);
    printf("96now\n");
    getchar();
    softPwmWrite(PinNumber,11);
    printf("54now\n");
    fflush(stdout);
    delay(800);
    softPwmWrite(PinNumber,12.6);
    printf("start!");
    fflush(stdout);
    ret = pthread_create(&mpu6050,NULL, (void *)gyro_acc,NULL);
    if(ret!=0)
    {
        printf ("Create pthread error!\n");
        exit (1);
    }
    while(1)  
    {  
        Calc_Dutycycle();
        softPwmWrite(PinNumber,DutyCycle[0]);
        //printf("DutyCycle: %d\n",DutyCycle[0]);
        //fflush(stdout);
    }  
}

