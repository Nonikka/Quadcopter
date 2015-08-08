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

static char msg[] = "time is running out";
static int len;
int fd,Flag;
unsigned int TimeStart;
float Acceleration[3],AngleSpeed[3],Angle[3],Roll,Pitch,Yaw,DutyCycle[3];

int set_ticker(int n_msecs)  
{  
    struct itimerval new_timeset;  
    long n_sec, n_usecs;  
  
    n_sec = n_msecs / 1000;  
  
    n_usecs = (n_msecs % 1000) * 1000L;  
  
    new_timeset.it_interval.tv_sec = n_sec;  // set reload   设置初始间隔  
    new_timeset.it_interval.tv_usec = n_usecs; //new ticker value  
  
    new_timeset.it_value.tv_sec = n_sec;   //store this   设置重复间隔  
    new_timeset.it_value.tv_usec = n_usecs; //and this  
  
    return setitimer(ITIMER_REAL, &new_timeset, NULL);   
      //#include<sys/time.h>  getitimer(int which, struct itimerval *val)  
      //setitimer(int which, const struct itimerval* newval, struct itimerval *oldval);  
      //which 指定哪种计时器  ITMER_REAL, ITIMER_VIRTUAL, ITIMER_PROF.   
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

void countdown(int signum)  
{  
    int Num_Avail,i,all_count,t;
    unsigned int TimeNow;
    unsigned char Re_buf[11],counter,FirstBuf;
    unsigned char ucStra[6],ucStrw[6],ucStrAngle[6];
    char buffer[80];
    if ((Num_Avail = serialDataAvail (fd)) < 0)
    {
        fprintf (stderr, "Unable to open serial device: %s\n", strerror (errno)) ;
        return  ;
    }
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
    for(i = 0;i < Num_Avail;i++)
    //for(;;)
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
                printf("Num_Avail: %d",Num_Avail);
                printf("a:%.3f %.3f %.3f ",Acceleration[0],Acceleration[1],Acceleration[2]); 
                
                AngleSpeed[0] = ((short)(ucStrw[1]<<8| ucStrw[0]))/32768.0*2000;
                AngleSpeed[1] = ((short)(ucStrw[3]<<8| ucStrw[2]))/32768.0*2000;
                AngleSpeed[2] = ((short)(ucStrw[5]<<8| ucStrw[4]))/32768.0*2000;
                printf("w:%.3f %.3f %.3f  \n",AngleSpeed[0],AngleSpeed[1],AngleSpeed[2]); 

                Angle[0] = ((short)(ucStrAngle[1]<<8| ucStrAngle[0]))/32768.0*180;//Roll
                Angle[1] = ((short)(ucStrAngle[3]<<8| ucStrAngle[2]))/32768.0*180;//Pitch
                Angle[2] = ((short)(ucStrAngle[5]<<8| ucStrAngle[4]))/32768.0*180;//Yaw
                printf("A:%.2f %.2f %.2f\r\n",Angle[0],Angle[1],Angle[2]); 
                all_count++;
                printf("count: %d time: %d\n",all_count,TimeNow - TimeStart);
                printf("DutyCycle[0]: %.2f\n",DutyCycle[0]);
                fflush(stdout);
                serialFlush(fd);
                //break;
                return;
            }
        //write(STDERR_FILENO, msg, len);
        }
    }
} 

int main()  
{    
    len = strlen(msg);
    int PinNumber = 7;
    TimeStart = millis();
    if ((fd = serialOpen ("/dev/ttyAMA0", 115200)) < 0)
    {
        fprintf (stderr, "Unable to open serial device: %s\n", strerror (errno)) ;
        return 1 ;
    }
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
    delay(100);
    signal(SIGALRM, countdown);  
    //set_ticker(1000);
    if(set_ticker(10) == -1)  
        perror("set_ticker");  
    else  
        while(1)  
        {  
            //delay(10); //不知为何会影响计时器的循环？
            Calc_Dutycycle();
            softPwmWrite(PinNumber,DutyCycle[0]);
            //printf("DutyCycle[0]DutyCycle[0]DutyCycle[0]DutyCycle[0]: %d\n",DutyCycle[0]);
            //fflush(stdout);
        }  
    return 0;  
} 