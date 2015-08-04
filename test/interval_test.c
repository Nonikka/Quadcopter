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

/*int getitimer(int which, struct itimerval *value);
int setitimer(int which, struct itimerval*newvalue, struct itimerval* oldvalue);
struct timeval
{
long tv_sec; //秒
long tv_usec; //微秒
};
struct itimerval
{
struct timeval it_interval; //时间间隔
struct timeval it_value;   //当前时间计数
};*/

static char msg[] = "time is running out";
static int len;
// 向标准错误输出信息，告诉用户时间到了
static DutyCycle[3];

void prompt_info(int signo,int fd)
{
    int Num_Avail;
    unsigned int TimeNow,TimeStart;
    unsigned char Re_buf[11],counter;
    int all_count;
    unsigned char ucStra[6],ucStrw[6],ucStrAngle[6];
    float Value[3];
    Num_Avail = serialDataAvail(fd);
    for(i = 0;i < Num_Avail;i++)
    {
        Re_buf[counter]=serialGetchar(fd);
            //printf("%x\t",Re_buf[counter]&0xff);
            //printf("%x\t",Re_buf[counter]);
            counter++; 
            if(counter==0&&Re_buf[0]!=0x55) {counter--;}//第0号数据不是帧头
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
                Value[0] = ((short)(ucStra[1]<<8| ucStra[0]))/32768.0*16;
                Value[1] = ((short)(ucStra[3]<<8| ucStra[2]))/32768.0*16;
                Value[2] = ((short)(ucStra[5]<<8| ucStra[4]))/32768.0*16;
                system("clear");
                printf("a:%.3f %.3f %.3f  ",Value[0],Value[1],Value[2]); 
                
                Value[0] = ((short)(ucStrw[1]<<8| ucStrw[0]))/32768.0*2000;
                Value[1] = ((short)(ucStrw[3]<<8| ucStrw[2]))/32768.0*2000;
                Value[2] = ((short)(ucStrw[5]<<8| ucStrw[4]))/32768.0*2000;
                printf("w:%.3f %.3f %.3f  ",Value[0],Value[1],Value[2]); 

                Value[0] = ((short)(ucStrAngle[1]<<8| ucStrAngle[0]))/32768.0*180;
                Value[1] = ((short)(ucStrAngle[3]<<8| ucStrAngle[2]))/32768.0*180;
                Value[2] = ((short)(ucStrAngle[5]<<8| ucStrAngle[4]))/32768.0*180;
                printf("A:%.2f %.2f %.2f\r\n",Value[0],Value[1],Value[2]); 
                all_count++;
                printf("count: %d time: %d",all_count,TimeNow - TimeStart);
                break;
    }
    //write(STDERR_FILENO, msg, len);
}
// 建立信号处理机制
void init_sigaction(void)
{
struct sigaction tact;
/*信号到了要执行的任务处理函数为prompt_info*/
tact.sa_handler = prompt_info(fd));
tact.sa_flags = 0;
/*初始化信号集*/
sigemptyset(&tact.sa_mask);
/*建立信号处理机制*/
sigaction(SIGALRM, &tact, NULL);
}
void init_time()
{
struct itimerval value;
/*设定执行任务的时间间隔为0秒10000微秒*/
value.it_value.tv_sec = 0;
value.it_value.tv_usec = 10000;
/*设定初始时间计数也为0秒10000微秒*/
value.it_interval = value.it_value;
/*设置计时器ITIMER_REAL*/
setitimer(ITIMER_REAL, &value, NULL);
}
int main(void)
{
    int DutyCycle;
    len = strlen(msg);
    init_sigaction();
    init_time();
    int level;
    int PinNumber = 7;
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
    delay(1000);
    softPwmWrite(PinNumber,12.6);
    printf("start!")
    while(1)
        {
        softPwmWrite(PinNumber,DutyCycle[0]);
        printf("DutyCycle[0]: %d",DutyCycle[0]);
        }
    exit(0);
}