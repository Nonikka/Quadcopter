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

#define PinNumber1 7  //对应7
#define PinNumber2 0  //对应11
#define PinNumber3 2  //对应13
#define PinNumber4 3  //对应15
#define IMU_UPDATE_DT 10
#define DEFAULT_POW 0.26

float Acceleration[3],AngleSpeed[3],Angle[3],Roll,Pitch,Yaw,DutyCycle[3],TARGET,Pid_Pitch;
int all_count;

float PidUpdate(/*pidsuite* pid,*/ const float measured,float expect,float gyro);
void PidInital();

void gyro_acc()
{
    int fd,i;
    int Num_Avail;
    unsigned int TimeNow,TimeStart;
    unsigned char Re_buf[11],counter;
    unsigned char ucStra[6],ucStrw[6],ucStrAngle[6];
    if ((fd = serialOpen ("/dev/ttyAMA0", 115200)) < 0)
    {
        fprintf (stderr, "Unable to open serial device: %s\n", strerror (errno)) ;
        return  ;
    }
    TimeStart = millis();
    delay(50);
    for(;;)
    {
        Re_buf[counter]=serialGetchar(fd);
        if(Re_buf[0]!=0x55) 
        {
            memset(Re_buf, 0, 11*sizeof(char));
            counter = 0;
        }
        else
        {
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
                //system("clear");
                Num_Avail = serialDataAvail (fd);
                //printf("Num_Avail; %d",Num_Avail);
                //printf("a:%.3f %.3f %.3f  ",Acceleration[0],Acceleration[1],Acceleration[2]); 
                
                AngleSpeed[0] = ((short)(ucStrw[1]<<8| ucStrw[0]))/32768.0*2000;
                AngleSpeed[1] = ((short)(ucStrw[3]<<8| ucStrw[2]))/32768.0*2000;
                AngleSpeed[2] = ((short)(ucStrw[5]<<8| ucStrw[4]))/32768.0*2000;
                //printf("w:%.3f %.3f %.3f  \n",AngleSpeed[0],AngleSpeed[1],AngleSpeed[2]); 

                Angle[0] = ((short)(ucStrAngle[1]<<8| ucStrAngle[0]))/32768.0*180;
                Angle[1] = ((short)(ucStrAngle[3]<<8| ucStrAngle[2]))/32768.0*180;
                Angle[2] = ((short)(ucStrAngle[5]<<8| ucStrAngle[4]))/32768.0*180;
                //printf("A:%.2f %.2f %.2f\r\n",Angle[0],Angle[1],Angle[2]); 
                all_count++;
                //printf("count:%d\t  angle:%f\t  time:",all_count, Angle[1], clock());
                //printf("count: %d time: %d\n",all_count,TimeNow - TimeStart);
                //printf("DutyCycle[0]: %.2f\n",DutyCycle[0]);
                serialFlush(fd);
                break;
                }
                memset(Re_buf, 0, 11*sizeof(char));
                counter = 0;
            }
        }
    }
}

void PWMOut(int pin, float pwm)//pwm valaue:0~1
{
    float outpwm = pwm * 7.8 + 11.2;//(20-11.2-1)
    softPwmWrite(pin, (int)outpwm);
}

int main()
{
    pthread_t mpu6050;
    int ret;
    if (-1 == wiringPiSetup())
    {
        printf("Setup WiringPi failed!");
        return 1;
    }
    softPwmCreate(PinNumber1,0,20);//20*100=2000microseconds=500hz
    softPwmCreate(PinNumber2,0,20);
    softPwmCreate(PinNumber3,0,20);
    softPwmCreate(PinNumber4,0,20);
    /***********
    //启动方法1：最高油门确认
    softPwmWrite(PinNumber1,19);
    softPwmWrite(PinNumber2,19);
    softPwmWrite(PinNumber3,19);
    softPwmWrite(PinNumber4,19);
    printf("Way1:input to start ");
    getchar();
    softPwmWrite(PinNumber1,11.2);
    softPwmWrite(PinNumber2,11.2);
    softPwmWrite(PinNumber3,11.2);
    softPwmWrite(PinNumber4,11.2);
    delay(1000);
    softPwmWrite(PinNumber1,13.5);
    softPwmWrite(PinNumber2,13.5);
    softPwmWrite(PinNumber3,13.5);
    softPwmWrite(PinNumber4,13.5);
    printf("start!");
    fflush(stdout);
    ***************/
    
    //启动方法2：最低油门拉起
    printf("Way 2:PWM in 0\% \n");
    PWMOut(PinNumber1,0);
    PWMOut(PinNumber2,0);
    PWMOut(PinNumber3,0);
    PWMOut(PinNumber4,0);
    printf("input to start!\n");
    fflush(stdout);
    getchar();
    PWMOut(PinNumber1,0.06);
    PWMOut(PinNumber2,0.06);
    PWMOut(PinNumber3,0.06);
    PWMOut(PinNumber4,0.06);
    delay(500);
    /*********************/
    ret = pthread_create(&mpu6050,NULL, (void *)gyro_acc,NULL);
    if(ret!=0)
    {
        printf ("Create pthread error!\n");
        exit (1);
    }
    PidInital();
    while(1)  
    {  
        Pid_Pitch = PidUpdate(Angle[1],0,AngleSpeed[1]);
        
        system("clear");
        //delay(100);
        printf("Pid_Pitch: %.2f all_count: %d\n",Pid_Pitch,all_count);
        printf("A:%.2f %.2f %.2f\n",Angle[0],Angle[1],Angle[2]); 
        printf("gyro：Pitch：%.2f\n",AngleSpeed[1]); 
        fflush(stdout);
        DutyCycle[3] = DEFAULT_POW + Pid_Pitch;
        DutyCycle[2] = DEFAULT_POW - Pid_Pitch;
        DutyCycle[1] = DEFAULT_POW + Pid_Pitch;
        DutyCycle[0] = DEFAULT_POW - Pid_Pitch;
        PWMOut(PinNumber1,DutyCycle[1]);
        PWMOut(PinNumber2,DutyCycle[0]);
        PWMOut(PinNumber3,DutyCycle[3]);
        PWMOut(PinNumber4,DutyCycle[2]);
        
    }
}

struct PID{  
        double kp ;//proportionalgain调整比例参数 
        double ki;   //integral gain调整积分参数 
        double kd ;   //derivative gain调整微分参数 
        double desired;
        double error;
        double integ;//integral积分参数
        double iLimit ;
        double deriv;//derivative微分参数 
        double prevError;
        double outP;
        double outI;
        double outD;
        double lastoutput;
}pid/*,_Pitch,_Yaw*/;  

void PidInital()
{
  pid.kp = 0.0031;// p = 0.0029,d = 0.001可进行振荡 可能有点大，但是低角度回复又有点不足，最多有10度左右偏移
  pid.ki = 0.00;
  pid.kd = 0.001; //0.001可以进行有效抑制振荡 但是可能造成回复不足
  pid.iLimit = 0.00;
  pid.lastoutput = 0.00;
}

float PidUpdate(/*pidsuite* pid,*/ const float measured,float expect,float gyro)
{
    float output;
    float Piddeadband=0.3; //就是PID死区，不进行转速更改的区域
    
    pid.desired=expect;//获取期望角度
    //pid.desired=0;
    pid.error = pid.desired - measured;//偏差：期望-测量值
    
    pid.integ += pid.error * IMU_UPDATE_DT;//偏差积分，IMU_UPDATE_DT也就是每调整漏斗大小的步辐
   
    if (pid.integ >= pid.iLimit)//作积分限制
    {
      pid.integ = pid.iLimit;
    }
    else if (pid.integ < -pid.iLimit)
    {
      pid.integ = -pid.iLimit;
    }

    // pid.deriv = (pid.error - pid.prevError) / IMU_UPDATE_DT;//微分     应该可用陀螺仪角速度代替
    //pid.deriv = -gyro;//注意是否跟自己的参数方向相反，不然会加剧振荡
    pid.deriv = -gyro;
    if(fabs(pid.error)>Piddeadband)//pid死区
    {
        pid.outP = pid.kp * pid.error;//方便独立观察
        pid.outI = pid.ki * pid.integ;
        pid.outD = pid.kd * pid.deriv;
        output = (pid.kp * pid.error) + (pid.ki * pid.integ) + (pid.kd * pid.deriv);
    }
    else
    {
      output=pid.lastoutput;
    }

    pid.prevError = pid.error;//更新前一次偏差
    
    if (output >  0.2)
    {
        output = 0.2;
    }
    if (output <  -0.2)
    {
        output = -0.2;
    }
    pid.lastoutput=output;
    return output;
}