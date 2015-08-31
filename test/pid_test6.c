#include "pca9685.h"
#include <wiringPi.h>
#include <pthread.h>
#include <sys/time.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include <string.h>
#include <errno.h>
#include <wiringSerial.h>
#include <math.h>
#include <wiringPiI2C.h>

#define PIN_BASE 300
#define HERTZ 500
#define PinNumber1 0  
#define PinNumber2 1  
#define PinNumber3 2  
#define PinNumber4 3  
#define IMU_UPDATE_DT 0.01
#define MAX_ACC 0.55

float Acceleration[3],AngleSpeed[3],Angle[3],Roll,Pitch,Yaw,DutyCycle[3],Accelerator,Pid_Pitch,Pid_Roll,Default_Acc = 0.35;
float error=0;
int all_count,Pid_Count=0;
float output=0,Pid_Roll=0;
float PidUpdate(/*pidsuite* pid,*/ const float measured,float expect,float gyro);
float PidUpdate_roll(/*pidsuite* pid,*/ const float measured,float expect,float gyro);
void PidInital();
void PWMOut(int pin, float pwm);

void gyro_acc()
{
    //float kp = 0.00375,ki = 0.0000,kd = 0.00076;
    float kp = 0.0,ki = 0.0,kd = 0.0;
    //0030 0088 0014 有偏角 p0.0031偏角更大 0.0029也是 i=0 小偏角 p0.00305 d0.00143 不错 i0.0005 偏角变大
    //0032 0017
    float pregyro =0;
    float desired = 0;
    //double error;
    float integ=0;//integral积分参数
    float iLimit =8 ;
    float deriv=0;//derivative微分参数 
    float prevError=0;
    float lastoutput=0;
    
    float Piddeadband=0.3;
    
    int fd,i=0;
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
                //Num_Avail = serialDataAvail (fd);
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
                //desired=expect;//获取期望角度
    
                error = desired - Angle[0];//偏差：期望-测量值
                error = error * 0.85 + prevError * 0.15;
                if (fabs(prevError - error ) > 12)
                {
                    error = prevError;
                }
                
                /*
                    if (fabs(error) >= 2.8)
                    {   
                        if (fabs(error) >= 3.9)
                        {
                            error = error * 1.04;
                        }
                        else
                        {
                            error = error * 1.02;
                        }
                    }
                */
                integ += error * IMU_UPDATE_DT;//偏差积分，IMU_UPDATE_DT也就是每调整漏斗大小的步辐
                
                if (integ >= iLimit)//作积分限制
                {
                  integ = iLimit;
                }
                else if (integ < -iLimit)
                {
                  integ = -iLimit;
                }
                
                deriv = (error - prevError) / IMU_UPDATE_DT;//微分     应该可用陀螺仪角速度代替
                //if(deriv
                //roll.deriv = -gyro;//注意是否跟自己的参数方向相反，不然会加剧振荡
                
                //deriv = -AngleSpeed[0];
                if (fabs(pregyro - deriv) > 20)
                {
                    deriv = deriv * 0.5 + pregyro * 0.5;
                }
                //if (fabs(pregyro - deriv ) > )
                /*
                if (fabs(deriv) > 15 && fabs(error) > 4)
                {
                    deriv = deriv * 0.8;
                }*/
                /*
                if (error < 2 )
                {
                    deriv = deriv * 1.2;
                }*/
                //if(fabs(error)>Piddeadband)//roll死区
                //{
                    //roll.outP = roll.kp * roll.error;//方便独立观察
                    //roll.outI = roll.ki * roll.integ;
                    //roll.outD = roll.kd * roll.deriv;
                output = (kp * error) + (ki * integ) + (kd * deriv);
                Pid_Roll = output;
                
                //}
                //else
                //{
                  //output=lastoutput;
                //}
                prevError = error;//更新前一次偏差
                pregyro = deriv;
                if (output >  0.16)
                {
                    output = 0.16;
                }
                if (output <  -0.16)
                {
                    output = -0.16;
                }
                //output = output * 0.9 + lastoutput * 0.1;
                if (fabs(error) < 0.3 )
                {
                    output =0;
                }
                lastoutput = output;
                
                //DutyCycle[0] = Default_Acc  - output;
                //DutyCycle[1] = Default_Acc  - output;
                DutyCycle[0] = Default_Acc;
                
                DutyCycle[1] = 0;
                //DutyCycle[2] = Default_Acc  + output;
                //DutyCycle[3] = Default_Acc  + output;
                DutyCycle[2] = 0;
                DutyCycle[3] = 0;
                PWMOut(PinNumber1,DutyCycle[0]);
                PWMOut(PinNumber2,DutyCycle[1]);
                PWMOut(PinNumber3,DutyCycle[2]);
                PWMOut(PinNumber4,DutyCycle[3]);
                Pid_Count ++ ;
                serialFlush(fd);
                break;
                }
                memset(Re_buf, 0, 11*sizeof(char));
                counter = 0;
            }
        }
    }
}

void KeyBoard()
{
    char keychar;
    while(1)
    {
        keychar = getchar();
        if (keychar == 'q')
        {
            Default_Acc += 0.01;
            if (Default_Acc > MAX_ACC) 
            {
                Default_Acc = MAX_ACC;
            }
        }
        else if (keychar == 'a')
        {
            Default_Acc -= 0.01;
            if (Default_Acc < 0.02)
            {
                Default_Acc = 0.02;
            }
        }
        else if (keychar == 'e')
        {
            Default_Acc = 0.05;
            delay(200);
            Default_Acc = 0.03;
            delay(200);
            PWMOut(PinNumber1,0.03);
            PWMOut(PinNumber2,0.03);
            PWMOut(PinNumber3,0.03);
            PWMOut(PinNumber4,0.03);
        }    
    }
}

void PWMOut(int pin, float pwm)//pwm valaue:0~1
{
    int outpwm = pwm * 2048 + 2048;
    //softPwmWrite(pin, (int)outpwm);
    pwmWrite(PIN_BASE + pin,outpwm);
}

int main()
{
    pthread_t mpu6050,transport;
    int ret;
    unsigned int TimeNow,TimeStart;
    if (-1 == wiringPiSetup())
    {
        printf("Setup WiringPi failed!");
        return 1;
    }
    int fd_pca9685 = pca9685Setup(PIN_BASE, 0x40, HERTZ);
	if (fd_pca9685 < 0)
	{
		printf("Error in setup\n");
		return fd_pca9685;
	}
    pca9685PWMReset(fd_pca9685);
    delay(100);
    /***********
    //启动方法1：最高油门确认
    PWMOut(PinNumber1,0.99);
    PWMOut(PinNumber2,0.99);
    PWMOut(PinNumber3,0.99);
    PWMOut(PinNumber4,0.99);
    printf("Way1:input to start ");
    getchar();
    PWMOut(PinNumber1,0.02);
    PWMOut(PinNumber2,0.02);
    PWMOut(PinNumber3,0.02);
    PWMOut(PinNumber4,0.02);
    delay(1200);
    PWMOut(PinNumber1,0.05);
    PWMOut(PinNumber2,0.05);
    PWMOut(PinNumber3,0.05);
    PWMOut(PinNumber4,0.05);
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
        printf ("Create mpu6050 thread error!\n");
        exit (1);
    }
    TimeStart = millis();
    ret = pthread_create(&transport,NULL, (void *)KeyBoard,NULL);
    if(ret!=0)
    {
        printf ("Create KeyBoard thread error!\n");
        exit (1);
    }
    
    //PidInital();
    while(1)  
    {  
        //Pid_Pitch = PidUpdate(Angle[1],-1.6,AngleSpeed[1]);
        //Pid_Roll = PidUpdate_roll(Angle[0],0,AngleSpeed[0]);
        TimeNow = millis();
        system("clear");
        //delay(100);
        printf("Pid_Roll:%.3f %.3f  error %.3f  Pid_Count: %d time:%d\n",output,Pid_Roll,error,Pid_Count,TimeNow - TimeStart);
        printf("A:%.2f %.2f %.2f\n",Angle[0],Angle[1],Angle[2]); 
        printf("Default_Acc:%.2f gyro：Pitch：%.2f roll :%.2f\n",Default_Acc,AngleSpeed[1],AngleSpeed[0]); 
        fflush(stdout);
        //DutyCycle[3] = Default_Acc + Pid_Pitch - Pid_Roll;//+yaw
        //DutyCycle[2] = Default_Acc - Pid_Pitch + Pid_Roll;//+yaw
        //DutyCycle[1] = Default_Acc + Pid_Pitch + Pid_Roll;//-yaw
        //DutyCycle[0] = Default_Acc - Pid_Pitch - Pid_Roll;//-yaw
        
        
    }
}