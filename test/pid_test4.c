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
#define IMU_UPDATE_DT 0.0143
#define MAX_ACC 0.58

float Acceleration[3],AngleSpeed[3],Angle[3],Roll,Pitch,Yaw,DutyCycle[3],Accelerator,Pid_Pitch,Pid_Roll,Default_Acc = 0.38;
int all_count,Pid_Count;
float output;
float PidUpdate(/*pidsuite* pid,*/ const float measured,float expect,float gyro);
float PidUpdate_roll(/*pidsuite* pid,*/ const float measured,float expect,float gyro);
void PidInital();
void PWMOut(int pin, float pwm);

void gyro_acc()
{
    double kp = 0.0030,ki = 0.0088,kd = 0.0014;
    double pregyro ;
    double desired;
    double error;
    double integ;//integral积分参数
    double iLimit ;
    double deriv;//derivative微分参数 
    double prevError;
    double lastoutput;
    
    float Piddeadband=0.3;
    
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
                //desired=expect;//获取期望角度
    
                error = desired - Angle[0];//偏差：期望-测量值
                
                integ += error * IMU_UPDATE_DT;//偏差积分，IMU_UPDATE_DT也就是每调整漏斗大小的步辐
               
                if (integ >= iLimit)//作积分限制
                {
                  integ = iLimit;
                }
                else if (integ < -iLimit)
                {
                  integ = -iLimit;
                }
                
                // roll.deriv = (roll.error - roll.prevError) / IMU_UPDATE_DT;//微分     应该可用陀螺仪角速度代替
                //roll.deriv = -gyro;//注意是否跟自己的参数方向相反，不然会加剧振荡
                deriv = -AngleSpeed[0];
                if(fabs(error)>Piddeadband)//roll死区
                {
                    //roll.outP = roll.kp * roll.error;//方便独立观察
                    //roll.outI = roll.ki * roll.integ;
                    //roll.outD = roll.kd * roll.deriv;
                    output = (kp * error) + (ki * integ) + (kd * deriv);
                }
                else
                {
                  output=lastoutput;
                }

                prevError = error;//更新前一次偏差
                
                if (output >  0.2)
                {
                    output = 0.2;
                }
                if (output <  -0.2)
                {
                    output = -0.2;
                }
                lastoutput=output;
                DutyCycle[3] = Default_Acc  - output;
                DutyCycle[2] = Default_Acc  + output;
                DutyCycle[1] = Default_Acc  + output;
                DutyCycle[0] = Default_Acc  - output;
                PWMOut(PinNumber1,DutyCycle[1]);
                PWMOut(PinNumber2,DutyCycle[0]);
                PWMOut(PinNumber3,DutyCycle[3]);
                PWMOut(PinNumber4,DutyCycle[2]);
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
            Default_Acc = 0.02;
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
    pthread_t mpu6050,transport;
    int ret;
    unsigned int TimeNow,TimeStart;
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
    /***************/
    
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
        printf("Pid_Pitch: %.2f Pid_Roll:%.2f all_count: %d\n Pid_Count: %d time:%d\n",Pid_Pitch,Pid_Roll,all_count,Pid_Count,TimeNow - TimeStart);
        printf("A:%.2f %.2f %.2f\n",Angle[0],Angle[1],Angle[2]); 
        printf("Default_Acc:%.2f gyro：Pitch：%.2f\n",Default_Acc,AngleSpeed[1]); 
        fflush(stdout);
        //DutyCycle[3] = Default_Acc + Pid_Pitch - Pid_Roll;//+yaw
        //DutyCycle[2] = Default_Acc - Pid_Pitch + Pid_Roll;//+yaw
        //DutyCycle[1] = Default_Acc + Pid_Pitch + Pid_Roll;//-yaw
        //DutyCycle[0] = Default_Acc - Pid_Pitch - Pid_Roll;//-yaw
        
        
    }
}


/*
struct PID{  
        double kp ;//proportionalgain调整比例参数 
        double ki;   //integral gain调整积分参数 
        double kd ;   //derivative gain调整微分参数 
        double pregyro ;
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
}pid,roll/*,_Pitch,_Yaw*/;  
/*
void PidInital()
{
  pid.kp = 0.0031;// p = 0.0029,d = 0.001可进行振荡 可能有点大，但是低角度回复又有点不足，最多有10度左右偏移 
                  // p = 0.0031 d = 0.001 会有超调
  pid.ki = 0.00;
  pid.kd = 0.00216; //0.001可以进行有效抑制振荡 但是可能造成回复不足
  pid.iLimit = 0.00;
  
  pid.lastoutput = 0.00;
  roll.kp = 0.0030;
  roll.ki = 0.0088;
  roll.kd = 0.0014; //0.0032，0.00133在悬挂电池时超调  0.0015时在32%油门时超调 0.0025未知震荡  0.0035可能是太大了，过于灵敏0.0018发散 0.0021接近等幅 0.0022有平稳征兆，不过电池没电不知是否被限制了功率 0.0023在新电池下发散振荡  0.0026在新电池下发散振荡 0.0032在新电池下发散振荡
  //换p=0.0025 d=0.0032 依然发散 d=0.006 剧烈发散 0.011不行  0.001接近但是发散 可能是绳子的原因 0.0012发散 0.0015接近不怎么发散 可能是回复力太大
  //换p=0.0023 还是轻微发散 油门32%测试：p0.0022 d0.00155 发散。  0.0021 0.0016：发散 但是不错了，可能是d太大了？ 0.0018，不懂了
  //0.0020 0.00185 完全不行 0.0022 0.0018 估计在附近  0.0022 0.0017不行 0.0016 还需改进 0.0015 不行 0.155差不多？
  //0.00215 0.00155 可能比上一个差点0.00218 0.00156 不行
  //0.0022回复不足 改0.0028不足  0.0032 0.00152 回复不足 0.0036 0.00155 回复可能够了 就是d可能不够 0.0035 0.00159几乎是等幅振荡0.00162 可能还是不够
  //0.0035 0.00165 还是有振荡 而且可能是反了 0.0017估计是太大了 导致回复不足 0.00162不够大 0.00164可能也不够 
  //0.168太大0.163标记0.161标记0.16标记0.159可能不行 0.158不行0.1615不行1625不行0.163加剧 0.165可能太小 改0.0034 0.165不行 0.32不行 
  //0.0031 0.00166有点回复不足但是振荡小点0.0032 0.00169目前最好 就是有偏角 0.00325 0.001717 有振荡 要调 有偏角
  //0.002 0 太小 0.0025小
  //00325 008 001717 可能d太大 偏角回不来 d0017偏角 i0085 好了一点但是回复可能不够 p0033 表现不错 但是感觉i太大 回复太久于是超调 i0.0825不错
  //认为i不该有太长作用时间，改iLimit20->12 大偏角不回 改15 d0.0016 无效  00825改00855 可参考00165 差不多
  //0033 00885 00165 iLimit 15 目前表现最好  0033 0088 0015也是差不多
  //d001偏角 i改0.008震荡0.007震荡 p0.0035 有突然的加速导致超调 d0.0012no i0.008no
  //0030 0088 0015参考 想让i的作用时间短一点曲线平滑一点
  roll.iLimit = 14;
  roll.lastoutput = 0.00;
}
*//**/
