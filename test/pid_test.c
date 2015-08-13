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
#define PinNumber1 7
#define PinNumber2 0
#define IMU_UPDATE_DT 10
float Acceleration[3],AngleSpeed[3],Angle[3],Roll,Pitch,Yaw,DutyCycle[3],TARGET,Pid_Pitch;
int all_count;
float PidUpdate(/*pidsuite* pid,*/ const float measured,float expect,float gyro);

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
            //printf("count: %d time: %d\n",all_count,TimeNow - TimeStart);
            //printf("DutyCycle[0]: %.2f\n",DutyCycle[0]);
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
    int ret;
    if (-1 == wiringPiSetup())
    {
        printf("Setup WiringPi failed!");
        return 1;
    }
    softPwmCreate(PinNumber1,0,20);//20*100=2000microseconds=500hz
    softPwmCreate(PinNumber2,0,20);
    /**
    softPwmWrite(PinNumber,19);
    printf("96now\n");
    getchar();
    softPwmWrite(PinNumber,11);
    printf("54now\n");
    fflush(stdout);
    delay(800);
    **/
    softPwmWrite(PinNumber1,19);
    softPwmWrite(PinNumber2,19);
    printf("input to start");
    getchar();
    softPwmWrite(PinNumber1,11.2);
    softPwmWrite(PinNumber2,11.2);
    delay(1000);
    softPwmWrite(PinNumber1,13.5);
    softPwmWrite(PinNumber2,13.5);
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
        Pid_Pitch = PidUpdate(Angle[1],0,AngleSpeed[1]);
        if (Pid_Pitch >  3.0)
        {
            Pid_Pitch = 3.0;
        }
        if (Pid_Pitch <  -3.0)
        {
            Pid_Pitch = -3.0;
        }
        system("clear");
        //delay(100);
        printf("Pid_Pitch: %.2f all_count: %d\n",Pid_Pitch,all_count);
        printf("A:%.2f %.2f %.2f\n",Angle[0],Angle[1],Angle[2]); 
        fflush(stdout);
        DutyCycle[1] = 12.4 + Pid_Pitch;
        
        DutyCycle[0] = 12.4 - Pid_Pitch;
        softPwmWrite(PinNumber1,DutyCycle[1]);
        softPwmWrite(PinNumber2,DutyCycle[0]);
        
        //printf("DutyCycle: %d\n",DutyCycle[0]);
        //fflush(stdout);
    }  
}

/*****
void PID_CAL(void)//PID计算函数
{
static float thr=0,rool=0,pitch=0,yaw=0;//控制量
static float rool_i=0,pitch_i=0; 积分

int16_t Motor1,Motor2,Motor3,Motor4;//四个电机输出

IMU_TEST();//计算姿态
GET_EXPRAD();//获得控制量

rool = PID_RP.P * DIF_ANGLE.X;//roll方向的P计算

if(Q_ANGLE.Rool>-0.1 && Q_ANGLE.Rool<0.1)//判断I是否需要清零
{ 
    rool_i = 0;
}
rool_i -= PID_RP.I * Q_ANGLE.Rool;//I计算
PID_RP.IMAX = DIF_ANGLE.X * 10;//积分限幅
if(PID_RP.IMAX<0) 
{
    PID_RP.IMAX = (-PID_RP.IMAX) + 100;
}
else
{
    PID_RP.IMAX += 100;
}
if(rool_i>PID_RP.IMAX) 
{
    rool_i = PID_RP.IMAX;
}
if(rool_i<-PID_RP.IMAX) 
{
    rool_i = -PID_RP.IMAX;
}
rool += rool_i; 积分

rool -= PID_RP.D * GYRO_F.X;//D计算

pitch = PID_RP.P * DIF_ANGLE.Y;//同上

if(Q_ANGLE.Pitch>-0.1 && Q_ANGLE.Pitch<0.1)
{
    pitch_i = 0;
}
pitch_i -= PID_RP.I * Q_ANGLE.Pitch;
if(PID_RP.IMAX<0) 
{
    PID_RP.IMAX = (-PID_RP.IMAX) + 100;
}
else
{
    PID_RP.IMAX += 100;
}
if(PID_RP.IMAX<0) 
{
    PID_RP.IMAX = -PID_RP.IMAX;
}
if(pitch_i>PID_RP.IMAX) 
{
    pitch_i = PID_RP.IMAX;
}
    
if(pitch_i<-PID_RP.IMAX) 
{
    pitch_i = -PID_RP.IMAX;
}
pitch += pitch_i;

pitch -= PID_RP.D * GYRO_F.Y;

GYRO_I[0].Z += EXP_ANGLE.Z/3000;//yaw方向就简单的用了陀螺的积分 PD
yaw = -10 * GYRO_I[0].Z;

yaw -= 3 * GYRO_F.Z;

thr = RC_DATA.THROTTLE+400;
//将控制量输出给电机
Motor1=(int16_t)(thr + rool - pitch + yaw);
Motor2=(int16_t)(thr + rool + pitch - yaw);
Motor3=(int16_t)(thr - rool + pitch + yaw);
Motor4=(int16_t)(thr - rool - pitch - yaw);
if(FLY_ENABLE && (RC_DATA.THROTTLE>-400))//和解锁有关,未解锁或油门太低电机禁止转动
MOTO_PWMRFLASH(Motor1,Motor2,Motor3,Motor4);
else
MOTO_PWMRFLASH(0,0,0,0);
}
****************/

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
}pid/*,_Pitch,_Yaw*/;  

float PidUpdate(/*pidsuite* pid,*/ const float measured,float expect,float gyro)
{
  float output;
  static float lastoutput=0;
  float Piddeadband=0.2;
  pid.kp = 0.03;// 0.15以上就已经十分危险！！！！
  pid.ki = 0.005;
  pid.kd = 0.01;
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
        
          output = (pid.kp * pid.error) +
                   (pid.ki * pid.integ) +
                   (pid.kd * pid.deriv);
  }
  else
  {
    output=lastoutput;
  }

  pid.prevError = pid.error;//更新前一次偏差
  lastoutput=output;

  return output;
}