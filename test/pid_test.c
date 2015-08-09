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
            //serialFlush(fd);
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
    /**
    softPwmWrite(PinNumber,19);
    printf("96now\n");
    getchar();
    softPwmWrite(PinNumber,11);
    printf("54now\n");
    fflush(stdout);
    delay(800);
    **/
    printf("input to start");
    getchar();
    softPwmWrite(PinNumber,13.5);
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
        DutyCycle[1] = PidUpdate(_Pitch,Pitch,0,AngleSpeed[1]);
        softPwmWrite(PinNumber,DutyCycle[0]);
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
        double kp = 0.66;  
        double ki = 0;
        double kd = 0;
        double desired;
        double error;
        double integ;//积分
        double iLimit;
        double deriv;//微分
        double prevError;
        double outP;
        double outI;
        double outD;
}_Roll,_Pitch,_Yaw;  

float PidUpdate(pidsuite* pid, const float measured,float expect,float gyro)
{
  float output;
  static float lastoutput=0;

  pid->desired=expect;//获取期望角度

  pid->error = pid->desired - measured;//偏差：期望-测量值
  
  pid->integ += pid->error * IMU_UPDATE_DT;//偏差积分
 
  if (pid->integ > pid->iLimit)//作积分限制
  {
    pid->integ = pid->iLimit;
  }
  else if (pid->integ < -pid->iLimit)
  {
    pid->integ = -pid->iLimit;
  }

 // pid->deriv = (pid->error - pid->prevError) / IMU_UPDATE_DT;//微分     应该可用陀螺仪角速度代替
  pid->deriv = -gyro;
  if(fabs(pid->error)>Piddeadband)//pid死区
  {
          pid->outP = pid->kp * pid->error;//方便独立观察
          pid->outI = pid->ki * pid->integ;
          pid->outD = pid->kd * pid->deriv;
        
          output = (pid->kp * pid->error) +
                   (pid->ki * pid->integ) +
                   (pid->kd * pid->deriv);
  }
  else
  {
    output=lastoutput;
  }

  pid->prevError = pid->error;//更新前一次偏差
  lastoutput=output;

  return output;
}