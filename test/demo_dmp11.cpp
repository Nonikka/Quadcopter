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
#include <stdint.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "pca9685.h"

#define DEFAULT_PORT 8099 
#define MAXLINE 4096  
#define PIN_BASE 300
#define HERTZ 500
#define PinNumber1 0  
#define PinNumber2 1  
#define PinNumber3 8  //放里面省的被浆卷到
#define PinNumber4 3  
#define IMU_UPDATE_DT 0.01
#define MAX_ACC 0.59
#define OUTPUT_READABLE_YAWPITCHROLL
#define OUTPUT_READABLE_WORLDACCEL
// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
int count_time = 0;
// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };

float Default_Acc = 0.03,Pid_Pitch=0,Pid_Roll=0,Pid_Yaw=0,Accelerator,Roll,Pitch,Yaw,pid_in,pid_error,Roll_PError,Pitch_PError,Yaw_PError,pregyro,Acceleration[3],AngleSpeed[3],Angle[3],DutyCycle[4],Inital_Yaw[7],Inital_Roll[7],Inital_Pitch[7],Filter_Roll[10],Filter_Pitch[10];
int All_Count=0,START_FLAG=0,Inital=0,PID_ENABLE=0,_axis[6],filter_count,STOP1 = 0,STOP2 = 0,LEFT_ROTATE,RIGHT_ROTATE;//遥控器传来的轴
unsigned int TimeNow,TimeStart,TimeLastGet;
void PWMOut(int pin, float pwm);

MPU6050 mpu;
struct PID
{  
  float kp;           //< proportional gain调整比例参数  
  float ki;           //< integral gain调整积分参数  
  float kd;           //< derivative gain调整微分参数  
  float pregyro;
  //float desired;     //< set point   期望值  
  float integ;        //< integral积分参数  
  float iLimit;      //< integral limit积分限制范围  
  float deriv;        //< derivative微分参数  
  float preerror;    //< previous error 上一次误差  
  float output;
  float error;        //< error 误差  
  float lastoutput;
} ;

PID Roll_Suit;
PID Pitch_Suit;
PID Yaw_Suit;

void Pid_Inital()
{
    Roll_Suit.kp = 0.00105;//0.0068有点大 0.064有点大 0.56太大 0.0052太大 //0.0046 0.00285 p太大
    Roll_Suit.ki = 0.0000;
    Roll_Suit.kd = 0.00182;//跟着0.0018改 0.175太小 0.0019太小 0.0023太小         //0.00105 0.00165 可以试试 0.00168可以
    Roll_Suit.pregyro =0;
    //Roll_Suit.desired = 1;
    Roll_Suit.integ=0;
    Roll_Suit.iLimit =8;
    Roll_Suit.deriv=0;
    Roll_Suit.output = 0.00;
    Roll_Suit.lastoutput=0;
    
    Pitch_Suit.kp = 0.00105;
    Pitch_Suit.ki = 0.0000;
    Pitch_Suit.kd = 0.00182;
    Pitch_Suit.pregyro =0;
    //Pitch_Suit.desired = -0.7;
    Pitch_Suit.integ=0;
    Pitch_Suit.iLimit =8;
    Pitch_Suit.deriv=0;
    Pitch_Suit.lastoutput=0;
    Pitch_Suit.output = 0.00;
    
    Yaw_Suit.kp = 0.003;
    Yaw_Suit.kd = 0.002;
}

float Pid_Calc(PID &pidsuite,float measured,float desired,float Inital_Error)
{
    
    pidsuite.error = desired - measured + Inital_Error ;//偏差：期望-测量值
    pidsuite.error = pidsuite.error; 
    
    pidsuite.integ += pidsuite.error * IMU_UPDATE_DT;//偏差积分，IMU_UPDATE_DT也就是每调整漏斗大小的步辐
    
    if (pidsuite.integ >= pidsuite.iLimit)//作积分限制
    {
      pidsuite.integ = pidsuite.iLimit;
    }
    else if (pidsuite.integ < -pidsuite.iLimit)
    {
      pidsuite.integ = -pidsuite.iLimit;
    }
    
    pidsuite.deriv = (pidsuite.error - pidsuite.preerror) / 0.01;//微分     应该可用陀螺仪角速度代替
    pidsuite.preerror = pidsuite.error;//debug用 更新前一次偏差
    
    AngleSpeed[0] = pidsuite.deriv;
    
    pidsuite.output = (pidsuite.kp * pidsuite.error) + (pidsuite.ki * pidsuite.integ) + (pidsuite.kd * pidsuite.deriv);
    pid_in = pidsuite.output;
    
    pregyro = pidsuite.pregyro;
    if (pidsuite.output >  0.15)
    {
        pidsuite.output = 0.15;
    }
    if (pidsuite.output <  -0.15)
    {
        pidsuite.output = -0.15;
    }
    if (fabs(pidsuite.error) < 0.3 )
    {
        pidsuite.output = pidsuite.lastoutput * 0.5;
    }
    pidsuite.lastoutput = pidsuite.output;
    if(PID_ENABLE = 0)
    {
        return 0;
    }
    return pidsuite.output;
}

float average_filter(float filter_input ,float (&Filter)[10])
{
    float filter_output;
    Filter[9] = filter_input;
    
    filter_output = 0.65 * Filter[8] + 0.14 * Filter[6] + 0.11 * Filter[4] + 0.06 * Filter[2] + 0.04 * Filter[0];
    
    for(filter_count=9;filter_count>0;filter_count--)
    {
        Filter[filter_count-1]=Filter[filter_count];
    }
    
    return filter_output;
}

void* gyro_acc(void*)
{
    printf("Initializing I2C devices...\n");
    mpu.initialize();
    
    printf("Testing device connections...\n");
    printf(mpu.testConnection() ? "MPU6050 connection successful\n" : "MPU6050 connection failed\n");
    mpu.setI2CMasterModeEnabled(false);
    mpu.setI2CBypassEnabled(true);
    printf("Initializing DMP...\n");
    devStatus = mpu.dmpInitialize();
    
    if (devStatus == 0) 
    {
        printf("Enabling DMP...\n");
        mpu.setDMPEnabled(true);

        
        mpuIntStatus = mpu.getIntStatus();
        printf("DMP ready!\n");
        dmpReady = true;
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else 
    {
        printf("DMP Initialization failed (code %d)\n", devStatus);
        return 0;
    }
    while(1)
    {
        if (START_FLAG == 0)
        {
            delay(200);
        }
        if (START_FLAG == 1)
        {
            break;
        }
    }
    delay(50);
    for(;;)
    {
        if (!dmpReady) return 0;
        fifoCount = mpu.getFIFOCount();

        if (fifoCount == 1024) 
        {
            mpu.resetFIFO();
            printf("FIFO overflow!\n");
        } 
        else if (fifoCount >= 42) 
        {
            mpu.getFIFOBytes(fifoBuffer, packetSize);
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            Angle[2] = ypr[0] * 180/M_PI;
            Angle[1] = average_filter(ypr[1] * 180/M_PI,Filter_Pitch);//此为Pitch
            Angle[0] = average_filter(ypr[2] * 180/M_PI,Filter_Roll);//此为Roll
            
            Pid_Roll = Pid_Calc(Roll_Suit,Angle[0],0.68 - 11 * _axis[1] * 0.01,0.38);
            Pid_Pitch = Pid_Calc(Pitch_Suit,Angle[1],-0.55 - 11 * _axis[2] * 0.01,-0.13);
            Pid_Yaw = Pid_Calc(Yaw_Suit,Angle[2],0,Inital_Yaw[1]);
            if (Pid_Yaw > 0.03)
            {
                Pid_Yaw = 0.03;
            }
            if (Pid_Yaw < -0.03)
            {
                Pid_Yaw = -0.03;
            }
            All_Count = All_Count + 1;
            Default_Acc = Default_Acc + _axis[0] * 0.0001 * 0.052;//0.04太小
            if (STOP1 != 48 && STOP2 != 48)
            {
                Default_Acc = 0.04;
            }
            TimeNow = millis();
            if (abs(TimeNow - TimeLastGet) > 800)
            {
                if(Default_Acc > 0.4)
                {
                    Default_Acc = 0.46;
                    _axis[1] = 0;
                    _axis[2] = 0;//前后左右置零
                }
                else
                {
                    Default_Acc = 0.03;
                }
            }
            
            if (Default_Acc >= MAX_ACC)
            {
                Default_Acc = MAX_ACC;
            }
            if (Default_Acc <= 0.03)
            {
                Default_Acc = 0.03;
            }
            if (LEFT_ROTATE != 48)
            {
                Pid_Yaw = Pid_Yaw - 0.018;
                Inital_Yaw[1] = Angle[2];
            }
            if (RIGHT_ROTATE != 48)
            {
                Pid_Yaw = Pid_Yaw + 0.018;
                Inital_Yaw[1] = Angle[2];
            }
            DutyCycle[0] = Default_Acc  - Pid_Roll - Pid_Pitch - Pid_Yaw;
            DutyCycle[1] = Default_Acc  - Pid_Roll + Pid_Pitch + Pid_Yaw;
            DutyCycle[2] = Default_Acc  + Pid_Roll - Pid_Pitch + Pid_Yaw;
            DutyCycle[3] = Default_Acc  + Pid_Roll + Pid_Pitch - Pid_Yaw;
        
            PWMOut(PinNumber1,DutyCycle[0]);
            PWMOut(PinNumber2,DutyCycle[1]);
            PWMOut(PinNumber3,DutyCycle[2]);
            PWMOut(PinNumber4,DutyCycle[3]);
            
        }
    }
}

//1油门 2前后 3左右 4旋转 5预留 6预留 每个三位
void* serial_DL22(void*)
{
    int fd,counter=0;
    //int Num_Avail;
    unsigned char Re_buf[19];
    unsigned char ucStr[18];
    char axis1[5],axis2[5],axis3[5],axis4[5];
    if ((fd = serialOpen ("/dev/ttyAMA0", 115200)) < 0)
    {
    fprintf (stderr, "Unable to open serial device: %s\n", strerror (errno)) ;
    return 0 ;
    }

    for (;;)
    {
        Re_buf[counter]=serialGetchar(fd);
        if(Re_buf[0]!=0x55) // 0x55 = U
        {
            memset(Re_buf, 0, 18*sizeof(char));
            counter = 0;
            
        }
        else
        {
            counter++;
            if(counter==18)             //接收到11个数据
            {    
            counter=0;               //重新赋值，准备下一帧数据的接收        
            TimeLastGet = millis();
            ucStr[0]=Re_buf[1];
            ucStr[1]=Re_buf[2];
            ucStr[2]=Re_buf[3];
            ucStr[3]=Re_buf[4];
            ucStr[4]=Re_buf[5];
            ucStr[5]=Re_buf[6];
            ucStr[6]=Re_buf[7];
            ucStr[7]=Re_buf[8];
            ucStr[8]=Re_buf[9];
            ucStr[9]=Re_buf[10];
            ucStr[10]=Re_buf[11];
            ucStr[11]=Re_buf[12];
            ucStr[12]=Re_buf[13];
            ucStr[13]=Re_buf[14];
            ucStr[14]=Re_buf[15];
            ucStr[15]=Re_buf[16];
            ucStr[16]=Re_buf[17];
            ucStr[17]=Re_buf[18]; //注意在接受中显示只有前17位，最后一位估计是作为了\0
            
            axis1[0] = Re_buf[1];
            axis1[1] = Re_buf[2];
            axis1[2] = Re_buf[3];
            axis1[3] = Re_buf[4];
            axis1[4] = '\0';
            _axis[0] =  atoi(axis1);//油门控制
            
            axis2[0] = Re_buf[5];
            axis2[1] = Re_buf[6];
            axis2[2] = Re_buf[7];
            axis2[3] = Re_buf[8];
            axis2[4] = '\0';
            _axis[1] =  atoi(axis2);//前后控制 roll
            
            axis3[0] = Re_buf[9];
            axis3[1] = Re_buf[10];
            axis3[2] = Re_buf[11];
            axis3[3] = Re_buf[12];
            axis3[4] = '\0';//不加会导致转换错误
            _axis[2] =  atoi(axis3);//左右控制 pitch
            STOP1 = Re_buf[13];
            STOP2 = Re_buf[14];
            LEFT_ROTATE = Re_buf[15];
            RIGHT_ROTATE = Re_buf[16];
            
            printf("recv from client: %s %d %d %d %d %d \n\n",ucStr,_axis[0],_axis[1],_axis[2],LEFT_ROTATE,RIGHT_ROTATE);
            memset(Re_buf, 0, 18*sizeof(char));
            }
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
    pthread_t mpu6050,joystick;//transport
    int ret;
    Pid_Inital();
    PID_ENABLE = 1;
    if (-1 == wiringPiSetup())
    {
        printf("Setup WiringPi failed!");
        return 1;
    }
    
    delay(100);
    
    ret = pthread_create(&mpu6050,NULL,gyro_acc,NULL);
    if(ret!=0)
    {
        printf ("Create mpu6050 thread error!\n");
        exit (1);
    }
    delay(50);
    mpu.setI2CMasterModeEnabled(false);//不知道这句话要放哪，此处有作用
    mpu.setI2CBypassEnabled(true);
    int fd_pca9685 = pca9685Setup(PIN_BASE, 0x40, HERTZ);
	if (fd_pca9685 < 0)
	{
		printf("Error in setup pca9685\n");
		return 0;
	}
    pca9685PWMReset(fd_pca9685);
    
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
    
    printf("Way 2:PWM in 0 \n");
    PWMOut(PinNumber1,0);
    PWMOut(PinNumber2,0);
    PWMOut(PinNumber3,0);
    PWMOut(PinNumber4,0);
    printf("input to start!\n");
    fflush(stdout);
    delay(1000);
    //getchar();
    
    START_FLAG = 1;
    PWMOut(PinNumber1,0.06);
    PWMOut(PinNumber2,0.06);
    PWMOut(PinNumber3,0.06);
    PWMOut(PinNumber4,0.06);
    delay(500);
    TimeStart = millis();
    
    ret = pthread_create(&joystick,NULL,serial_DL22,NULL);//启动serial手柄线程
    if(ret!=0)
    {
        printf ("Create joystick thread error!\n");
        exit (1);
    }
    delay(4200);
    Inital_Yaw[1] = Angle[2];
    while(1)  
    {  
        system("clear");
        printf("Pid_Roll:%.4f Pid_Yaw:%.4f pid_error:%.3f  pregyro %.3f All_Count: %d",Pid_Roll,Pid_Yaw,pid_error,pregyro,All_Count);
        printf("A:%.2f %.2f %.2f\n",Angle[0],Angle[1],Angle[2]); 
        printf("Default_Acc:%.3f gyro： roll :%.2f\n",Default_Acc,AngleSpeed[0]); 
        fflush(stdout);
    }
}