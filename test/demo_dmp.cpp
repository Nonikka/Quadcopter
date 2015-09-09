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
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "pca9685.h"

#define PIN_BASE 300
#define HERTZ 500
#define PinNumber1 0  
#define PinNumber2 1  
#define PinNumber3 2  
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

float Acceleration[3],AngleSpeed[3],Angle[3],Roll,Pitch,Yaw,DutyCycle[4],Accelerator,Pid_Pitch=0,Pid_Roll=0,Pid_Yaw=0,Default_Acc = 0.03,pid_in,pid_error,Roll_PError,Pitch_PError,Yaw_PError,pregyro,Inital_Yaw[7],Inital_Roll[7],Inital_Pitch[7];
int All_Count=0,START_FLAG=0,Inital=0,PID_ENABLE=0;

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
    Roll_Suit.kp = 0.0068;
    Roll_Suit.ki = 0.000;
    Roll_Suit.kd = 0.0018;
    Roll_Suit.pregyro =0;
    //Roll_Suit.desired = 1;
    Roll_Suit.integ=0;
    Roll_Suit.iLimit =8;
    Roll_Suit.deriv=0;
    Roll_Suit.output = 0.00;
    Roll_Suit.lastoutput=0;
    
    Pitch_Suit.kp = 0.0068;
    Pitch_Suit.ki = 0.000;
    Pitch_Suit.kd = 0.0018;
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
    //pidsuite.preerror = pidsuite.error;//更新前一次偏差
    
    pidsuite.error = desired - measured + Inital_Error ;//偏差：期望-测量值
    pidsuite.error = pidsuite.error * 0.88 + pidsuite.preerror * 0.12;
    //pid_error = pidsuite.error;//debug用
    
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
    if (fabs(pidsuite.deriv) < 20 )
    {
        if (fabs(pidsuite.deriv) < 10 )
        {
            pidsuite.deriv = pidsuite.deriv * 0.8;
        }
        else
        {
            pidsuite.deriv = pidsuite.deriv * 0.9;
        }
    }
    pidsuite.output = (pidsuite.kp * pidsuite.error) + (pidsuite.ki * pidsuite.integ) + (pidsuite.kd * pidsuite.deriv);
    pid_in = pidsuite.output;
    
    pregyro = pidsuite.pregyro;
    if (pidsuite.output >  0.12)
    {
        pidsuite.output = 0.12;
    }
    if (pidsuite.output <  -0.12)
    {
        pidsuite.output = -0.12;
    }
    //output = output * 0.9 + lastoutput * 0.1;
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

void* gyro_acc(void*)
{
    //int i = 0;
    // initialize device
    printf("Initializing I2C devices...\n");
    mpu.initialize();
    
    // verify connection
    printf("Testing device connections...\n");
    printf(mpu.testConnection() ? "MPU6050 connection successful\n" : "MPU6050 connection failed\n");
    mpu.setI2CMasterModeEnabled(false);
    mpu.setI2CBypassEnabled(true);
    // load and configure the DMP
    printf("Initializing DMP...\n");
    devStatus = mpu.dmpInitialize();
    
    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        printf("Enabling DMP...\n");
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        //Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        //attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        printf("DMP ready!\n");
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        printf("DMP Initialization failed (code %d)\n", devStatus);
        return 0;
    }
    /*****************************************************/
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
        // get current FIFO count
        fifoCount = mpu.getFIFOCount();

        if (fifoCount == 1024) 
        {
            // reset so we can continue cleanly
            mpu.resetFIFO();
            printf("FIFO overflow!\n");

            // otherwise, check for DMP data ready interrupt (this should happen frequently)
        } 
        else if (fifoCount >= 42) 
        {
            // read a packet from FIFO
            mpu.getFIFOBytes(fifoBuffer, packetSize);
            
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            //printf("ypr  %7.2f %7.2f %7.2f  ", ypr[0] * 180/M_PI, ypr[1] * 180/M_PI, ypr[2] * 180/M_PI);
            Angle[2] = ypr[0] * 180/M_PI;
            Angle[1] = ypr[1] * 180/M_PI;//此为Pitch
            Angle[0] = ypr[2] * 180/M_PI;//此为Roll
            
            // display initial world-frame acceleration, adjusted to remove gravity
            // and rotated based on known orientation from quaternion
            /*
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetAccel(&aa, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
            //printf("aworld %6d %6d %6d    ", aaWorld.x, aaWorld.y, aaWorld.z);
            //AngleSpeed[0] =  aaWorld.x;
            //AngleSpeed[1] =  aaWorld.y;
            //AngleSpeed[2] =  aaWorld.z;
            */
            /****************************读取完毕*********************************/
            /*
            if (Inital <= 600)
            {
                Inital ++;
                if (Inital % 98 == 1)
                {
                Inital_Roll[i] = Angle[0];
                Inital_Pitch[i] = Angle[1];
                Inital_Yaw[i] = Angle[2];
                printf("\nRoll:%.2f Pitch:%.2f Yaw:%.2f",Inital_Roll[i],Inital_Pitch[i],Inital_Yaw[i]);
                i++;
                printf("\t%d\n",Inital);
                fflush(stdout);
                if (i == 6)
                {
                    Inital_Yaw[6] = (Inital_Yaw[3] + Inital_Yaw[4] + Inital_Yaw[5]) / 3;
                    Inital_Roll[6] =(Inital_Roll[3] + Inital_Roll[4] + Inital_Roll[5]) / 3;
                    Inital_Pitch[6] = (Inital_Pitch[3] + Inital_Pitch[4] + Inital_Pitch[5]) / 3;
                    printf("\n\nInital: Roll:%.2f Pitch:%.2f Yaw:%.2f\n\n\n",Inital_Roll[i],Inital_Pitch[i],Inital_Yaw[i]);
                }
                }
            }*/
            
            Pid_Roll = Pid_Calc(Roll_Suit,Angle[0],1 ,0.38);
            Pid_Pitch = Pid_Calc(Pitch_Suit,Angle[1],-0.7 ,-0.13);
            Pid_Yaw = Pid_Calc(Yaw_Suit,Angle[2],0,-2);
            All_Count = All_Count + 1;
            DutyCycle[0] = Default_Acc  - Pid_Roll - Pid_Pitch; //- Pid_Yaw;
            DutyCycle[1] = Default_Acc  - Pid_Roll + Pid_Pitch; //+ Pid_Yaw;
            //DutyCycle[0] = Default_Acc;
            //DutyCycle[1] = Default_Acc;
            DutyCycle[2] = Default_Acc  + Pid_Roll - Pid_Pitch; //+ Pid_Yaw;
            DutyCycle[3] = Default_Acc  + Pid_Roll + Pid_Pitch; //- Pid_Yaw;
            //DutyCycle[2] = Default_Acc;
            //DutyCycle[3] = Default_Acc;
        
            PWMOut(PinNumber1,DutyCycle[0]);
            PWMOut(PinNumber2,DutyCycle[1]);
            PWMOut(PinNumber3,DutyCycle[2]);
            PWMOut(PinNumber4,DutyCycle[3]);
            
        }
    }
}

void* KeyBoard(void*)
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
            PID_ENABLE = 0;
        }
        else if (keychar == 'r')
        {
            Default_Acc = 0.05;
            delay(200);
            Default_Acc = 0.03;
            delay(200);
            PWMOut(PinNumber1,0.03);
            PWMOut(PinNumber2,0.03);
            PWMOut(PinNumber3,0.03);
            PWMOut(PinNumber4,0.03);
            return 0;
        }
        else if (keychar == 'w')
        {
            Default_Acc = 0.5;
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
    TimeStart = millis();
    
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
    getchar();
    
    START_FLAG = 1;
    PWMOut(PinNumber1,0.06);
    PWMOut(PinNumber2,0.06);
    PWMOut(PinNumber3,0.06);
    PWMOut(PinNumber4,0.06);
    delay(500);
    
    /*********************/
    ret = pthread_create(&transport,NULL,KeyBoard,NULL);
    if(ret!=0)
    {
        printf ("Create KeyBoard thread error!\n");
        exit (1);
    }
    while(1)  
    {  
        TimeNow = millis();
        system("clear");
        printf("Pid_Roll:%.4f  pid_error:%.3f  pid_pError:%.3f pregyro %.3f All_Count: %d",Pid_Roll,pid_error,Roll_PError,pregyro,All_Count);
        printf("A:%.2f %.2f %.2f\n",Angle[0],Angle[1],Angle[2]); 
        printf("Default_Acc:%.2f gyro： roll :%.2f\n",Default_Acc,AngleSpeed[0]); 
        fflush(stdout);
    }
}