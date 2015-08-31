/*
这个版本无failed to write reg，去除了pca9865 未接线
*/
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
#define MAX_ACC 0.55
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

float Acceleration[3],AngleSpeed[3],Angle[3],Roll,Pitch,Yaw,DutyCycle[4],Accelerator,Pid_Pitch,Pid_Roll=0,Default_Acc = 0.10;
float error=0;
int All_Count=0;
float output=0;
float PidUpdate(/*pidsuite* pid,*/ const float measured,float expect,float gyro);
float PidUpdate_roll(/*pidsuite* pid,*/ const float measured,float expect,float gyro);
void PWMOut(int pin, float pwm);

MPU6050 mpu;

void* gyro_acc(void*)
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
    //float Piddeadband=0.3;
    // initialize device
    printf("Initializing I2C devices...\n");
    mpu.initialize();

    // verify connection
    printf("Testing device connections...\n");
    printf(mpu.testConnection() ? "MPU6050 connection successful\n" : "MPU6050 connection failed\n");

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
        Angle[1] = ypr[1] * 180/M_PI;
        Angle[0] = ypr[2] * 180/M_PI;
        // display initial world-frame acceleration, adjusted to remove gravity
        // and rotated based on known orientation from quaternion
        
        mpu.dmpGetAccel(&aa, fifoBuffer);
        
        mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
        //printf("aworld %6d %6d %6d    ", aaWorld.x, aaWorld.y, aaWorld.z);
        AngleSpeed[0] =  aaWorld.x;
        AngleSpeed[1] =  aaWorld.y;
        AngleSpeed[2] =  aaWorld.z;
        
        /****************************读取完毕*********************************/
        error = desired - Angle[0];//偏差：期望-测量值
        All_Count = All_Count + 1;
        error = error * 0.85 + prevError * 0.15;
        if (fabs(prevError - error ) > 12)
        {
            error = prevError;
        }
        
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
        
        output = (kp * error) + (ki * integ) + (kd * deriv);
        Pid_Roll = output;
        
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
        /*
        PWMOut(PinNumber1,DutyCycle[0]);
        PWMOut(PinNumber2,DutyCycle[1]);
        PWMOut(PinNumber3,DutyCycle[2]);
        PWMOut(PinNumber4,DutyCycle[3]);
        */
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
            /*
            PWMOut(PinNumber1,0.03);
            PWMOut(PinNumber2,0.03);
            PWMOut(PinNumber3,0.03);
            PWMOut(PinNumber4,0.03);
            */
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
    /*
    int fd_pca9685 = pca9685Setup(PIN_BASE, 0x40, HERTZ);
	if (fd_pca9685 < 0)
	{
		printf("Error in setup\n");
		return fd_pca9685;
	}
    pca9685PWMReset(fd_pca9685);
    */
    /**********************************************************************/
    
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
    /***
    printf("Way 2:PWM in 0 \n");
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
    *********************/
    ret = pthread_create(&mpu6050,NULL,gyro_acc,NULL);
    if(ret!=0)
    {
        printf ("Create mpu6050 thread error!\n");
        exit (1);
    }
    TimeStart = millis();
    ret = pthread_create(&transport,NULL,KeyBoard,NULL);
    if(ret!=0)
    {
        printf ("Create KeyBoard thread error!\n");
        exit (1);
    }
    
    while(1)  
    {  
        //Pid_Pitch = PidUpdate(Angle[1],-1.6,AngleSpeed[1]);
        //Pid_Roll = PidUpdate_roll(Angle[0],0,AngleSpeed[0]);
        TimeNow = millis();
        system("clear");
        //delay(100);
        printf("Pid_Roll:%.3f %.3f  error %.3f  All_Count: %d time:%d\n",output,Pid_Roll,error,All_Count,TimeNow - TimeStart);
        printf("A:%.2f %.2f %.2f\n",Angle[0],Angle[1],Angle[2]); 
        printf("Default_Acc:%.2f gyro：Pitch：%.2f roll :%.2f\n",Default_Acc,AngleSpeed[1],AngleSpeed[0]); 
        fflush(stdout);
        //DutyCycle[3] = Default_Acc + Pid_Pitch - Pid_Roll;//+yaw
        //DutyCycle[2] = Default_Acc - Pid_Pitch + Pid_Roll;//+yaw
        //DutyCycle[1] = Default_Acc + Pid_Pitch + Pid_Roll;//-yaw
        //DutyCycle[0] = Default_Acc - Pid_Pitch - Pid_Roll;//-yaw
    }
}