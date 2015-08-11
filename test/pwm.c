#include <wiringPi.h>
#include <softPwm.h>
#include <stdio.h>
#include <stdlib.h>
//http://raspberrypi.stackexchange.com/questions/23361/wiringpi-softpwm-calculation-to-determine-duty-cycle
int main ()
{
    int level;
    int PinNumber1 = 7, PinNumber2 = 0;
    if (-1 == wiringPiSetup()){
    printf("Setup WiringPi failed!");
    return 1;}
    softPwmCreate(PinNumber1,0,20);//20*100=2000microseconds=500hz
    softPwmCreate(PinNumber2,0,20);
    softPwmWrite(PinNumber1,19);
    softPwmWrite(PinNumber2,19);
    printf("96now\n");
    getchar();
    softPwmWrite(PinNumber1,11);
    softPwmWrite(PinNumber2,11);
    printf("54now\n");
    delay(1000);
    softPwmWrite(PinNumber1,12.6);
    softPwmWrite(PinNumber2,12.6);
    printf("start ,press to stop");
    getchar();

}