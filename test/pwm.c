#include <wiringPi.h>
#include <softPwm.h>
#include <stdio.h>
#include <stdlib.h>
//http://raspberrypi.stackexchange.com/questions/23361/wiringpi-softpwm-calculation-to-determine-duty-cycle
int main ()
{
    int level;
    int PinNumber = 7;
    if (-1 == wiringPiSetup()){
    printf("Setup WiringPi failed!");
    return 1;}
    softPwmCreate(PinNumber,0,20);//20*100=2000microseconds=500hz
    softPwmWrite(PinNumber,19);
    printf("96now\n");
    getchar();
    softPwmWrite(PinNumber,11);
    printf("54now\n");
    delay(1000);
    softPwmWrite(PinNumber,12.6);
    printf("start ,press to stop");
    getchar();

}