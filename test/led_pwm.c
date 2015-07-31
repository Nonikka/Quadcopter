#include <wiringPi.h>
#include <softPwm.h>
#include <stdio.h>
#include <stdlib.h>

int main ()
{
    int level;
    int PinNumber = 7;
    if (-1 == wiringPiSetup()){
    printf("Setup WiringPi failed!");
    return 1;}
    
    softPwmCreate(PinNumber,0,100);
    
    while(1){
        printf("WiringPi up");
        for(level = 10;level <= 90;level = level + 1){
            softPwmWrite(PinNumber,level);
            delay(40);
        }
        printf("WiringPi down");
        delay(200);
        for(level = 100;level >= 10;level = level - 1){
            softPwmWrite(PinNumber,level);
            delay(40);
        }
        delay(500);
    }
}