#include <wiringPi.h>
#include <softPwm.h>
#include <stdio.h>
#include <stdlib.h>
#include <wiringPiSPI.h> 
//http://wiringpi.com/reference/spi-library/

int main (){
    int SPIChannel = 1;
    int SPIClock = 115200
    if (wiringPiSPISetup(SPIChannel,SPIClock) =  -1){
        printf("Setup WiringPiSPI failed!");
        return 1;}
    
}