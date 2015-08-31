#include "pca9685.h"
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
#include <softPwm.h>
#include <math.h>
#include <wiringPiI2C.h>

#define PIN_BASE 300
#define HERTZ 500

int main()
{
    int fd_pca9685 = pca9685Setup(PIN_BASE, 0x40, HERTZ);
	if (fd_pca9685 < 0)
	{
		printf("Error in setup\n");
		return fd_pca9685;
	}

	// Reset all output
	pca9685PWMReset(fd_pca9685);
    pwmWrite(PIN_BASE + 0,2099);
    pwmWrite(PIN_BASE + 1,2099);
    pwmWrite(PIN_BASE + 2,2099);
    pwmWrite(PIN_BASE + 3,2099);
    while(1)
    {
        
    }
    return 0;
}