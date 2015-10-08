//-lwiringPi -Wall -lpthread -I/usr/local/include -L/usr/local/lib

#ifndef pwmlibH
#define pwmlibH

#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdint.h>
#include <errno.h>
#include <math.h>

// taken from Adafruit_PWM_Servo_Driver.py
#define SUBADR1 0x02
#define SUBADR2 0x03
#define SUBADR3 0x04
#define MODE1 0x00
#define PRESCALE 0xFE
#define LED0_ON_L 0x06
#define LED0_ON_H 0x07
#define LED0_OFF_L 0x08
#define LED0_OFF_H 0x09
#define ALLLED_ON_L 0xFA
#define ALLLED_ON_H 0xFB
#define ALLLED_OFF_L 0xFC
#define ALLLED_OFF_H 0xFD

class PWM {
private:
  const int FREQUENCY = 50;
  const int ADDRESS = 0x40;

  bool Debug;

  int fd;

  float prescaleval;
  float prescale;

  void setPWMFreq();

public:
  PWM(bool debug = false);
  void setPWM(int channel, int on, int off);
};
#endif
PWM::PWM(bool debug) {

  Debug = debug;

  int errnr;
  // Init Api
  errnr = wiringPiSetupGpio();
  if (Debug) cout << "wiringPiSetupGPIO()" << " " << strerror(errnr) << endl;

  // GPIO Ports 2 and 3 set to output
  // if wiringPiSetup() is used, then wiringPi pins are necessary (8 = 2, 9 = 3)
  //  pinMode(2, OUTPUT);
  //  pinMode(3, OUTPUT);

  fd = wiringPiI2CSetup(ADDRESS);
  if (Debug) cout << "wiringPiI2CSetup(0x" << hex << ADDRESS << ")" << " " << strerror(fd) << endl;

  // reset device
  if (Debug) cout << "Reseting PCA9685" << endl;
  errnr = wiringPiI2CWriteReg8(fd, MODE1, 0x00);
  if (Debug) cout << "write 0x" << hex << 0x00 << " to register 0x" << hex << MODE1 << " " << strerror(errnr) << endl;

  setPWMFreq();

  //  pinMode(2, INPUT);
  //  pinMode(3, INPUT);
}
void PWM::setPWMFreq() {

  prescaleval = 25000000.0f;
  prescaleval /= 4096.0;
  prescaleval /= (float) FREQUENCY;
  prescaleval -= 1.0;

  int errnr;

  if (Debug) {
    cout << "Setting PWM frequency to " << dec << FREQUENCY << " Hz" << endl;
    cout << "Estimated pre-scale: " << dec << prescaleval << endl;
  }

  prescale = floor(prescaleval + 0.5);

  if (Debug) {
    cout << "Final pre-scale: " << dec << prescale << endl;
  }

  int oldmode = wiringPiI2CReadReg8(fd, MODE1);
  int newmode = (oldmode & 0x7F) | 0x10;

  if (Debug) cout << "red 0x" << hex << oldmode << " from register 0x" << hex << MODE1 << " " << strerror(oldmode) << endl;
  if (Debug) cout << "oldmode: " << dec << oldmode << endl;
  if (Debug) cout << "newmode: " << dec << newmode << endl;


  errnr = wiringPiI2CWriteReg8(fd, MODE1, newmode);
  if (Debug) cout << "write 0x" << hex << newmode << " to register 0x" << hex << MODE1 << " " << strerror(errnr) << endl;

  errnr = wiringPiI2CWriteReg8(fd, PRESCALE, (int) floor(prescale));
  if (Debug) cout << "write 0x" << hex << ((int) floor(prescale)) << " to register 0x" << hex << PRESCALE << " " << strerror(errnr) << endl;

  errnr = wiringPiI2CWriteReg8(fd, MODE1, oldmode);
  if (Debug) cout << "write 0x" << hex << oldmode << " to register 0x" << hex << MODE1 << " " << strerror(errnr) << endl;

  sleep(0.005);

  errnr = wiringPiI2CWriteReg8(fd, MODE1, oldmode | 0x80);
  if (Debug) cout << "write 0x" << hex << (oldmode | 0x80) << " to register 0x" << hex << MODE1 << " " << strerror(errnr) << endl;
}
void PWM::setPWM(int channel, int on, int off) {

  int errnr;

  if (Debug) cout << "setPWM" << endl;
  //  pinMode(2, OUTPUT);
  //  pinMode(3, OUTPUT);


  errnr = wiringPiI2CWriteReg8(fd, LED0_ON_L + 4 * channel, on & 0xFF);
  if (Debug) cout << "write 0x" << hex << (on & 0xFF) << " to register 0x" << hex << (LED0_ON_L + 4 * channel) << " " << strerror(errnr) << endl;


  errnr = wiringPiI2CWriteReg8(fd, LED0_ON_H + 4 * channel, on >> 8);
  if (Debug) cout << "write 0x" << hex << (on >> 8) << " to register 0x" << hex << (LED0_ON_H + 4 * channel) << " " << strerror(errnr) << endl;


  errnr = wiringPiI2CWriteReg8(fd, LED0_OFF_L + 4 * channel, off & 0xFF);
  if (Debug) cout << "write 0x" << hex << (off & 0xFF) << " to register 0x" << hex << (LED0_OFF_L + 4 * channel) << " " << strerror(errnr) << endl;

  errnr = wiringPiI2CWriteReg8(fd, LED0_OFF_H + 4 * channel, off >> 8);
  if (Debug) cout << "write 0x" << hex << (off >> 8) << " to register 0x" << hex << (LED0_OFF_H + 4 * channel) << " " << strerror(errnr) << endl;

  //  pinMode(2, INPUT);
  //  pinMode(3, INPUT);
}
