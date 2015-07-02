#-*- coding:utf8 -*-
import time,os,sys
import RPi.GPIO as GPIO #http://sourceforge.net/p/raspberry-gpio-python/wiki/Inputs/
GPIO.setmode(GPIO.BOARD)
_Input_pin = [7,8]
GPIO.setup(_Input_pin, GPIO.OUT,pull_up_down=GPIO.PUD_UP) #MPU6050 send 0x55 (01010101) as head
GPIO.add_event_detect(channel, GPIO.RISING)  # add rising edge detection on a channel
GPIO.input() #read the value of the pin
if GPIO.event_detected(channel):
    print('Button pressed')