#-*- coding:utf8 -*-
import time,os,sys      #http://pythonhosted.org/RPIO/
import RPi.GPIO as GPIO #http://sourceforge.net/p/raspberry-gpio-python/wiki/Inputs/
GPIO.setmode(GPIO.BOARD)
GPIO.setup(7, GPIO.IN,pull_up_down=GPIO.PUD_DOWN)
GPIO.add_event_detect(7, GPIO.RISING)
while 1:
    if GPIO.event_detected(7):
        print('Rising')
        break
time.sleep(3)
GPIO.cleanup()