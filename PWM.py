import time
import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BOARD)
GPIO.setup(12, GPIO.OUT)
p = GPIO.PWM(12, 500)
p.start(50)
time.sleep(1.2)
p.ChangeDutyCycle(80)
time.sleep(10)
p.stop()