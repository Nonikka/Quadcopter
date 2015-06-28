import time
import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BOARD)
GPIO.setup(12, GPIO.OUT)
p = GPIO.PWM(12, 500)
p.start(95)
content = raw_input("input to 50:")
p.ChangeDutyCycle(50)
content = raw_input("input to 60:")
p.ChangeDutyCycle(60)
content = raw_input("input to stop:")
p.stop()
GPIO.cleanup()