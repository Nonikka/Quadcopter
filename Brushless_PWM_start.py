#-*- coding:utf8 -*-
import time,os,sys 
import RPi.GPIO as GPIO
#import termios

#def press_any_key_exit(msg):
#    fd = sys.stdin.fileno()
#    old_ttyinfo = termios.tcgetattr(fd)
#    new_ttyinfo = old_ttyinfo[:]
#    new_ttyinfo[3] &= ~termios.ICANON
#    new_ttyinfo[3] &= ~termios.ECHO
#    sys.stdout.write(msg)
#    sys.stdout.flush()
#    termios.tcsetattr(fd, termios.TCSANOW, new_ttyinfo)
#    os.read(fd, 7)
#    termios.tcsetattr(fd, termios.TCSANOW, old_ttyinfo)
    
def main():
    try:
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(12, GPIO.OUT)
        p = GPIO.PWM(12, 500)
        p.start(95)
        #press_any_key_exit('按任意键设置最低行程50：')
        raw_input("按任意键设置最低行程50：")
        p.ChangeDutyCycle(50)
        _Accelerator = 0
        time.sleep(1)
        print  "Start!"
        p.ChangeDutyCycle(50 + 0.5 * 6)
        while 1:
            try:
                _Accelerator = input("input the accelerator: ")
            except:
                print 'input error!stop'
                break
            if _Accelerator <=3:
                p.ChangeDutyCycle(0)
                print 'Stop'
                break
            if _Accelerator > 100 or _Accelerator < 3:
                _Accelerator = 10
            p.ChangeDutyCycle(50 + 0.5 * _Accelerator)
            
        content = raw_input("input to stop PWM")
        p.stop()
        GPIO.cleanup()
    
    except KeyboardInterrupt:
        GPIO.cleanup()
        
if __name__ == '__main__':
    main()
        