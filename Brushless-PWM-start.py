#-*- coding:utf8 -*-
import time,os,sys
import RPi.GPIO as GPIO
import termios
def press_any_key_exit(msg):
    # 获取标准输入的描述符
    fd = sys.stdin.fileno()
    # 获取标准输入(终端)的设置
    old_ttyinfo = termios.tcgetattr(fd)
    # 配置终端
    new_ttyinfo = old_ttyinfo[:]
    # 使用非规范模式(索引3是c_lflag 也就是本地模式)
    new_ttyinfo[3] &= ~termios.ICANON
    # 关闭回显(输入不会被显示)
    new_ttyinfo[3] &= ~termios.ECHO
    # 输出信息
    sys.stdout.write(msg)
    sys.stdout.flush()
    # 使设置生效
    termios.tcsetattr(fd, termios.TCSANOW, new_ttyinfo)
    # 从终端读取
    os.read(fd, 7)
    # 还原终端设置
    termios.tcsetattr(fd, termios.TCSANOW, old_ttyinfo)
    
GPIO.setmode(GPIO.BOARD)
GPIO.setup(12, GPIO.OUT)
p = GPIO.PWM(12, 500)
p.start(95)
press_any_key_exit('按任意键设置最低行程50：')
p.ChangeDutyCycle(50)
_Accelerator = 0
time.sleep(1)
print  "Start!"
p.ChangeDutyCycle(50 + 0.5 * 6)
while 1:
    _Accelerator = input("input the accelerator: ")
    if _Accelerator <=3:
        p.ChangeDutyCycle(0)
        print 'Stop'
        break
    if _Accelerator > 100 or _Accelerator < 3:
        _Accelerator = 10
    p.ChangeDutyCycle(50 + 0.5 * int(_Accelerator))
    
content = raw_input("input to stop PWM")
p.stop()
GPIO.cleanup()