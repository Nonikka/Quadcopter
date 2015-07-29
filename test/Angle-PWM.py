#-*- coding:utf8 -*-
#MPU6050接法:GND → 6
#            VCC → 1
#            RX → 8
#            TX → 10
import serial,time,sys,os #http://pythonhosted.org/pyserial/pyserial_api.html
import RPi.GPIO as GPIO
from trans import *
ser = serial.Serial('/dev/ttyAMA0', 115200, timeout=1)

def trans_to_hex(argv):  
    result = ''  
    hLen = len(argv)  
    for i in xrange(hLen):  
        hvol = ord(argv[i])  
        hhex = '%02x'%hvol  
        result += hhex+' '  
    return result 
def Roll_to_PWM(angle):
    if angle <= 160 and angle >= 0:
        return angle/1.6
    else :
        return abs((angle-360)/1.6)
    

def main():
    try:
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(12, GPIO.OUT)
        PWM1 = GPIO.PWM(12, 490)
        PWM1.start(95)
        #press_any_key_exit('按任意键设置最低行程50：')
        raw_input("按任意键设置最低行程54：")
        PWM1.ChangeDutyCycle(54)
        _Accelerator = 0
        time.sleep(1)
        print  "Start!"
        PWM1.ChangeDutyCycle(50 + 0.5 * 6)
        COUNTER = 0
        Roll_PWM = 0
        Time_Start = time.time()
        Num_of_Time = 0
        ser.open()
        while(1):
            #ser.open()
            #ser.flushInput()
            time.sleep(0.03)
            num_in_waiting = ser.inWaiting()
            if num_in_waiting < 100:   #预防无数据
                raise KeyboardInterrupt
            str = ser.read(num_in_waiting)
            str_hex = trans_to_hex(str)
            while(1):
                if str_hex[COUNTER] == '5':
                    if str_hex[COUNTER + 3 : COUNTER + 5] == '53':
                        os.system('clear')
                        print ' yes'
                        RollL = int(str_hex[COUNTER + 6 :COUNTER + 8],16)
                        RollH = int(str_hex[COUNTER + 9 :COUNTER + 11],16)
                        PitchL = int(str_hex[COUNTER + 12 :COUNTER + 14],16)
                        PitchH = int(str_hex[COUNTER + 15 :COUNTER + 17],16)
                        Roll = ((RollH*(2**8))+RollL)/32768.0*180   #C中原算法(ucStrAngle[1]<<8| ucStrAngle[0])/32768.0*180
                        Pitch = ((PitchH)*(2**8)+PitchL)/32768.0*180
                        print 'RollL ',str_hex[COUNTER + 6 :COUNTER + 8],RollL#在python中这样算方便一点
                        print 'RollH ',str_hex[COUNTER + 9 :COUNTER + 11],RollH,RollH*(2**8)
                        print 'Roll = ',Roll,'(RollH*(2**8))+RollL=',(RollH*(2**8))+RollL
                        print 'Pitch = ',Pitch
                        COUNTER = 0
                        DutyCycle = Roll_to_PWM(Roll)
                        PWM1.ChangeDutyCycle(50 + DutyCycle * 0.4)
                        Time_Now = time.time()
                        Num_of_Time = Num_of_Time + 1
                        print 'time: ',Time_Now - Time_Start,' Num:',Num_of_Time
                        print 'num in waiting:',num_in_waiting
                        print str_hex
                        break
                    else:
                        COUNTER = COUNTER + 1
                else:
                    COUNTER = COUNTER + 1
                if COUNTER > 100:
                    print 'May not get the data,break...'
                    break
            #ser.close()
    except KeyboardInterrupt:
        PWM1.stop()
        GPIO.cleanup()
        ser.close()

if __name__ == '__main__':
    main()