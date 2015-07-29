#-*- coding:utf8 -*-
#MPU6050接法:GND → 6
#            VCC → 1
#            RX → 8
#            TX → 10
import serial,time,sys,os #http://pythonhosted.org/pyserial/pyserial_api.html
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

def main():
    try:
        COUNTER = 0
        while(1):
            ser.open()
            ser.flushInput()
            time.sleep(0.04)
            num_in_waiting = ser.inWaiting()
            if num_in_waiting < 100:   #预防无数据
                raise KeyboardInterrupt
            str = ser.read(num_in_waiting)
            print 'num in waiting:',num_in_waiting
            str_hex = trans_to_hex(str)
            print str_hex
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
                        break
                    else:
                        COUNTER = COUNTER + 1
                else:
                    COUNTER = COUNTER + 1
                if COUNTER > 100:
                    print 'May not get the data,break...'
                    break
            raw_input("输入继续")
            ser.close()
    except KeyboardInterrupt:
        ser.close()

if __name__ == '__main__':
    main()