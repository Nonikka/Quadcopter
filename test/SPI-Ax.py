#-*- coding:utf8 -*-
#GND → 6
#VCC → 1
#RX → 8
#TX → 10
import serial,time,sys #http://pythonhosted.org/pyserial/pyserial_api.html
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
            time.sleep(0.1)
            num_in_waiting = ser.inWaiting()
            if num_in_waiting < 100:   #预防无数据
                raise KeyboardInterrupt
            str = ser.read(num_in_waiting)
            print 'num in waiting:',num_in_waiting
            str_hex = trans_to_hex(str)
            print str_hex
            while(1):
                if str_hex[COUNTER] == '5':
                    if str_hex[COUNTER + 3 : COUNTER + 5] == '51':
                        os.system('clear')
                        print ' yes'
                        AxL = int(str_hex[COUNTER + 6 :COUNTER + 8])
                        AxH = int(str_hex[COUNTER + 9 :COUNTER + 11])
                        Ax = (AxH*(2**8)+AxL)/32768*16
                        print 'AxL ',str_hex[COUNTER + 6 :COUNTER + 8],AxL
                        print 'AxH ',str_hex[COUNTER + 9 :COUNTER + 11],AxH,AxH*(2**8)
                        print 'Ax = ',Ax,'(AxH*(2**8)+AxL)=',(AxH*(2**8)+AxL)
                        COUNTER = 0
                        break
                    else:
                        COUNTER = COUNTER + 1
                else:
                    COUNTER = COUNTER + 1
                if COUNTER > 50:
                    print 'May not get the data,break...'
                    break
            raw_input("输入继续")
            ser.close()
    except KeyboardInterrupt:
        ser.close()

if __name__ == '__main__':
    main()