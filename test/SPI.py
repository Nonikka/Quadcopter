#-*- coding:utf8 -*-
import serial,time #http://pythonhosted.org/pyserial/pyserial_api.html

from trans import *
ser = serial.Serial('/dev/ttyAMA0', 115200, timeout=1)
ser.open()

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
        time.sleep(0.1)
        num_in_waiting = ser.inWaiting()
        if num_in_waiting < 100:   #预防无数据
            raise KeyboardInterrupt
        str = ser.read(num_in_waiting)
        print 'num in waiting:',num_in_waiting
        str_hex = trans_to_hex(str)
        
        print str_hex
        print str_hex[0:2],
        #if str_hex[0:2] == '55':
        COUNTER = 0
        while(1):
            if str_hex[COUNTER : COUNTER + 1] == '5':
                if str_hex[COUNTER + 1 : COUNTER + 2 ] == '5':
                    if str_hex[COUNTER + 3 : COUNTER + 5] == '51':
                    print ' yes'
                    #AxL = int(hex2bin(str_hex[ 6 : 8]))
                    #AxH = int(hex2bin(str_hex[ 9 : 11]))
                    AxL = int(str_hex[COUNTER + 6 : COUNTER + 8 ])
                    AxH = int(str_hex[COUNTER + 9 : COUNTER + 11])
                    Ax = (((AxH<<8))|AxL)/32768*16
                    print 'AxL ',str_hex[COUNTER + 6 :COUNTER + 8],AxL
                    print 'AxH ',str_hex[COUNTER + 9 :COUNTER + 11],AxH,(AxH<<8)
                    print 'Ax = ',Ax,'(((AxH<<8))|AxL)=',(((AxH<<8))|AxL)
                    else:
                        COUNTER = COUNTER + 1
                else:
                    COUNTER = COUNTER + 1
            else:
                COUNTER = COUNTER + 1
            if COUNTER > 50:
                print 'May not get the data,break...'
                break
    except KeyboardInterrupt:
        ser.close()

if __name__ == '__main__':
    main()