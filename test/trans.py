#-*- coding:utf8 -*-
#http://www.cnblogs.com/txw1958/archive/2013/08/29/python3-scale.html
base = [str(x) for x in range(10)] + [ chr(x) for x in range(ord('A'),ord('A')+6)]

def bin2dec(string_num):
    return str(int(string_num, 2))
    
def hex2dec(string_num):
    return str(int(string_num.upper(), 16))
    
def dec2bin(string_num):
    num = int(string_num)
    mid = []
    while True:
        if num == 0: break
        num,rem = divmod(num, 2)
        mid.append(base[rem])
    return ''.join([str(x) for x in mid[::-1]])
    
def dec2hex(string_num):
    num = int(string_num)
    mid = []
    while True:
        if num == 0: break
        num,rem = divmod(num, 16)
        mid.append(base[rem])
    return ''.join([str(x) for x in mid[::-1]])
    
def hex2bin(string_num):
    str = dec2bin(hex2dec(string_num.upper()))
    if len(str) < 8:
        str = "0" * (8 - len(str)) + str
    return str
    
def bin2hex(string_num):
    return dec2hex(bin2dec(string_num))