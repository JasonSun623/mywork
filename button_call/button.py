#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import serial
import requests

ser = serial.Serial(

	port='/dev/ttyUSB0',
	baudrate = 9600,
	parity=serial.PARITY_NONE,
	stopbits=serial.STOPBITS_ONE,
	bytesize=serial.EIGHTBITS,
	timeout=1
    )

#msg = "ok"

while 1:
    x=ser.readline().decode('utf-8',errors='replace').splitlines()
    x = ''.join(map(str, x))
    if len(x) >=6:
        if x[0] == "S" and x[5] == "E":
            print(x[:6])
    else:
        pass
    #if x == "button 1 pressed":
    #    ser.write(b'ok')
    #    x = None
    #elif x == "button 2 pressed":
    #    ser.write(b'ok')
    #else:
    #    pass 
#ser.write(str.encode('AT+C005'))
#print(ser.readline().decode())
