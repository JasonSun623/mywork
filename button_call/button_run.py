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

counter=0

while 1:
    x=ser.readline().decode('utf-8',errors='replace')
    print(x)
#ser.write(str.encode('AT+C005'))
#print(ser.readline().decode())
