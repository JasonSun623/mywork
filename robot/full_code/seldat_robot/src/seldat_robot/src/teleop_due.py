#!/usr/bin/env python
# -*- coding: utf-8 -*-
'''
import serial
import time

serial = serial.Serial("/dev/ttyACM0", baudrate = 115200, timeout = 0.001)
'''
import sys
import serial
import time

serial = serial.Serial("/dev/ttyACM0", baudrate = 115200, timeout = 0.01)

if sys.platform[:3] == 'win':
	import msvcrt
	def getkey():
		key = msvcrt.getch()
		if ord(key) == 224: #catch second event with arrow keys
			key = msvcrt.getch()
		return key
elif sys.platform[:3] == 'lin':
	import termios, sys, os
	TERMIOS = termios
	def getkey():
		fd = sys.stdin.fileno()
		old = termios.tcgetattr(fd)
		new = termios.tcgetattr(fd)
		new[3] = new[3] & ~TERMIOS.ICANON & ~TERMIOS.ECHO
		new[6][TERMIOS.VMIN] = 1
		new[6][TERMIOS.VTIME] = 0
		termios.tcsetattr(fd, TERMIOS.TCSANOW, new)
		c = None
		try:
			c = os.read(fd, 1)
		finally:
			termios.tcsetattr(fd, TERMIOS.TCSAFLUSH, old)
		return c
print ("=======================================")
print ("Teleop ready ...")
print ("Key commands")
print ("w: run forward")
print ("s: run backward")
print ("d: turn right")
print ("a: turn left")
print ("f: R = 2047")
print ("space: stop all")
print ("e: increase velocity")
print ("q: decrease velocity")
print ("z: lift up")
print ("c: lift down")
print ("x: stop the lift")
print ("r: restart Arduino DUE")
print ("o: Exit teleop")
print ("=======================================")

t_speed = 0
r_speed = 2047
count_t = 0
d = 0
a = 0
out = 1
while out == 1:
	k = getkey().decode()
	if k == "w":
		if count_t == -1:
			print ("Need to stop the robot...")
		else:
			serial.write("T" + str(t_speed) + "e")
			count_t = 1
		#print ("w_OK")
	elif k == "s":
		if count_t == 1:
			print ("Need to stop the robot...")
		else:
			serial.write("T-" + str(t_speed) + "e")
			count_t = -1
		#print ("s_OK")
	elif k == " ":
		print ("Stop the robot ...")
		serial.write("T1e")
		count_t = 0
		t_speed = 0
		r_speed = 2047
		time.sleep(1)
		#serial.write("R2047\n")
		#time.sleep(0.5)
		#serial.write("L0")
		print ("Done")
		#print ("space")
	elif k == "d":
		r_speed = r_speed - 100
		if r_speed <= 0:
			r_speed = 0
		d = "R" + str(r_speed) + "e"
		serial.write(d)# + "\n")
		#print (d)
		#print r_speed
	elif k == "a":
		r_speed = r_speed + 100
		if r_speed >= 4095:
			r_speed = 4095
		a = "R" + str(r_speed) + "e"
		serial.write(a)# + "\n")
		#print (a)
		#serial.flushOutput()
		#print r_speed
	elif k == "f":
		r_speed = 2047
		serial.write("R2047\n")
	elif k == "e":
		t_speed = t_speed + 100
		if t_speed >= 4095:
			t_speed = 4095
		if count_t == 1:
                	serial.write("T" + str(t_speed) + "e")
		elif count_t == -1:
                	serial.write("T-" + str(t_speed) + "e")
		#print ("e_OK")
	elif k == "q":
		t_speed = t_speed - 100
		if t_speed < 100:
			t_speed = 0
		if count_t == 1:
                	serial.write("T" + str(t_speed) + "e")
        	elif count_t == -1:
                	serial.write("T-" + str(t_speed) + "e")
	elif k == "z":
		print ("Lift up...")
		serial.write("L1\n")
	elif k == "c":
		print ("Lift down...")
		serial.write("L2\n")
	elif k == "x":
		print ("Stop the lift...")
		serial.write("L0\n")
	elif k == "r":
		print ("Restart Arduino DUE...")
		serial.write("S\n")
		time.sleep(3)
		print ("Done")
	elif k == 'o':
		print("Exit the program ...")
		serial.write("T0\n")
		time.sleep(1)
		serial.write("R2047\n")
		serial.close()
		out = 0
		
	if count_t != -1:
		print ("T speed: ",t_speed)
	else:
		print ("T speed: -",t_speed)
	print ("R speed: ",r_speed)

