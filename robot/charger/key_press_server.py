#!/usr/bin/env python
# -*- coding: utf-8 -*-
import curses
import socket
import ast
import json

host = "192.168.1.201"
port = 8081
stdscr = curses.initscr()
curses.cbreak()
stdscr.keypad(1)

stdscr.refresh()

key = ''
while key != ord('q'):
    key = stdscr.getch()
    stdscr.refresh()
    if key ==ord('c'):
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.connect((host, port))
        s.sendall(('\xFA\x55\x09\x04\x00\xF3'))
        data = s.recv(1024)
        s.close()
        print('Received', repr(data))
    elif key == ord('t') : 
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.connect((host, port))
        s.sendall(('\xFA\x55\x0B\x04\x00\xF1'))
        data = s.recv(1024)
        #jdata = json.load(data.decode('utf-8'))
        #parsed = json.loads(data)
        #print(type(parsed))
        #dic_data = ast.literal_eval(data)
        print(type(data))
        s.close()
        print('Received', repr(data))
    elif key == ord('s') :
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.connect((host, port))
        s.sendall('\xFA\x55\x0B\x04\x00\xF1')
        data = s.recv(1024)
        s.close()
        print('Received', repr(data))
    elif key == ord('g') :
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.connect((host, port))
        s.sendall('\xFA\x55\x01\x04\x00\xFB')
        data = s.recv(1024)
        s.close()
        print('Received', repr(data))
    elif key == ord('h') :
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.connect((host, port))
        s.sendall('\xFA\x55\x03\x05\x00\x06\xF2')
        data = s.recv(1024)
        s.close()
        print('Received', repr(data))


curses.endwin()
