#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Mon Dec 10 09:29:56 2018

@author: dongho
"""

import config
import time
import socket
import ast

def charger_server(cmd,stt):
    while True:        
        if config.host != None:
            try:
                s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                s.connect((config.host, config.port))
                s.sendall(cmd)
                data = s.recv(1024).strip()
                str_dic_data = (data.strip('\0'))
                dict_data = ast.literal_eval(str_dic_data)
                #print dict_data
                config.dict_ = dict_data
                s.close()
                if dict_data['status'] == stt :
                    break
            except socket.error as e:
                #print("socket error {} reconnecting".format(e))
                time.sleep(5)
            except KeyboardInterrupt:
                #print " canceled function"
                break
        else:
            #print "check host"
            time.sleep(5)
                
def charger_server_1(cmd):        
    data0=''
    try:
        # Create a TCP/IP socket
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        # Connect the socket to the port where the server is listening
        server_address = (config.host, config.port)
        sock.settimeout(10)     # TimeOut 5 secunde
        while True:
            try:
                sock.connect(server_address)
                sock.sendall(cmd)
                # Look for the response
                data = sock.recv(1024).strip()
                str_dic_data = (data.strip('\0'))
                dict_data = ast.literal_eval(str_dic_data)
                config.dict_ = dict_data
                ##print "heeeeeee",self.dict_,dict_data
                #time.sleep(3)
                return
            finally:
                pass
    except:
        sock.close()
        time.sleep(10)
        del data0