#!/usr/bin/env python
# -*- coding: utf-8 -*-

from utils import *
from ctypes import *
import time
import serial
from enum import Enum
import struct
import rospy
import sys
import math
import numpy
from std_msgs.msg import Int32,UInt32,Bool,String,Float32
from sensor_msgs.msg import Range
from geometry_msgs.msg import Vector3, Quaternion
from rospy.numpy_msg import numpy_msg
from subprocess import call

class StateRec(Enum):
    STATE_REC_HEADER = 0,
    STATE_REC_CMD = 1,
    STATE_REC_DATA = 2

class ResPacket(Structure):
    _fields_ = [
        ("header", c_uint16),
        ("command", c_ubyte),
        ("length", c_uint16),
        ("ack", c_ubyte),
        ("data", c_ubyte*256)
    ]


class SerialTranfer():
    __SEL_ACK = 0
    __SEL_NACK = 1
    __SEL_RESEND_MAX_TIME = 5
    __SEL_SERIAL_WAIT_READ = 10  #ms
    __SEL_TIME_SLEEP_ERROR = 0.0000001 #second
    
    def __init__(self,port,baud,readTimeOut,writeTimeOut):
        self.serPort = serial.Serial(port = port, baudrate = baud, timeout = readTimeOut, write_timeout = writeTimeOut)

    def __CalChecksum(self,data,len):
        checkSum = 0
        for i in range(0,len):
            try:
                checkSum = (checkSum + ord(data[i]))&0xFF
            except:
                checkSum = (checkSum + data[i])&0xFF
        checkSum = ((~checkSum) + 1)&0xFF
        return checkSum
    
    def __SendCmd(self,msg):
        # print(msg)
        try:
            if(False == self.serPort.isOpen()):
                self.serPort.open()
            if(len(msg) <= self.serPort.write(msg)):
                return True
        except:
            self.ClosePort()
            time.sleep(__SEL_TIME_SLEEP_ERROR)
            printErr("Send has been error")
            pass
        printErr("Send fail")
        return False
    
    def __MakePacket(self,cmd, data=""):
        '''
        Header      Command     Length      Data    CRC
        2byte       1byte       2byte       nbyte   1byte
        0xFA55 
        '''
        pkt = [0xFA,0x55]
        pkt.append(cmd)
        length = len(data) + 4
        pkt.append(length&0xFF)
        pkt.append((length>>8)&0xFF)
        for byte in data:
            pkt.append(byte)
        crc = self.__CalChecksum(pkt[2:],length-1)
        pkt.append(crc)
        return pkt

    def ClosePort(self):
        if(True == self.serPort.isOpen()):
            self.serPort.close()

    
    def Tranfer(self,cmd,data=""):
        '''
        return result,data
        '''
        packRec = ResPacket()
        numResent = 0
        pack = self.__MakePacket(cmd,data)
        stateRec = StateRec.STATE_REC_HEADER
        while(True):
            if(numResent < self.__SEL_RESEND_MAX_TIME):
                if(False == self.__SendCmd(pack)):
                    printErr("Send data fail : %u"%numResent)
                    return False,""
                else:
                    lenRx=0
                    dataRead = ""
                    stateRec = StateRec.STATE_REC_HEADER
                    numResent += 1
                    # print("Send data %u"%numResent)
            else:
                printErr("Send data fail : %u"%numResent)
                return False,""
            # printOk("Waitting response")          
            preTime = int(round(time.time() * 1000))
            while (True):
                if (int(round(time.time() * 1000))) - preTime >= self.__SEL_SERIAL_WAIT_READ:
                    self.serPort.close()
                    printErr("Time out wait response")
                    break
                    # return False,""
                try:
                    if(self.serPort.in_waiting > 0):
                        data = self.serPort.read()
                        preTime = int(round(time.time()*1000))
                        if(stateRec == StateRec.STATE_REC_HEADER):
                            lenRx = 0
                            if(ord(data) == 0xFA):
                                packRec.header = 0xFA
                            if(packRec.header&0xFF == 0xFA):
                                if(ord(data) == 0x55):
                                    packRec.header = 0
                                    stateRec = StateRec.STATE_REC_DATA
                            else:
                                packRec.header = 0
                        elif(stateRec == StateRec.STATE_REC_DATA):
                            if(lenRx < 3):
                                dataRead += data
                                lenRx += 1
                                if(lenRx == 3):
                                    packRec.command = ord(dataRead[0])
                                    packRec.length = (ord(dataRead[2])<<8|ord(dataRead[1]))
                            else:
                                if(lenRx < packRec.length):
                                    dataRead += data
                                    lenRx += 1
                                    if(lenRx == packRec.length):
                                        if(ord(dataRead[lenRx - 1]) == self.__CalChecksum(dataRead,len(dataRead)-1)):
                                            packRec.command = ord(dataRead[0])
                                            # print("packRec.command %u"%packRec.command)
                                            if((cmd + 1) == packRec.command):
                                                packRec.ack = ord(dataRead[3])
                                                if(self.__SEL_ACK == packRec.ack):
                                                    # printOk("Tranfer success")
                                                    # stateRec = StateRec.STATE_REC_HEADER
                                                    if(packRec.length > 5):
                                                        self.ClosePort()
                                                        return True,dataRead[4:len(dataRead)-1]
                                                    else:
                                                        self.ClosePort()
                                                        return True,""
                                                else:
                                                    self.ClosePort()
                                                    time.sleep(__SEL_TIME_SLEEP_ERROR)
                                                    printErr("Nack")
                                                    break
                                            else:
                                                self.ClosePort()
                                                time.sleep(__SEL_TIME_SLEEP_ERROR)
                                                printErr("Cmd response false")
                                                return False,""
                                                # break
                                        else:
                                            self.ClosePort()
                                            time.sleep(__SEL_TIME_SLEEP_ERROR)
                                            printErr("Crc false")
                                            return False,""                
                except:
                    self.ClosePort()
                    time.sleep(__SEL_TIME_SLEEP_ERROR)
                    printErr("serial read fail")
                    return False,""	 


class ComBoard(SerialTranfer):

    # command
    __CMD_NONE = 0
    __CMD_GET_ENCODER_DATA = 0x20
    __RES_GET_EBCODER_DATA = 0x21
    __CMD_GET_MAGNETIC_DATA = 0x22
    __RES_GET_MAGNETIC_DATA = 0x23
    __CMD_GET_COLLECTED_DATA = 0x24 
    __RES_GET_COLLECTED_DATA = 0x25
    __CMD_GET_DIAGNOTICS_DATA = 0x26 
    __RES_GET_DIAGNOTICS_DATA = 0x27 
    __CMD_CTRL_TWHEEL = 0x28
    __RES_CTRL_TWHEEL = 0x29
    __CMD_CTRL_RWHEEL = 0x2A
    __RES_CTRL_RWHEEL = 0x2B
    __CMD_CTRL_LIFT_MT = 0x2C
    __RES_CTRL_LIFT_MT = 0x2D
    __CMD_CTRL_CHARGE_SW = 0x2E
    __RES_CTRL_CHARGE_SW = 0x2F
    __CMD_CTRL_PC_ERROR = 0x30
    __RES_CTRL_PC_ERROR = 0x31
    __CMD_CTRL_PW_LASER = 0x32
    __RES_CTRL_PW_LASER = 0x33

    #const
    __REQUEST_TURNON_LASER_SENSOR = 3000
    __REQUEST_TURNOFF_LASER_SENSOR = 1000
    __REQUEST_RESET_RUN_BUTTON = 2000
    __REQUEST_CHARGE = 5000

    def __init__(self,port,baud,readTimeOut,writeTimeOut):
        SerialTranfer.__init__(self,port,baud,readTimeOut,writeTimeOut) 
        rospy.init_node("STM32_COM")
        self.nodename = rospy.get_name()
        self.rate = rospy.get_param('~rate', 50)

        LogInit("/home/seldat/Seldat_Robot_Unilever/src/seldat_robot/src/Log_STM32_COM")
        rospy.loginfo("%s started" % self.nodename)
        printOk("Initializing STM32 Class -----> (0_0) -> (^_^) -> (-_-)")
        
        self.t_vel = 0
        self.r_vel = 0
        self.r_vel_rad = 0.0
        self.controllift = 0
        self.charge_cmd = 0
        self.wifi_msg = False
        self.lidar_msg = False
        self.laserOff = 1 #turnoff laser
		
        self.t_vel_pub = False
        self.r_vel_pub = False
        self.r_vel_rad_pub = False
        self.controllift_pub = False
        self.charge_cmd_pub = False
        self.wifi_msg_pub = False
        self.lidar_msg_pug = False
        self.laser_on_pub = True

        self.encoderMsg		= Vector3()
        self.magneticfrontMsg	= String()
        self.magneticbackMsg	= String()
        self.magneticleftMsg	= String()
        self.magneticrightMsg	= String()
        self.manualMsg		= Int32()		
        self.liftMsg		= UInt32()
        self.batteryMsg		= Int32()
        self.poweroffMsg	= Int32()
        self.magOnMsg		= False
        self.runbuttonMsg	= Int32()
        self.loadcellMsg	= Int32()
        self.warningMsg		= String()
        self.errorMsg		= String()

        self.encoders_pub 	= rospy.Publisher('pos',Vector3,queue_size = 10)
        self.magnetic_front_pub	= rospy.Publisher('magnetfront',String,queue_size = 10)
        self.magnetic_back_pub 	= rospy.Publisher('magnetback',String,queue_size = 10)
        self.magnetic_left_pub	= rospy.Publisher('magnetleft',String,queue_size = 10)
        self.magnetic_right_pub = rospy.Publisher('magnetright',String,queue_size = 10)
        self.manual_pub		= rospy.Publisher('manual',Int32,queue_size = 10)
        self.lift_pub		= rospy.Publisher('lift',UInt32,queue_size = 10)
        self.batt_pub		= rospy.Publisher('battery_vol',Int32,queue_size = 10)
        self.error_pub		= rospy.Publisher('stm_error',String,queue_size = 10)
        self.warning_pub	= rospy.Publisher('stm_warning',String,queue_size = 10)
        self.loadcell_pub	= rospy.Publisher('loadcell',Int32,queue_size = 10)
        self.runbutton_pub	= rospy.Publisher('run_button',Int32,queue_size = 10)
        #self.encoderMsg.header  = std_msgs.msg.Header()
        #self.sonarMsg.header.stamp = rospy.Time.now()

        #self.frameid = ['/ultrasound']

        rospy.Subscriber("twheel", Int32, self.twheelCallback)
        rospy.Subscriber("swheel", Int32, self.rwheelCallback)
        rospy.Subscriber("swheel_rtarget",Float32, self.rwheelRadCallback)
        rospy.Subscriber("lift_control", String, self.liftCallback)
        rospy.Subscriber("ctrlRobotHardware", Int32, self.chargeCallback)
        rospy.Subscriber("serverChecker", Bool, self.wifiCallback)
        rospy.Subscriber("/lidar/laser_error", Bool, self.lidarCallback)
        rospy.Subscriber("line_enable", Int32, self.LineDetectionCallback)
        
    def spin(self):
        time_t = 0
        r = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            try:
                # print ("================(<_>)GetEncoderValue(<_>)================")  
                if(False == self.GetEncoderValue()):
                    printErr ("================(<_>)Update_encs failed(<_>)================")  
                time.sleep(time_t)

                # print ("================(<_>)GetMagnetic(<_>)================") 
                if(False == self.GetMagnetic()):
                    printErr ("================(<_>)Update_magnetic failed(<_>)================")
                time.sleep(time_t)

                # print ("================(<_>)Update_collect (<_>)================")
                if(False == self.GetCollected()):
                    printErr ("================(<_>)Update_collect failed(<_>)================")
                time.sleep(time_t)

                # print ("================(<_>)Update_diagnotic(<_>)================") 
                if(False == self.GetDiagnotics()):
                    printErr ("================(<_>)Update_diagnotic failed(<_>)================") 
                time.sleep(time_t)

                if (self.manualMsg == 0):
                    if (self.t_vel_pub == True):
                        # print("CtrlTWheel")
                        pak = struct.pack('<h',self.t_vel)
                        # print(pak)
                        if(False == self.CtrlTWheel(pak)):
                            printErr ("================(<_>)CtrlTWheel failed(<_>)================")  
                        self.t_vel_pub = False
                        
                    if (self.r_vel_pub == True):
                        # print("CtrlRWheel")
                        pak = struct.pack('<h',self.r_vel)
                        # print(pak)
                        if(False == self.CtrlRWheel(pak)):
                            printErr ("================(<_>)CtrlRWheel failed(<_>)================")
                        self.r_vel_pub = False
                    if (self.r_vel_rad_pub == True):
                        # print("CtrlRWheel")
                        r_vel_rad_ = int(5250 - (3183*self.r_vel_rad))
                        pak = struct.pack('<h',r_vel_rad_)
                        # print(pak)
                        if(False == self.CtrlRWheel(pak)):
                            printErr ("================(<_>)CtrlRWheelRad failed(<_>)================")
                        self.r_vel_rad_pub = False
                        
                    if (self.controllift_pub == True):
                        # print("CtrlLiftMt")
                        #print(self.controllift)
                        pak = struct.pack('<B',self.controllift)
                        # print(pak)
                        if(False == self.CtrlLiftMt(pak)):
                            printErr ("================(<_>)CtrlLiftMt failed(<_>)================")
                        self.controllift_pub = False
                        
                    if (self.charge_cmd_pub == True):
                        # print("CtrlChargeSw")
                        pak = struct.pack('<B',self.charge_cmd)
                        # print(pak)
                        if(False == self.CtrlChargeSw(pak)):
                            printErr ("================(<_>)CtrlChargeSw failed(<_>)================")
                        self.charge_cmd_pub = False
                        
                    if (self.wifi_msg_pub == True):
                        # printErr("CtrlPcError")
                        pak = struct.pack('<B',self.wifi_msg)
                        # print(pak)
                        if(False == self.CtrlPcError(pak)):
                            printErr ("================(<_>)CtrlPcError failed(<_>)================")
                        self.wifi_msg_pub = False

                    if (self.lidar_msg_pug == True):
                        # print("lidar_msg_pug")
                        pak = struct.pack('<B',self.lidar_msg)
                        # print(pak)
                        if(False == self.CtrlPcError(pak)):
                            printErr ("================(<_>)CtrlPcError failed(<_>)================")
                        self.lidar_msg_pug = False
                        
                    if (self.laser_on_pub == True):
                        # print("CtrlPwLaser")
                        pak = struct.pack('<B',self.laserOff)
                        # print(pak)
                        if(False == self.CtrlPwLaser(pak)):
                            printErr ("================(<_>)CtrlPwLaser failed(<_>)================")
                        self.laser_on_pub = False
                r.sleep()
            except:
                printErr("error serial (--_--)")
                time.sleep(1)

    def GetEncoderValue(self):
        ret,data = self.Tranfer(self.__CMD_GET_ENCODER_DATA)
        if(ret == True):
            dataT = struct.unpack('<lh', data)
            # print(dataT)
            self.encoderMsg.x = int(dataT[0]) #tracking
            self.encoderMsg.z = long(dataT[1]) #rotating
            #printOk("time : %d"%(time.time()*1000000))
            #printOk("enc_T: %d"%self.encoderMsg.x)
            #printOk("enc_R %d"%self.encoderMsg.z)
            self.encoders_pub.publish(self.encoderMsg)
            return True
        else:
            printErr("Cannot GetEncoderValue")
            return False

    def GetMagnetic(self):
        ret,data = self.Tranfer(self.__CMD_GET_MAGNETIC_DATA)
        # print(data)
        if(ret == True):
            self.magneticfrontMsg 	= str(data[0:16])
            # print(self.magneticfrontMsg)
            self.magneticbackMsg 	= str(data[16:32])
            # print(self.magneticbackMsg)
            self.magneticleftMsg 	= str(data[32:34])
            # print(self.magneticleftMsg)
            self.magneticrightMsg 	= str(data[34:])
            # print(self.magneticrightMsg)
        
            if (self.magOnMsg == True):
                self.magnetic_front_pub.publish(self.magneticfrontMsg)
                self.magnetic_back_pub.publish(self.magneticbackMsg)
                self.magnetic_left_pub.publish(self.magneticleftMsg)
                self.magnetic_right_pub.publish(self.magneticrightMsg)
            return True
        else:
            printErr("Cannot GetMagnetic")
            return False

    def GetCollected(self):
        ret,data = self.Tranfer(self.__CMD_GET_COLLECTED_DATA)
        # print(data)
        if(ret == True):
            dataT = struct.unpack('<BHBBB', data[0:6])
            # print(dataT)

            self.battvoltageMsg = int(dataT[0])
            # print(self.battvoltageMsg)
            self.batt_pub.publish(self.battvoltageMsg)

            self.loadcellMsg = int(dataT[1])
            # print(self.loadcellMsg)
            self.loadcell_pub.publish(self.loadcellMsg)

            self.manualMsg = int(dataT[2])
            # print(self.manualMsg)
            self.manual_pub.publish(self.manualMsg)
        
            self.liftMsg = int(dataT[3])
            # print(self.liftMsg)
            self.lift_pub.publish(self.liftMsg)

            self.poweroffMsg = int(dataT[4])
            # print(self.poweroffMsg)

            if (self.poweroffMsg == 1):
                rospy.loginfo("====Computer Shutdown====")
                time.sleep(1)
                call("sudo shutdown -h now", shell=True)
                time.sleep(30)
            elif (self.poweroffMsg == 2):
                rospy.loginfo("====Computer Restart=====")
                time.sleep(1)
                #os.system("shutdown now -h")
                call("sudo reboot -h now", shell=True)
                time.sleep(30)

            return True
        else:
            printErr("Cannot GetCollected")
            return False

    def GetDiagnotics(self):
        ret,data = self.Tranfer(self.__CMD_GET_DIAGNOTICS_DATA)
        # print(data)
        if(ret == True):
            dataT = struct.unpack('<B', data[0])
            self.runbuttonMsg =	int(dataT[0])
            # print(self.runbuttonMsg)
            self.errorMsg	=	str(data[1:4])
            # print(self.errorMsg)
            self.warningMsg =	str(data[4:])
            # print(self.warningMsg)
            
            self.runbutton_pub.publish(self.runbuttonMsg)
            self.error_pub.publish(self.errorMsg)
            self.warning_pub.publish(self.warningMsg)
            return True
        else:
            printErr("Cannot GetDiagnotics")
            return False

    def CtrlTWheel(self,data):
        ret,data = self.Tranfer(self.__CMD_CTRL_TWHEEL,data)
        if(ret == True):
            return True
        else:
            printErr("Cannot CtrlTWheel")
            return False  

    def CtrlRWheel(self,data):
        ret,data = self.Tranfer(self.__CMD_CTRL_RWHEEL,data)
        if(ret == True):
            return True
        else:
            printErr("Cannot CtrlRWheel")
            return False 
    def CtrlLiftMt(self,data):
        ret,data = self.Tranfer(self.__CMD_CTRL_LIFT_MT,data)
        if(ret == True):
            return True
        else:
            printErr("Cannot CtrlLiftMt")
            return False 

    def CtrlChargeSw(self,data):
        ret,data = self.Tranfer(self.__CMD_CTRL_CHARGE_SW,data)
        if(ret == True):
            return True
        else:
            printErr("Cannot CtrlChargeSw")
            return False 

    def CtrlPcError(self,data):
        ret,data = self.Tranfer(self.__CMD_CTRL_PC_ERROR,data)
        if(ret == True):
            return True
        else:
            printErr("Cannot CtrlPcError")
            return False  

    def CtrlPwLaser(self,data):
        ret,data = self.Tranfer(self.__CMD_CTRL_PW_LASER,data)
        if(ret == True):
            return True
        else:
            printErr("Cannot CtrlPwLaser")
            return False
    
    def twheelCallback(self, msg):
        self.t_vel = msg.data
        self.t_vel_pub = True

    def rwheelCallback(self, msg):
        self.r_vel =  msg.data
        self.r_vel_pub = True

    def rwheelRadCallback(self,msg):
        self.r_vel_rad = msg.data
        self.r_vel_rad_pub = True

    def liftCallback(self,msg):
        if msg.data == "lift_up":
            # print("lift_up")
            self.controllift = 1
        elif msg.data == "lift_down":
            # print("lift_down")
            self.controllift = 2
        elif msg.data == "lift_stop":
            # print("lift_stop")
            self.controllift = 0
        self.controllift_pub = True

    def chargeCallback(self,msg):
        if(msg.data == self.__REQUEST_CHARGE):
            self.charge_cmd = 1
        else:
            self.charge_cmd = 0
        self.charge_cmd_pub = True

    def wifiCallback(self,msg):
        if (msg.data == False):
            self.wifi_msg = 0
        elif (msg.data == True):
            self.wifi_msg = 1
        self.wifi_msg_pub = True

    def lidarCallback(self,msg):
        if (msg.data == False):
            self.lidar_msg = 0
        elif (msg.data == True):
            self.lidar_msg = 1
        self.lidar_msg_pug = True

    def LineDetectionCallback(self,msg):
        if (msg.data == self.__REQUEST_TURNOFF_LASER_SENSOR):
            # print("__REQUEST_TURNOFF_LASER_SENSOR")
            self.magOnMsg = True
            self.laserOff = 1
        elif (msg.data == self.__REQUEST_TURNON_LASER_SENSOR):
            # print("__REQUEST_TURNON_LASER_SENSOR")
            self.magOnMsg = False
            self.laserOff = 0
        self.laser_on_pub = True	

if __name__ == "__main__":

    sCom = ComBoard(port = "/dev/STM32F4", baud = 921600, readTimeOut = 0.005, writeTimeOut = 0.005)
    try:
        sCom.spin()
    except:
        printErr("Robot note Loi roi (>_<)")
