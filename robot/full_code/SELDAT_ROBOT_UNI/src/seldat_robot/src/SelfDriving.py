#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import math
import json
import os
from subprocess import call
from enum import Enum #sudo pip install enum34
from ruamel import yaml # sudo pip install ruamel.yaml
from std_msgs.msg import String, Bool
from std_msgs.msg import Int16,Int32,Int64, Float32, Float64
from geometry_msgs.msg import PoseStamped,PoseWithCovarianceStamped,Vector3
from actionlib_msgs.msg import GoalStatusArray
from nav_msgs.msg import Odometry
from time import sleep

class Position():
	def __init__(self):
		self.X=0.0;
		self.Y=0.0;
		self.Angle=0.0;
class LimitedPosition():
	def __init__(self):
		self.Xmax=0.0;
		self.Xmin=0.0;
class RobotOrder():
	def __init__(self):
		self.posCheckInDocking=Position();
		self.posDocking=Position();
		self.limitedPosDocking=LimitedPosition();
		self.posCheckInPutAway=Position();
		self.posPutAway=Position();
		self.limitedPosPutAway=LimitedPosition();
class RobotAtReadyArea():
	def __init__(self):
		self.readyArea=Position();
		self.limitedReadyArea=LimitedPosition();
class RobotAtChargeArea():
	def __init__(self):
		self.chargeArea=Position();
		self.limitedChargeArea=LimitedPosition();
		self.url="";
class BatteryInfo():
	def __init__(self):
		self.currentBatteryLevel=0.0;
		self.defautLowBattery=0.0;
		self.flagNeedToCharge=False;
class RequestCommandLineDetect(Enum):
	REQUEST_LINEDETECT_PALLETUP=1203;
	REQUEST_LINEDETECT_PALLETDOWN=1204;
	REQUEST_LINEDETECT_CHARGEAREA=1208;
	REQUEST_LINEDETECT_READYAREA=1208;
	REQUEST_LINEDETECT_REACHING_POSITION=1205;
	REQUEST_CHARGECTRL_START=1206;
	REQUEST_CHARGECTRL_CANCEL=1201;
	REQUEST_LINEDETECT_REFRESH=4000;
class ResponseCommandLineDetect(Enum):
	RESPONSE_LINEDETECT_PALLETUP=3203;
	RESPONSE_LINEDETECT_PALLETDOWN=3204;
	RESPONSE_FINISH_DETECTLINE_CHARGEAREA=3206;
	RESPONSE_FINISH_RETURN_LINE_CHARGEAREA=3207;
class CallErrorLineDetect(Enum):
	CallERROR_LINEDETECTION=4205;

	
class StatesSelfDriving(Enum):

	
	PROCESS_SELFDRIVING_START_READY = 5000 #50000;
	PROCESS_SELFDRIVING_RESET = 5001#50015;
	PROCESS_SELFDRIVING_STOP  = 5002#50016;
	PROCESS_SELFDRIVING_PAUSE  = 5003#50017;
	PROCESS_SELFDRIVING_IDLE= 5004  #9879;

	ROBOT_STATUS_IDLE 	 =5005 #1000;
	ROBOT_STATUS_READY     = 5006 #1001;
	ROBOT_STATUS_ORDERED   = 5007 #1002;
	ROBOT_STATUS_WORKING   =5008#1003;
	ROBOT_STATUS_FINISH    =5009#1004;
	ROBOT_STATUS_CANCELED  =5010#1005;
	ROBOT_STATUS_CHARGING  =5011#1006;

	#-- DOCKING AREA-- ################################################
	# Goto Check In Docking Area
	PROCESS_SELFDRIVING_WAIT_GOTO_CHECKINDOCKING = 5012 #500021;
	PROCESS_SELFDRIVING_FINISHED_GOTO_CHECKINDOCKING =5013 #500031;
	
	# Goto Docking Line
	PROCESS_SELFDRIVING_START_GOTO_DOCKINGLINE =5014 #50001;
	PROCESS_SELFDRIVING_WAIT_GOTO_DOCKINGLINE =5015 #50002;
	PROCESS_SELFDRIVING_FINISHED_GOTO_DOCKINGLINE = 5016#50003;
	
	# Goto Docking Line Again when Detect Line Error
	PROCESS_SELFDRIVING_START_REGOTO_DOCKINGLINE = 5017 #50101;
	PROCESS_SELFDRIVING_WAIT_REGOTO_DOCKINGLINE = 5018 #50102;
	PROCESS_SELFDRIVING_FINISHED_REGOTO_DOCKINGLINE =5019 #50103;
	
	# Goto Pallet UP
	PROCESS_SELFDRIVING_START_PALLETUP = 5020#50004;
	PROCESS_SELFDRIVING_ERRORIN_PALLETUP = 5021 #50005;
	PROCESS_SELFDRIVING_WAIT_PALLETUP = 5022#50006;
	PROCESS_SELFDRIVING_FINISH_PALLETUP = 5023 #50007;    // ***
	#-- PUT-AWAY AREA-- ################################################
	
	# Goto Check In PutAway Area
	PROCESS_SELFDRIVING_WAIT_GOTO_CHECKINPUTAWAY = 5024 #5000212;
	PROCESS_SELFDRIVING_FINISHED_GOTO_CHECKINPUTAWAY = 5025 #500129;
	
	# Goto PutAway Line
	PROCESS_SELFDRIVING_START_GOTO_PUTAWAYLINE = 5026 #50008;
	PROCESS_SELFDRIVING_WAIT_GOTO_PUTAWAYLINE = 5027 #50009;
	PROCESS_SELFDRIVING_FINISHED_GOTO_PUTAWAYLINE = 5028 #50010;
	
	# Goto PutAway Line Again when Detect Line Error
	PROCESS_SELFDRIVING_START_REGOTO_PUTAWAYLINE = 5029 #58008;
	PROCESS_SELFDRIVING_WAIT_REGOTO_PUTAWAYLINE = 5030#58009;
	PROCESS_SELFDRIVING_FINISHED_REGOTO_PUTAWAYLINE = 5031#58010;

	# Goto Pallet Down
	PROCESS_SELFDRIVING_START_PALLETDOWN = 5032#50011;
	PROCESS_SELFDRIVING_ERRORIN_PALLETDOWN = 5033#50012;
	PROCESS_SELFDRIVING_WAIT_PALLETDOWN = 5034 #50013;
	PROCESS_SELFDRIVING_FINISH_PALLETDOWN =5035 #50014;  // ***

	#-- CHARGE AREA-- ################################################
	
	PROCESS_SELFDRIVING_BEGIN_GOTOLINE_CHARGEAREA = 5036#500111;
	PROCESS_SELFDRIVING_GOTOLINE_CHARGEAREA =5037#500121;
	PROCESS_SELFDRIVING_WAIT_GOTOLINE_CHARGEAREA =5038#500131;
	PROCESS_SELFDRIVING_FINISH_GOTOLINE_CHARGEAREA  = 5039 #500141;
	
	PROCESS_SELFDRIVING_REGOTOLINE_CHARGEAREA = 5040 #201121;
	PROCESS_SELFDRIVING_WAIT_REGOTOLINE_CHARGEAREA = 5041 #201131;
	PROCESS_SELFDRIVING_FINISH_REGOTOLINE_CHARGEAREA  = 5042 #201141;

	PROCESS_SELFDRIVING_DETECTLINE_TO_CHARGEAREA = 5043 #5001213;
	PROCESS_SELFDRIVING_WAIT_DETECTLINE_TO_CHARGEAREA = 5044 #5001313;
	PROCESS_SELFDRIVING_FINISH_DETECTLINE_TO_CHARGEAREA  =  5045#2001413;#5001413;
	
	PROCESS_SELFDRIVING_FINISH_RETURN_THELINNE = 5046 #3453334;
	PROCESS_SELFDRIVING_WAITING_RETURN_THELINNE = 5047 #3453339;
	PROCESS_SELFDRIVING_ERRORIN_DETECTLINE_TO_CHARGEAREA = 5048 #5001299;
	
	PROCESS_SELFDRIVING_START_BATTERYCHARGED = 5049 #5001211;
	PROCESS_SELFDRIVING_WAIT_BATTERYCHARGING = 5050#5001311;
	PROCESS_SELFDRIVING_FINISH_BATTERYCHARGING  = 5051#5001411;
	PROCESS_SELFDRIVING_CANCEL_BATTERYCHARGING  = 5052 #500654;
	
	#-- READY AREA-- ################################################
	
	PROCESS_SELFDRIVING_BEGIN_GOTOLINE_READYAREA = 5053 #200111;
	PROCESS_SELFDRIVING_GOTOLINE_READYAREA = 5054 #200121;
	PROCESS_SELFDRIVING_WAIT_GOTOLINE_READYAREA = 5055 #200131;
	PROCESS_SELFDRIVING_FINISH_GOTOLINE_READYAREA  = 5056 #200141;
	
	PROCESS_SELFDRIVING_REGOTOLINE_READYAREA = 5057 #290121;
	PROCESS_SELFDRIVING_WAIT_REGOTOLINE_READYAREA = 5058#290131;
	PROCESS_SELFDRIVING_FINISH_REGOTOLINE_READYAREA  = 5059 #290141;

	PROCESS_SELFDRIVING_DETECTLINE_TO_READYAREA = 5070 #2001213;
	PROCESS_SELFDRIVING_WAIT_DETECTLINE_TO_READYAREA = 5071 #2001313;
	PROCESS_SELFDRIVING_FINISH_DETECTLINE_TO_READYAREA  = 5072 #5001413; #2001413;
	
	PROCESS_SELFDRIVING_ERRORIN_DETECTLINE_TO_READYAREA = 5073 #1001299;
	


	PROCESS_SELFDRIVING_START_GOTO_STATION= 5074 #5000811;
	PROCESS_SELFDRIVING_WAIT_GOTO_STATION= 5075#5000911;
	PROCESS_SELFDRIVING_FINISHED_GOTO_STATION= 5076 #5001011;
	
	PROCESS_SELFDRIVING_REQUEST_CHARGEBATTERY= 5078 #5001011;

	
class SelfDriving():
	def __init__(self):
    #####################################################
		rospy.loginfo("Initializing SelfDriving Class...")
        	rospy.init_node("SelfDriving")
        	self.nodename = rospy.get_name()
        	rospy.loginfo("%s started" % self.nodename)
		self.rate=rospy.get_param('~rate',50);
		self.robotOrdered=RobotOrder();
		self.robotAtReadyArea=RobotAtReadyArea();
		self.robotAtChargeArea=RobotAtChargeArea();
		self.batteryInfo=BatteryInfo();
		self.amclpose_posX=0;
		self.amclpose_posY=0;
		self.amclpose_posthetaW=0.0;
		self.amclpose_posthetaZ=0.0;
		self.currentgoal_x=0.0;
		self.currentgoal_y=0.0;
		self.currentgoal_z=0.0;
		self.currentgoal_w=0.0;
		self.current_Vx=0.0;
		self.current_Vy=0.0;
		self.current_W=0.0;
		self.errorVx=0.0;
		self.errorVy=0.0;
		self.errorW=0.0;
		self.errorDx=0.0;
		self.errorDy=0.0;
		self.countpos=0;
		self.cntErrorDetectLine=0;
		self.cntTimeOutRequest=0;
		self.amoutOfErrorInLineDetection=3; # 3 times errors
		self.amoutOfRefreshLineDetection=10; # 3 times errors
		self.flag_state_reachedgoal_palletup=False;
		self.flag_state_reachedgoal_palletdown=False;
		self.flag_state_reachedgoal_battery=False;
		self.flag_state_reachedgoal_checkindocking=False;
		self.flag_state_reachedgoal_checkinputaway=False;
		self.flag_state_reachedgoal_chargearea=False;
		self.flag_state_reachedgoal_readyarea=False;
		self.flag_state_error_detectline=False; # dam bao request robot request vi tri pallet 1 lan
		self.processSelfDrivingRobot=StatesSelfDriving.PROCESS_SELFDRIVING_IDLE;
		
		self.pub=rospy.Publisher('chatter', String, queue_size=10)
		self.pub_navigation_setgoal=rospy.Publisher('/move_base_simple/goal',PoseStamped,queue_size=10);
		self.pub_posPallet=rospy.Publisher('pospallet',Int32,queue_size=10);
		self.pub_linedetectionctrl=rospy.Publisher('linedetectionctrl',Int32,queue_size=10);
		self.pub_errorturningrobot=rospy.Publisher('errorturningrobot',Int32,queue_size=10);
		self.pub_BatteryChargeResponseStatusSelfDriving=rospy.Publisher('batteryselfdrivingcallback',Int32,queue_size=10);
		self.pub_IpChargeBattery=rospy.Publisher('chargeIP',String, queue_size=10);
		self.pub_RobotStatus=rospy.Publisher('robotStatus',Int32,queue_size=10);
		self.pub_requestReadyAreaPos=rospy.Publisher('requestReadyAreaPos',Int32,queue_size=10);
		self.pub_FinishedStates=rospy.Publisher('finishedStates', Int32, queue_size=10);
		
		rospy.Subscriber('responsedStandPosInsideReadyArea',String,self.responsedStandPosInsideReadyArea_callback,queue_size=100);
		rospy.Subscriber('battery_vol',Float32,self.batterysub_callback,queue_size=100);
		rospy.Subscriber('errorDetectedLine',Int32, self.errorDetectedLine_callback);
		rospy.Subscriber('linedetectioncallback',Int32,self.LineDetection_callback,queue_size=1);
		rospy.Subscriber('amcl_pose',PoseWithCovarianceStamped,self.navigationAmclPose_callback,queue_size=100);
		rospy.Subscriber('odom',Odometry,self.odometry_callback,queue_size=100);
		rospy.Subscriber('move_base/status',GoalStatusArray,self.reachedGoal_Callback,queue_size=100);
	
		rospy.Subscriber('serverRobotGotToLineDockingArea',String,self.serverRobotGotToLineDockingAreaCallBack,queue_size=100);
		rospy.Subscriber("serverRobotGotToPalletDockingArea",String,self.serverRobotGotToPalletDockingAreaCallBack,queue_size=100);

	
		rospy.Subscriber("serverRobotGotToLinePutAwayArea",String,self.serverRobotGotToLinePutAwayAreaCallBack,queue_size=100);
		rospy.Subscriber("serverRobotGotToPalletPutAwayArea",String,self.serverRobotGotToPalletPutAwayAreaCallBack,queue_size=100);

		rospy.Subscriber("serverRobotGotToCheckInPutAwayArea",String,self.serverRobotGotToCheckInPutAwayAreaCallBack,queue_size=100);
		rospy.Subscriber("serverRobotGotToCheckInDockingArea",String,self.serverRobotGotToCheckInDockingAreaCallBack,queue_size=100);
			
		rospy.Subscriber("serverRobotGotToFrontReadyArea",String,self.serverRobotGotToFrontReadyAreaCallBack,queue_size=100);
		rospy.Subscriber("serverRobotGotToChargeArea",String,self.serverRobotGotToChargeAreaCallBack,queue_size=100);
		rospy.Subscriber("serverRequestRobotDetectLine",String,self.serverRequestRobotDetectLineCallBack,queue_size=100);
		#rospy.Subscriber("serverResetErrorDetectLine",String,self.serverResetErrorDetectLineCallBack);
	

	def spin(self):
	        self.r = rospy.Rate(self.rate)
		self.setparam();

		while not rospy.is_shutdown():
			#self.processSelfDrivingRobot=StatesSelfDriving.PROCESS_SELFDRIVING_FINISH_PALLETUP
			self.process();
			self.r.sleep()
	def counterTimeOutRequest(self,second):
		if self.cntTimeOutRequest>=second:
			self.cntTimeOutRequest=0;
			flagSetTimeOut=True;
		else:
			self.cntTimeOutRequest+=1;
			flagSetTimeOut=False;
			sleep(1);
		return flagSetTimeOut
	def setparam(self):
		#document = yaml.safe_load(open('/home/phat/default.yaml'))
		document = yaml.safe_load(open('/home/seldat/seldat_robot/src/seldat_robot/src/default.yaml'))
		# Robot At Ready
		self.robotAtReadyArea.readyArea.X=document["Ready Area"]["Line Location"]["X"]
		self.robotAtReadyArea.readyArea.Y=document["Ready Area"]["Line Location"]["Y"]
		self.robotAtReadyArea.readyArea.Angle=document["Ready Area"]["Line Location"]["Angle"]
		self.robotAtReadyArea.limitedReadyArea.Xmax=document["Ready Area"]["Position In Line"]["Max"]
		self.robotAtReadyArea.limitedReadyArea.Xmin=document["Ready Area"]["Position In Line"]["Min"]
		
		self.robotAtChargeArea.chargeArea.X=document["Charge Area"]["Line Location"]["X"]
		self.robotAtChargeArea.chargeArea.Y=document["Charge Area"]["Line Location"]["Y"]
		self.robotAtChargeArea.chargeArea.Angle=document["Charge Area"]["Line Location"]["Angle"]
		self.robotAtChargeArea.limitedChargeArea.Xmax=document["Charge Area"]["Position In Line"]["Max"]
		self.robotAtChargeArea.limitedChargeArea.Xmin=document["Charge Area"]["Position In Line"]["Min"]
		self.robotAtChargeArea.url=document["Charge Area"]["Url"]
		
		self.batteryInfo.defautLowBattery=document["Battery"]["Charged Level"]
		self.errorVx=document["Robot Determined Goal Position"]["Error Vx"];
		self.errorVy=document["Robot Determined Goal Position"]["Error Vy"];
		self.errorW=document["Robot Determined Goal Position"]["Error W"];
		self.errorDx=document["Robot Determined Goal Position"]["Error Dx"];
		self.errorDy=document["Robot Determined Goal Position"]["Error Dy"];
		
		self.amoutOfErrorInLineDetection=document["LineDetection"]["Amout Of Error In LineDetection"];
		self.amoutOfRefreshLineDetection=document["LineDetection"]["Amout OF Refresh"];
		
	def navigationAmclPose_callback(self,msg):
		self.amclpose_posX=msg.pose.pose.position.x;
		self.amclpose_posY=msg.pose.pose.position.y;
		self.amclpose_posthetaW=msg.pose.pose.orientation.w;
		self.amclpose_posthetaZ=msg.pose.pose.orientation.z;
	def odometry_callback(self,odom):
		self.current_Vx=odom.twist.twist.linear.x;
		self.current_Vy =odom.twist.twist.linear.y;
		self.current_W  =odom.twist.twist.angular.z;
	def serverRobotGotToLineDockingAreaCallBack(self,msg):
		document=json.loads(msg.data);
		self.robotOrdered.posDocking.X=document["docking"]["line"]["X"]
		self.robotOrdered.posDocking.Y=document["docking"]["line"]["Y"]
		self.robotOrdered.posDocking.Angle=document["docking"]["line"]["Angle"]
		self.processSelfDrivingRobot = StatesSelfDriving.PROCESS_SELFDRIVING_START_GOTO_DOCKINGLINE;
		rospy.loginfo("PROCESS_SELFDRIVING_START_GOTO_DOCKINGLINE");
	def serverRobotGotToPalletDockingAreaCallBack(self,msg):
		document=json.loads(msg.data);
		self.robotOrdered.limitedPosDocking.Xmax=document["docking"]["pallet"]["Xmax"]
		self.robotOrdered.limitedPosDocking.Xmin=document["docking"]["pallet"]["Xmin"]
		self.processSelfDrivingRobot = StatesSelfDriving.PROCESS_SELFDRIVING_START_PALLETUP;
		rospy.loginfo("PROCESS_SELFDRIVING_START_PALLETUP");
	def serverRobotGotToLinePutAwayAreaCallBack(self,msg):
		document=json.loads(msg.data);
		self.robotOrdered.posPutAway.X=document["putaway"]["line"]["X"]
		self.robotOrdered.posPutAway.Y=document["putaway"]["line"]["Y"]
		self.robotOrdered.posPutAway.Angle=document["putaway"]["line"]["Angle"]
		self.processSelfDrivingRobot = StatesSelfDriving.PROCESS_SELFDRIVING_START_GOTO_PUTAWAYLINE;
		rospy.loginfo("PROCESS_SELFDRIVING_START_GOTO_PUTAWAYLINE");
	def serverRobotGotToPalletPutAwayAreaCallBack(self,msg):
		document=json.loads(msg.data);
		self.robotOrdered.limitedPosPutAway.Xmax=document["putaway"]["pallet"]["Xmax"]
		self.robotOrdered.limitedPosPutAway.Xmin=document["putaway"]["pallet"]["Xmin"]
		self.processSelfDrivingRobot =StatesSelfDriving.PROCESS_SELFDRIVING_START_PALLETDOWN;
		rospy.loginfo("PROCESS_SELFDRIVING_START_PALLETDOWN");
	def serverRobotGotToFrontReadyAreaCallBack(self,msg):
		document=json.loads(msg.data);
		#self.robotAtReadyArea.readyAreaArea.X=document["ready"]["X"]
		#self.robotAtReadyArea.readyArea.Y=document["ready"]["Y"]
		#self.robotAtReadyArea.readyArea.Angle=document["ready"]["Angle"]
		self.processSelfDrivingRobot = StatesSelfDriving.PROCESS_SELFDRIVING_BEGIN_GOTOLINE_READYAREA;
		rospy.loginfo("PROCESS_SELFDRIVING_BEGIN_GOTOLINE_READYARE");
	def serverRobotGotToChargeAreaCallBack(self,msg):
		document=json.loads(msg.data);
		#self.robotAtReadyArea.readyAreaArea.X=document["ready"]["X"]
		#self.robotAtReadyArea.readyArea.Y=document["ready"]["Y"]
		#self.robotAtReadyArea.readyArea.Angle=document["ready"]["Angle"]
		self.processSelfDrivingRobot = StatesSelfDriving.PROCESS_SELFDRIVING_BEGIN_GOTOLINE_CHARGEAREA;
		rospy.loginfo("PROCESS_SELFDRIVING_BEGIN_GOTOLINE_CHARGEAREA");		
	def serverRobotGotToCheckInDockingAreaCallBack(self,msg):
		document=json.loads(msg.data);
		self.robotOrdered.posCheckInDocking.X=document["docking"]["checkin"]["X"]
		self.robotOrdered.posCheckInDocking.Y=document["docking"]["checkin"]["Y"]
		self.robotOrdered.posCheckInDocking.Angle=document["docking"]["checkin"]["Angle"]
		self.moveBaseSimple_goal(self.robotOrdered.posCheckInDocking.X,self.robotOrdered.posCheckInDocking.Y,self.robotOrdered.posCheckInDocking.Angle);
		self.currentgoal_x=self.robotOrdered.posCheckInDocking.X;
		self.currentgoal_y=self.robotOrdered.posCheckInDocking.Y;
		self.processSelfDrivingRobot =StatesSelfDriving.PROCESS_SELFDRIVING_WAIT_GOTO_CHECKINDOCKING;
		rospy.loginfo("PROCESS_SELFDRIVING_WAIT_GOTO_CHECKINDOCKING");
	def serverRobotGotToCheckInPutAwayAreaCallBack(self,msg):
		document=json.loads(msg.data);
		self.robotOrdered.posCheckInPutAway.X=document["putaway"]["checkin"]["X"]
		self.robotOrdered.posCheckInPutAway.Y=document["putaway"]["checkin"]["Y"]
		self.robotOrdered.posCheckInPutAway.Angle=document["putaway"]["checkin"]["Angle"]
		self.moveBaseSimple_goal(self.robotOrdered.posCheckInPutAway.X,self.robotOrdered.posCheckInPutAway.Y,self.robotOrdered.posCheckInPutAway.Angle);
		self.currentgoal_x=self.robotOrdered.posCheckInPutAway.X;
		self.currentgoal_y=self.robotOrdered.posCheckInPutAway.Y;
		self.processSelfDrivingRobot =StatesSelfDriving.PROCESS_SELFDRIVING_WAIT_GOTO_CHECKINPUTAWAY;
		rospy.loginfo("PROCESS_SELFDRIVING_WAIT_GOTO_CHECKINPUTAWAY");
	def responsedStandPosInsideReadyArea_callback(self,msg):	
		#document=json.loads(msg.data);
		#self.robotAtReadyArea.limitedReadyArea.Xmax=document["ready"]["Xmax"].GetDouble();
		#self.robotAtReadyArea.limitedReadyArea.Xmin=document["ready"]["Xmin"].GetDouble();
		#self.robotAtReadyArea.limitedReadyArea.iPCharger=document["ready"]["IP"].GetString();
		self.processSelfDrivingRobot = StatesSelfDriving.PROCESS_SELFDRIVING_DETECTLINE_TO_READYAREA;
		rospy.loginfo("PROCESS_SELFDRIVING_DETECTLINE_TO_CHARGEAREA");
	def batterysub_callback (self,msg):
		self.batteryInfo.currentBatteryLevel=msg.data;
		if self.batteryInfo.currentBatteryLevel<=self.batteryInfo.defautLowBattery:
			if self.batteryInfoflagNeedToCharge==False:
				self.batteryInfoflagNeedToCharge=True;
				self.pubFinishedStates(StatesSelfDriving.PROCESS_SELFDRIVING_REQUEST_CHARGEBATTERY);
	def posPalletCtrl(self,cmd):
		rospy.loginfo(cmd)
		numctrl=cmd.value
		self.pub_posPallet.publish(numctrl);
	def linedetectionctrl(self,cmd):
		rospy.loginfo(cmd)
		numctrl=cmd.value
		self.pub_linedetectionctrl.publish(numctrl);
	def pubFinishedStates(self,state):
		rospy.loginfo(state)
		num=state.value
		self.pub_FinishedStates.publish(num);
	def pubChargeIP(self,ip):
		rospy.loginfo(ip);
		self.pub_IpChargeBattery.publish(ip);
	def moveBaseSimple_goal(self,posx,posy,angle):
		pose=PoseStamped();
		pose.header.frame_id = "map";
		pose.pose.position.x=posx;
		pose.pose.position.y=posy;
		pose.pose.position.z=0;
		th=angle*math.pi/180;
		pose.pose.orientation.z=math.sin(th/2);
		pose.pose.orientation.w=math.cos(th/2);
		rospy.loginfo(pose);
		self.pub_navigation_setgoal.publish(pose);
	def LineDetection_callback(self,msg):
		rospy.loginfo (ResponseCommandLineDetect.RESPONSE_LINEDETECT_PALLETUP);
		if msg.data==ResponseCommandLineDetect.RESPONSE_LINEDETECT_PALLETUP.value:
			self.processSelfDrivingRobot=StatesSelfDriving.PROCESS_SELFDRIVING_FINISH_PALLETUP;
			rospy.loginfo("PROCESS_SELFDRIVING_FINISH_PALLETUP");
		elif msg.data==ResponseCommandLineDetect.RESPONSE_LINEDETECT_PALLETDOWN.value:
			self.processSelfDrivingRobot=StatesSelfDriving.PROCESS_SELFDRIVING_FINISH_PALLETDOWN;
			rospy.loginfo("PROCESS_SELFDRIVING_FINISH_PALLETDOWN");
		elif msg.data==ResponseCommandLineDetect.RESPONSE_FINISH_DETECTLINE_CHARGEAREA.value:
			self.processSelfDrivingRobot=StatesSelfDriving.PROCESS_SELFDRIVING_FINISH_DETECTLINE_TO_CHARGEAREA;
			rospy.loginfo("PROCESS_SELFDRIVING_FINISH_DETECTLINE_TO_CHARGEAREA")
		elif msg.data==ResponseCommandLineDetect.RESPONSE_FINISH_RETURN_LINE_CHARGEAREA.value:
			self.processSelfDrivingRobot=StatesSelfDriving.PROCESS_SELFDRIVING_FINISH_RETURN_THELINNE;
			rospy.loginfo("PROCESS_SELFDRIVING_FINISH_RETURN_THELINNE")
	def serverRequestRobotDetectLineCallBack(self,msg):
		rospy.loginfo("serverRequestRobotDetectLineCallBack");
		self.cntErrorDetectLine=0;
		if self.processSelfDrivingRobot==StatesSelfDriving.PROCESS_SELFDRIVING_WAIT_PALLETUP:
			self.processSelfDrivingRobot=StatesSelfDriving.PROCESS_SELFDRIVING_START_PALLETUP;
			rospy.loginfo("PROCESS_SELFDRIVING_RESET_PALLETUP");
		elif self.processSelfDrivingRobot==StatesSelfDriving.PROCESS_SELFDRIVING_WAIT_PALLETDOWN:
			self.processSelfDrivingRobot=StatesSelfDriving.PROCESS_SELFDRIVING_START_PALLETDOWN;
			rospy.loginfo("PROCESS_SELFDRIVING_RESET_PALLETDOWN");
		elif self.processSelfDrivingRobot==StatesSelfDriving.PROCESS_SELFDRIVING_WAIT_DETECTLINE_TO_CHARGEAREA:
			self.processSelfDrivingRobot=StatesSelfDriving.PROCESS_SELFDRIVING_DETECTLINE_TO_CHARGEAREA
			rospy.loginfo("PROCESS_SELFDRIVING_RESET_DETECTLINE_TO_CHARGEAREA");
		elif self.processSelfDrivingRobot==StatesSelfDriving.PROCESS_SELFDRIVING_WAIT_BATTERYCHARGING: # error tai tram sac, bat dau lai line
			self.processSelfDrivingRobot=StatesSelfDriving.PROCESS_SELFDRIVING_DETECTLINE_TO_CHARGEAREA
			rospy.loginfo("PROCESS_SELFDRIVING_RESET_DETECTLINE_TO_CHARGEAREA");
		elif self.processSelfDrivingRobot==StatesSelfDriving.PROCESS_SELFDRIVING_WAIT_DETECTLINE_TO_READYAREA:
			self.processSelfDrivingRobot=StatesSelfDriving.PROCESS_SELFDRIVING_DETECTLINE_TO_READYAREA
			rospy.loginfo("PROCESS_SELFDRIVING_RESET_DETECTLINE_TO_READYAREA");
	def errorDetectedLine_callback(self,msg):
		if msg.data==CallErrorLineDetect.CallERROR_LINEDETECTION.value: # error line detected
			if self.cntErrorDetectLine<self.amoutOfErrorInLineDetection:
				self.cntErrorDetectLine=self.cntErrorDetectLine+1;
				if self.processSelfDrivingRobot==StatesSelfDriving.PROCESS_SELFDRIVING_WAIT_PALLETUP:
					self.processSelfDrivingRobot=StatesSelfDriving.PROCESS_SELFDRIVING_ERRORIN_PALLETUP;
					rospy.loginfo("PROCESS_SELFDRIVING_ERRORIN_PALLETUP");
				elif self.processSelfDrivingRobot==StatesSelfDriving.PROCESS_SELFDRIVING_WAIT_PALLETDOWN:
					self.processSelfDrivingRobot=StatesSelfDriving.PROCESS_SELFDRIVING_ERRORIN_PALLETDOWN;
					rospy.loginfo("PROCESS_SELFDRIVING_ERRORIN_PALLETDOWN");
				elif self.processSelfDrivingRobot==StatesSelfDriving.PROCESS_SELFDRIVING_WAIT_DETECTLINE_TO_CHARGEAREA:
					self.processSelfDrivingRobot=StatesSelfDriving.PROCESS_SELFDRIVING_ERRORIN_DETECTLINE_TO_CHARGEAREA;
					rospy.loginfo("PROCESS_SELFDRIVING_ERRORIN_DETECTLINE_TO_CHARGEAREA");
				elif self.processSelfDrivingRobot==StatesSelfDriving.PROCESS_SELFDRIVING_WAIT_BATTERYCHARGING: # error tai tram sac, bat dau lai line
					self.processSelfDrivingRobot=StatesSelfDriving.PROCESS_SELFDRIVING_ERRORIN_DETECTLINE_TO_CHARGEAREA;
					rospy.loginfo("PROCESS_SELFDRIVING_ERRORIN_DETECTLINE_TO_CHARGEAREA");
				elif self.processSelfDrivingRobot==StatesSelfDriving.PROCESS_SELFDRIVING_WAIT_DETECTLINE_TO_READYAREA:
					self.processSelfDrivingRobot=StatesSelfDriving.PROCESS_SELFDRIVING_ERRORIN_DETECTLINE_TO_READYAREA;
					rospy.loginfo("PROCESS_SELFDRIVING_ERRORIN_DETECTLINE_TO_READYAREA");

	def reachedGoal_Callback(self,msg):
		if len(msg.status_list):
			if msg.status_list[0].status==msg.status_list[0].SUCCEEDED:
				_currentgoal_Ex = math.fabs(math.fabs(self.amclpose_posX)-math.fabs(self.currentgoal_x));
				_currentgoal_Ey = math.fabs(math.fabs(self.amclpose_posY)-math.fabs(self.currentgoal_y));
				_currentgoal_Ez = math.fabs(math.fabs(self.amclpose_posthetaZ)-math.fabs(self.currentgoal_z));
				_currentgoal_Ew = math.fabs(math.fabs(self.amclpose_posthetaW)-math.fabs(self.currentgoal_w));
				if math.fabs(self.current_Vx)<self.errorVx and math.fabs(self.current_Vy)<self.errorVy and math.fabs(self.current_W)<self.errorW and _currentgoal_Ex<=self.errorDx and _currentgoal_Ey<=self.errorDy:
					#if self.flag_state_reachedgoal_palletup == False :
					if self.processSelfDrivingRobot ==StatesSelfDriving.PROCESS_SELFDRIVING_WAIT_GOTO_DOCKINGLINE:
						rospy.loginfo("COME_DOCKINGLINE");
						self.processSelfDrivingRobot =StatesSelfDriving.PROCESS_SELFDRIVING_FINISHED_GOTO_DOCKINGLINE;
						#self.flag_state_reachedgoal_palletup=True;
					#if self.flag_state_reachedgoal_palletdown==False:
					if self.processSelfDrivingRobot==StatesSelfDriving.PROCESS_SELFDRIVING_WAIT_REGOTO_DOCKINGLINE:
						rospy.loginfo("COME_RE_DOCKINGLINE");
						self.processSelfDrivingRobot =StatesSelfDriving.PROCESS_SELFDRIVING_FINISHED_REGOTO_DOCKINGLINE;
						
					if self.processSelfDrivingRobot ==StatesSelfDriving.PROCESS_SELFDRIVING_WAIT_GOTO_PUTAWAYLINE:
						self.processSelfDrivingRobot =StatesSelfDriving.PROCESS_SELFDRIVING_FINISHED_GOTO_PUTAWAYLINE;
						rospy.loginfo("COME_PUTAWAYLINE");
					if self.processSelfDrivingRobot==StatesSelfDriving.PROCESS_SELFDRIVING_WAIT_REGOTO_PUTAWAYLINE:
						rospy.loginfo("COME_RE_PUTAWAYLINE");
						self.processSelfDrivingRobot =StatesSelfDriving.PROCESS_SELFDRIVING_FINISHED_REGOTO_PUTAWAYLINE;
						#self.flag_state_reachedgoal_palletdown=True;
					#if self.flag_state_reachedgoal_checkindocking == False:
					if self.processSelfDrivingRobot == StatesSelfDriving.PROCESS_SELFDRIVING_WAIT_GOTO_CHECKINDOCKING:
						self.processSelfDrivingRobot = StatesSelfDriving.PROCESS_SELFDRIVING_FINISHED_GOTO_CHECKINDOCKING;
						rospy.loginfo("COME_GOTO_CHECKINDOCKING");
						#self.flag_state_reachedgoal_checkindocking=True;
					#if self.flag_state_reachedgoal_checkinputaway == False:
					if self.processSelfDrivingRobot ==StatesSelfDriving.PROCESS_SELFDRIVING_WAIT_GOTO_CHECKINPUTAWAY:
						self.processSelfDrivingRobot =StatesSelfDriving.PROCESS_SELFDRIVING_FINISHED_GOTO_CHECKINPUTAWAY;
						rospy.loginfo("COME_CHECKINPUTAWAY");
						#self.flag_state_reachedgoal_checkinputaway=True;
					#if self.flag_state_reachedgoal_chargearea==False:
					if self.processSelfDrivingRobot == StatesSelfDriving.PROCESS_SELFDRIVING_WAIT_GOTOLINE_CHARGEAREA:
						self.processSelfDrivingRobot =StatesSelfDriving.PROCESS_SELFDRIVING_FINISH_GOTOLINE_CHARGEAREA;						rospy.loginfo("COME_GOTO_CHARGEAREA");
						#self.flag_state_reachedgoal_chargearea=True;
					#if self.flag_state_reachedgoal_readyarea==False:
					if self.processSelfDrivingRobot == StatesSelfDriving.PROCESS_SELFDRIVING_WAIT_GOTOLINE_READYAREA:
						self.processSelfDrivingRobot = StatesSelfDriving.PROCESS_SELFDRIVING_FINISH_GOTOLINE_READYAREA;
						rospy.loginfo("COME_GOTO_READYAREA");
						#self.flag_state_reachedgoal_readyarea=True;
					if self.processSelfDrivingRobot == StatesSelfDriving.PROCESS_SELFDRIVING_WAIT_REGOTOLINE_CHARGEAREA:
						self.processSelfDrivingRobot =StatesSelfDriving.PROCESS_SELFDRIVING_FINISH_REGOTOLINE_CHARGEAREA;
						rospy.loginfo("COME_REGOTO_CHARGEAREA");
						#self.flag_state_reachedgoal_chargearea=True;
					#if self.flag_state_reachedgoal_readyarea==False:
					if self.processSelfDrivingRobot == StatesSelfDriving.PROCESS_SELFDRIVING_WAIT_REGOTOLINE_READYAREA:
						self.processSelfDrivingRobot = StatesSelfDriving.PROCESS_SELFDRIVING_FINISH_REGOTOLINE_READYAREA;
						rospy.loginfo("COME_REGOTO_READYAREA");
						#self.flag_state_reachedgoal_readyarea=True;
						
	def process(self):
		if self.processSelfDrivingRobot==StatesSelfDriving.PROCESS_SELFDRIVING_STOP:
			rospy.loginfo("PROCESS_SELFDRIVING_STOP");
		elif self.processSelfDrivingRobot==StatesSelfDriving.PROCESS_SELFDRIVING_RESET:
			rospy.loginfo("PROCESS_SELFDRIVING_RESET");
        	elif self.processSelfDrivingRobot==StatesSelfDriving.PROCESS_SELFDRIVING_PAUSE:
			rospy.loginfo("PROCESS_SELDRIVING_PAUSE");
        	elif self.processSelfDrivingRobot==StatesSelfDriving.PROCESS_SELFDRIVING_START_READY:
			if self.batteryInfo.currentBatteryLevel<=self.batteryInfo.defautLowBattery:
				self.processSelfDrivingRobot =StatesSelfDriving.PROCESS_SELFDRIVING_START_BATTERYCHARGED;
		#######################################################################		
		#-- DOCKING AREA-- ################################################	

		# CHECK IN DOCKING
		elif self.processSelfDrivingRobot==StatesSelfDriving.PROCESS_SELFDRIVING_WAIT_GOTO_CHECKINDOCKING:
			if self.flag_state_reachedgoal_checkindocking:
				self.processSelfDrivingRobot =StatesSelfDriving.PROCESS_SELFDRIVING_FINISHED_GOTO_CHECKINDOCKING;
				self.flag_state_reachedgoal_checkindocking=False;
		elif self.processSelfDrivingRobot==StatesSelfDriving.PROCESS_SELFDRIVING_FINISHED_GOTO_CHECKINDOCKING:
			self.processSelfDrivingRobot =StatesSelfDriving.PROCESS_SELFDRIVING_IDLE;
			self.pubFinishedStates(StatesSelfDriving.PROCESS_SELFDRIVING_FINISHED_GOTO_CHECKINDOCKING);
		# FINISH CHECK IN DOCKING******************************************************************
		
		
		# GO TO LINE DOCKING		
		elif self.processSelfDrivingRobot==StatesSelfDriving.PROCESS_SELFDRIVING_START_GOTO_DOCKINGLINE:
            		self.currentgoal_x=self.robotOrdered.posDocking.X;
            		self.currentgoal_y=self.robotOrdered.posDocking.Y;
					#move base set goal
			self.moveBaseSimple_goal(self.robotOrdered.posDocking.X,self.robotOrdered.posDocking.Y,self.robotOrdered.posDocking.Angle);
            		self.processSelfDrivingRobot = StatesSelfDriving.PROCESS_SELFDRIVING_WAIT_GOTO_DOCKINGLINE;
            		self.flag_state_reachedgoal_palletup=False;
			rospy.loginfo("PROCESS_SELFDRIVING_WAIT_GOTO_DOCKINGLINE");
			
        	elif self.processSelfDrivingRobot==StatesSelfDriving.PROCESS_SELFDRIVING_WAIT_GOTO_DOCKINGLINE:
			if self.flag_state_reachedgoal_palletup==True:
				self.processSelfDrivingRobot = StatesSelfDriving.PROCESS_SELFDRIVING_FINISHED_GOTO_DOCKINGLINE;
				self.flag_state_reachedgoal_palletup=False;
				rospy.loginfo("PROCESS_SELFDRIVING_FINISHED_GOTO_DOCKINGLINE");
        	elif self.processSelfDrivingRobot==StatesSelfDriving.PROCESS_SELFDRIVING_FINISHED_GOTO_DOCKINGLINE:
			self.cntErrorDetectLine=0;
			self.pubFinishedStates(StatesSelfDriving.PROCESS_SELFDRIVING_FINISHED_GOTO_DOCKINGLINE);
			self.processSelfDrivingRobot =StatesSelfDriving.PROCESS_SELFDRIVING_IDLE;
			for x in range(0,self.amoutOfRefreshLineDetection):
				self.posPalletCtrl(RequestCommandLineDetect.REQUEST_LINEDETECT_REFRESH);
		# FINISH GO TO LINE DOCKING *****************************************************************		
				
		# REGOTO LINE DOCKING AREA
		elif self.processSelfDrivingRobot==StatesSelfDriving.PROCESS_SELFDRIVING_START_REGOTO_DOCKINGLINE:
            		self.currentgoal_x=self.robotOrdered.posDocking.X;
            		self.currentgoal_y=self.robotOrdered.posDocking.Y;
					#move base set goal
			self.moveBaseSimple_goal(self.robotOrdered.posDocking.X,self.robotOrdered.posDocking.Y,self.robotOrdered.posDocking.Angle);
            		self.processSelfDrivingRobot = StatesSelfDriving.PROCESS_SELFDRIVING_WAIT_REGOTO_DOCKINGLINE;
            		self.flag_state_reachedgoal_palletup=False;
			rospy.loginfo("PROCESS_SELFDRIVING_WAIT_REGOTO_DOCKINGLINE");
			
        	elif self.processSelfDrivingRobot==StatesSelfDriving.PROCESS_SELFDRIVING_WAIT_REGOTO_DOCKINGLINE:
			if self.flag_state_reachedgoal_palletup==True:
				#self.processSelfDrivingRobot = StatesSelfDriving.PROCESS_SELFDRIVING_FINISHED_REGOTO_DOCKINGLINE;
				self.flag_state_reachedgoal_palletup=False;
				rospy.loginfo("PROCESS_SELFDRIVING_FINISHED_REGOTO_DOCKINGLINE");
        	elif self.processSelfDrivingRobot== StatesSelfDriving.PROCESS_SELFDRIVING_FINISHED_REGOTO_DOCKINGLINE:
				self.processSelfDrivingRobot=StatesSelfDriving.PROCESS_SELFDRIVING_START_PALLETUP;
				
		# ERRORIN_PALLETUP ########################################################		
		elif self.processSelfDrivingRobot==StatesSelfDriving.PROCESS_SELFDRIVING_ERRORIN_PALLETUP:
			self.processSelfDrivingRobot=StatesSelfDriving.PROCESS_SELFDRIVING_START_REGOTO_DOCKINGLINE;
			self.flag_state_error_detectline=True;
		
		# START DETECT PALLET UP
       		elif self.processSelfDrivingRobot==StatesSelfDriving.PROCESS_SELFDRIVING_START_PALLETUP:  
			self.linedetectionctrl(RequestCommandLineDetect.REQUEST_LINEDETECT_PALLETUP);
			self.flag_state_error_detectline=False;
                    	self.processSelfDrivingRobot =StatesSelfDriving.PROCESS_SELFDRIVING_WAIT_PALLETUP;
			self.countpos=0;
       		elif self.processSelfDrivingRobot==StatesSelfDriving.PROCESS_SELFDRIVING_WAIT_PALLETUP:
			self.pubFinishedStates(StatesSelfDriving.PROCESS_SELFDRIVING_WAIT_PALLETUP);
			if self.amclpose_posX<=self.robotOrdered.limitedPosDocking.Xmax:
				self.posPalletCtrl(RequestCommandLineDetect.REQUEST_LINEDETECT_REACHING_POSITION);
       		elif self.processSelfDrivingRobot==StatesSelfDriving.PROCESS_SELFDRIVING_FINISH_PALLETUP:
			if self.counterTimeOutRequest(2)==True:
				self.pubFinishedStates(StatesSelfDriving.PROCESS_SELFDRIVING_FINISH_PALLETUP);
				self.cntErrorDetectLine=0;
				for x in range(0,self.amoutOfRefreshLineDetection):
					self.posPalletCtrl(RequestCommandLineDetect.REQUEST_LINEDETECT_REFRESH);
				#self.processSelfDrivingRobot =StatesSelfDriving.PROCESS_SELFDRIVING_IDLE;
		# FINISH DETECT PALLET UP *****************************************************************

		
		#############################################################################
		#-- PUT-AWAY AREA-- #########################################################

			
		# CHECK-IN PUT AWAY				
		elif self.processSelfDrivingRobot==StatesSelfDriving.PROCESS_SELFDRIVING_WAIT_GOTO_CHECKINPUTAWAY:
			if self.flag_state_reachedgoal_checkinputaway==True:
				StatesSelfDriving.processSelfDrivingRobot=StatesSelfDriving.PROCESS_SELFDRIVING_FINISHED_GOTO_CHECKINPUTAWAY;
		elif self.processSelfDrivingRobot==StatesSelfDriving.PROCESS_SELFDRIVING_FINISHED_GOTO_CHECKINPUTAWAY:
			rospy.loginfo("Finish check in putaway");
			self.processSelfDrivingRobot =StatesSelfDriving.PROCESS_SELFDRIVING_IDLE;
			self.pubFinishedStates(StatesSelfDriving.PROCESS_SELFDRIVING_FINISHED_GOTO_CHECKINPUTAWAY);
		# FINISH CHECK-IN PUTWAY*******************************************************************

		# START GO TO LINE PUT-AWAY
        	elif self.processSelfDrivingRobot==StatesSelfDriving.PROCESS_SELFDRIVING_START_GOTO_PUTAWAYLINE:
            		self.currentgoal_x=	self.robotOrdered.posPutAway.X;
            		self.currentgoal_y=	self.robotOrdered.posPutAway.Y;
						#move base set goal
			self.moveBaseSimple_goal(self.robotOrdered.posPutAway.X,self.robotOrdered.posPutAway.Y,self.robotOrdered.posPutAway.Angle);
            		self.processSelfDrivingRobot = StatesSelfDriving.PROCESS_SELFDRIVING_WAIT_GOTO_PUTAWAYLINE;
            		self.flag_state_reachedgoal_palletdown=False;
        	elif self.processSelfDrivingRobot==StatesSelfDriving.PROCESS_SELFDRIVING_WAIT_GOTO_PUTAWAYLINE:
            		if self.flag_state_reachedgoal_palletdown==True:
                		self.processSelfDrivingRobot = StatesSelfDriving.PROCESS_SELFDRIVING_FINISHED_GOTO_PUTAWAYLINE;
                		self.flag_state_reachedgoal_palletdown=False;
        	elif self.processSelfDrivingRobot==StatesSelfDriving.PROCESS_SELFDRIVING_FINISHED_GOTO_PUTAWAYLINE:
			self.cntErrorDetectLine=0;
			self.pubFinishedStates(StatesSelfDriving.PROCESS_SELFDRIVING_FINISHED_GOTO_PUTAWAYLINE);
			for x in range(0,self.amoutOfRefreshLineDetection):
				self.posPalletCtrl(RequestCommandLineDetect.REQUEST_LINEDETECT_REFRESH);
				self.processSelfDrivingRobot =StatesSelfDriving.PROCESS_SELFDRIVING_IDLE;
				
		# FINISH GO TO LINE PUT-AWAY*****************************************************************
		
		# REGOTO LINE PUT AREA	
		elif self.processSelfDrivingRobot==StatesSelfDriving.PROCESS_SELFDRIVING_START_REGOTO_PUTAWAYLINE:
            		self.currentgoal_x=	self.robotOrdered.posPutAway.X;
            		self.currentgoal_y=	self.robotOrdered.posPutAway.Y;
					#move base set goal
			self.moveBaseSimple_goal(self.robotOrdered.posPutAway.X,self.robotOrdered.posPutAway.Y,self.robotOrdered.posPutAway.Angle);
            		self.processSelfDrivingRobot = StatesSelfDriving.PROCESS_SELFDRIVING_WAIT_REGOTO_PUTAWAYLINE;
            		self.flag_state_reachedgoal_palletdown=False;
			rospy.loginfo("PROCESS_SELFDRIVING_WAIT_REGOTO_PUTAWAYLINE");
        	elif self.processSelfDrivingRobot==StatesSelfDriving.PROCESS_SELFDRIVING_WAIT_REGOTO_PUTAWAYLINE:
            		if self.flag_state_reachedgoal_palletdown==True:
                		#self.processSelfDrivingRobot = StatesSelfDriving.PROCESS_SELFDRIVING_FINISHED_REGOTO_PUTAWAYLINE;
                		self.flag_state_reachedgoal_palletdown=False;
        	elif self.processSelfDrivingRobot==StatesSelfDriving.PROCESS_SELFDRIVING_FINISHED_REGOTO_PUTAWAYLINE:
				self.processSelfDrivingRobot=StatesSelfDriving.PROCESS_SELFDRIVING_START_PALLETDOWN;
		# PROCESS_SELFDRIVING_ERRORIN_PALLETDOWN ###################################################################
		elif self.processSelfDrivingRobot==StatesSelfDriving.PROCESS_SELFDRIVING_ERRORIN_PALLETDOWN:
			self.processSelfDrivingRobot=StatesSelfDriving.PROCESS_SELFDRIVING_START_REGOTO_PUTAWAYLINE;
			self.flag_state_error_detectline=True;
			
		# START PALLET DOWN
        	elif self.processSelfDrivingRobot==StatesSelfDriving.PROCESS_SELFDRIVING_START_PALLETDOWN:
            		self.linedetectionctrl(RequestCommandLineDetect.REQUEST_LINEDETECT_PALLETDOWN);
			self.flag_state_error_detectline=False;
            		self.processSelfDrivingRobot =StatesSelfDriving.PROCESS_SELFDRIVING_WAIT_PALLETDOWN;
			self.countpos=0;
       		elif self.processSelfDrivingRobot==StatesSelfDriving.PROCESS_SELFDRIVING_WAIT_PALLETDOWN:
            		self.pubFinishedStates(StatesSelfDriving.PROCESS_SELFDRIVING_WAIT_PALLETDOWN);
			if self.amclpose_posX>=	self.robotOrdered.limitedPosPutAway.Xmin:
				self.posPalletCtrl(RequestCommandLineDetect.REQUEST_LINEDETECT_REACHING_POSITION);
        	elif self.processSelfDrivingRobot==StatesSelfDriving.PROCESS_SELFDRIVING_FINISH_PALLETDOWN:
			if self.counterTimeOutRequest(2)==True:
				self.pubFinishedStates(StatesSelfDriving.PROCESS_SELFDRIVING_FINISH_PALLETDOWN);
				for x in range(0,self.amoutOfRefreshLineDetection):
					self.posPalletCtrl(RequestCommandLineDetect.REQUEST_LINEDETECT_REFRESH);
				#self.processSelfDrivingRobot =StatesSelfDriving.PROCESS_SELFDRIVING_IDLE;
				self.cntErrorDetectLine=0;
		# FINISH PALLET DOWN ************************************************************************
		
		#############################################################################
		#-- CHARGE AREA-- #########################################################		
		
		# START GO TO LINE CHARGE AREA
		elif self.processSelfDrivingRobot==StatesSelfDriving.PROCESS_SELFDRIVING_BEGIN_GOTOLINE_CHARGEAREA:
			self.processSelfDrivingRobot =StatesSelfDriving.PROCESS_SELFDRIVING_GOTOLINE_CHARGEAREA;
		elif self.processSelfDrivingRobot==StatesSelfDriving.PROCESS_SELFDRIVING_GOTOLINE_CHARGEAREA:
			self.currentgoal_x=self.robotAtChargeArea.chargeArea.X;
			self.currentgoal_y=self.robotAtChargeArea.chargeArea.Y;
			#move base set goal
			self.moveBaseSimple_goal(self.robotAtChargeArea.chargeArea.X,self.robotAtChargeArea.chargeArea.Y,self.robotAtChargeArea.chargeArea.Angle);
			self.processSelfDrivingRobot = StatesSelfDriving.PROCESS_SELFDRIVING_WAIT_GOTOLINE_CHARGEAREA;
			rospy.loginfo("PROCESS_SELFDRIVING_GOTOLINE_CHARGEAREA\n");
		elif self.processSelfDrivingRobot==StatesSelfDriving.PROCESS_SELFDRIVING_WAIT_GOTOLINE_CHARGEAREA:
			if self.flag_state_reachedgoal_chargearea==True:
				self.processSelfDrivingRobot = StatesSelfDriving.PROCESS_SELFDRIVING_FINISH_GOTOLINE_CHARGEAREA;
                		self.flag_state_reachedgoal_chargearea=False;
		elif self.processSelfDrivingRobot==StatesSelfDriving.PROCESS_SELFDRIVING_FINISH_GOTOLINE_CHARGEAREA:
				#edit no request to server position in charge area
				self.processSelfDrivingRobot=StatesSelfDriving.PROCESS_SELFDRIVING_DETECTLINE_TO_CHARGEAREA;
				self.cntErrorDetectLine=0;
				#self.pubFinishedStates(StatesSelfDriving.PROCESS_SELFDRIVING_FINISH_GOTOLINE_CHARGEAREA);
				#self.processSelfDrivingRobot =StatesSelfDriving.PROCESS_SELFDRIVING_IDLE;
				
		# FINISH GO TO LINE CHARGE AREA****************************************************************	
				
		## REGOTO LINE CHARGE AREA WHEN ERROR APPEAR #######################
		elif self.processSelfDrivingRobot==StatesSelfDriving.PROCESS_SELFDRIVING_REGOTOLINE_CHARGEAREA:
			self.currentgoal_x=self.robotAtChargeArea.chargeArea.X;
			self.currentgoal_y=self.robotAtChargeArea.chargeArea.Y;
			#move base set goal
			self.moveBaseSimple_goal(self.robotAtChargeArea.chargeArea.X,self.robotAtChargeArea.chargeArea.Y,self.robotAtChargeArea.chargeArea.Angle);
			self.processSelfDrivingRobot = StatesSelfDriving.PROCESS_SELFDRIVING_WAIT_REGOTOLINE_CHARGEAREA;
			rospy.loginfo("PROCESS_SELFDRIVING_REGOTOLINE_CHARGEAREA\n");
		elif self.processSelfDrivingRobot==StatesSelfDriving.PROCESS_SELFDRIVING_WAIT_REGOTOLINE_CHARGEAREA:
			if self.flag_state_reachedgoal_chargearea==True:
				#self.processSelfDrivingRobot = StatesSelfDriving.PROCESS_SELFDRIVING_FINISH_REGOTOLINE_CHARGEAREA;
                		self.flag_state_reachedgoal_chargearea=False;
		elif self.processSelfDrivingRobot==StatesSelfDriving.PROCESS_SELFDRIVING_FINISH_REGOTOLINE_CHARGEAREA:
				self.processSelfDrivingRobot=StatesSelfDriving.PROCESS_SELFDRIVING_DETECTLINE_TO_CHARGEAREA;
				
		#DETECTLINE_TO_CHARGEAREA #########################################################
		elif self.processSelfDrivingRobot==StatesSelfDriving.PROCESS_SELFDRIVING_ERRORIN_DETECTLINE_TO_CHARGEAREA:
			self.processSelfDrivingRobot=StatesSelfDriving.PROCESS_SELFDRIVING_REGOTOLINE_CHARGEAREA;
			rospy.loginfo("PROCESS_SELFDRIVING_REGOTOLINE_CHARGEAREA")
			self.flag_state_error_detectline=True;
		
		# START DETECT LINE INSIDE CHARGE AREA
		elif self.processSelfDrivingRobot==StatesSelfDriving.PROCESS_SELFDRIVING_DETECTLINE_TO_CHARGEAREA:
			self.linedetectionctrl(RequestCommandLineDetect.REQUEST_LINEDETECT_CHARGEAREA);
			self.flag_state_error_detectline=False;
			self.processSelfDrivingRobot=StatesSelfDriving.PROCESS_SELFDRIVING_WAIT_DETECTLINE_TO_CHARGEAREA;
			rospy.loginfo("PROCESS_SELFDRIVING_WAIT_DETECTLINE_TO_CHARGEAREA");
		elif self.processSelfDrivingRobot==StatesSelfDriving.PROCESS_SELFDRIVING_WAIT_DETECTLINE_TO_CHARGEAREA:
			if self.amclpose_posX>=self.robotAtChargeArea.limitedChargeArea.Xmin:
				self.processSelfDrivingRobot =StatesSelfDriving.PROCESS_SELFDRIVING_FINISH_DETECTLINE_TO_CHARGEAREA;
				self.posPalletCtrl(RequestCommandLineDetect.REQUEST_LINEDETECT_REACHING_POSITION);
		elif self.processSelfDrivingRobot==StatesSelfDriving.PROCESS_SELFDRIVING_FINISH_DETECTLINE_TO_CHARGEAREA:
			#self.pubChargeIP(self.robotAtChargeArea.url);
			self.pubFinishedStates(StatesSelfDriving.PROCESS_SELFDRIVING_FINISH_DETECTLINE_TO_CHARGEAREA);
			self.processSelfDrivingRobot =StatesSelfDriving.PROCESS_SELFDRIVING_START_BATTERYCHARGED;
			self.cntErrorDetectLine=0;
		# FINISH DETECT LINE INSIDE CHARGE AREA****************************************************************
		
		elif self.processSelfDrivingRobot==StatesSelfDriving.PROCESS_SELFDRIVING_START_BATTERYCHARGED:
			self.pubChargeIP(self.robotAtChargeArea.url);
			self.linedetectionctrl(RequestCommandLineDetect.REQUEST_CHARGECTRL_START);
			self.pubFinishedStates(StatesSelfDriving.PROCESS_SELFDRIVING_START_BATTERYCHARGED);
			self.processSelfDrivingRobot =StatesSelfDriving.PROCESS_SELFDRIVING_WAIT_BATTERYCHARGING;
		elif self.processSelfDrivingRobot==StatesSelfDriving.PROCESS_SELFDRIVING_WAIT_BATTERYCHARGING:
			rospy.loginfo("PROCESS_SELFDRIVING_WAIT_BATTERYCHARGING");
		elif self.processSelfDrivingRobot==StatesSelfDriving.PROCESS_SELFDRIVING_FINISH_BATTERYCHARGING:
			self.pubFinishedStates(StatesSelfDriving.PROCESS_SELFDRIVING_FINISH_BATTERYCHARGING);
		elif self.processSelfDrivingRobot==StatesSelfDriving.PROCESS_SELFDRIVING_CANCEL_BATTERYCHARGING:
			rospy.loginfo("PROCESS_SELFDRIVING_CANCEL_BATTERYCHARGING");
		#############################################################################
		#-- READY AREA-- #########################################################
		
		# START GO TO LINE READY AREA
		elif self.processSelfDrivingRobot==StatesSelfDriving.PROCESS_SELFDRIVING_BEGIN_GOTOLINE_READYAREA:
			self.processSelfDrivingRobot =StatesSelfDriving.PROCESS_SELFDRIVING_GOTOLINE_READYAREA;
		elif self.processSelfDrivingRobot==StatesSelfDriving.PROCESS_SELFDRIVING_GOTOLINE_READYAREA:
			self.currentgoal_x=self.robotAtReadyArea.readyArea.X;
			self.currentgoal_y=self.robotAtReadyArea.readyArea.Y;
						#move base set goal
			self.moveBaseSimple_goal(self.robotAtReadyArea.readyArea.X,self.robotAtReadyArea.readyArea.Y,self.robotAtReadyArea.readyArea.Angle);
			self.processSelfDrivingRobot = StatesSelfDriving.PROCESS_SELFDRIVING_WAIT_GOTOLINE_READYAREA;
			rospy.loginfo("PROCESS_SELFDRIVING_GOTOLINE_READYAREA\n");
		elif self.processSelfDrivingRobot==StatesSelfDriving.PROCESS_SELFDRIVING_WAIT_GOTOLINE_READYAREA:
			if self.flag_state_reachedgoal_readyarea==True:
				self.processSelfDrivingRobot = StatesSelfDriving.PROCESS_SELFDRIVING_FINISH_GOTOLINE_READYAREA;
                		self.flag_state_reachedgoal_readyarea=False;
		elif self.processSelfDrivingRobot==StatesSelfDriving.PROCESS_SELFDRIVING_FINISH_GOTOLINE_READYAREA:
				#edit no request to server position in ready area
				self.processSelfDrivingRobot=StatesSelfDriving.PROCESS_SELFDRIVING_DETECTLINE_TO_READYAREA
				self.cntErrorDetectLine=0;
				#self.pubFinishedStates(StatesSelfDriving.PROCESS_SELFDRIVING_FINISH_GOTOLINE_READYAREA);
				#self.processSelfDrivingRobot =StatesSelfDriving.PROCESS_SELFDRIVING_IDLE;
				
		# FINISH GO TO LINE READY AREA ****************************************************************************
		
		# REGOTO LINE READY AREA
		elif self.processSelfDrivingRobot==StatesSelfDriving.PROCESS_SELFDRIVING_REGOTOLINE_READYAREA:
			self.currentgoal_x=self.robotAtReadyArea.readyArea.X;
			self.currentgoal_y=self.robotAtReadyArea.readyArea.Y;
						#move base set goal
			self.moveBaseSimple_goal(self.robotAtReadyArea.readyArea.X,self.robotAtReadyArea.readyArea.Y,self.robotAtReadyArea.readyArea.Angle);
			self.processSelfDrivingRobot = StatesSelfDriving.PROCESS_SELFDRIVING_WAIT_REGOTOLINE_READYAREA;
			rospy.loginfo("PROCESS_SELFDRIVING_REGOTOLINE_READYAREA\n");
		elif self.processSelfDrivingRobot==StatesSelfDriving.PROCESS_SELFDRIVING_WAIT_REGOTOLINE_READYAREA:
			if self.flag_state_reachedgoal_readyarea==True:
				self.processSelfDrivingRobot = StatesSelfDriving.PROCESS_SELFDRIVING_FINISH_REGOTOLINE_READYAREA;
                		self.flag_state_reachedgoal_readyarea=False;
		elif self.processSelfDrivingRobot==StatesSelfDriving.PROCESS_SELFDRIVING_FINISH_REGOTOLINE_READYAREA:
				self.processSelfDrivingRobot=StatesSelfDriving.PROCESS_SELFDRIVING_DETECTLINE_TO_READYAREA;
		# ERRORIN_DETECTLINE_TO_READYAREA ######################################################
		elif self.processSelfDrivingRobot==StatesSelfDriving.PROCESS_SELFDRIVING_ERRORIN_DETECTLINE_TO_READYAREA:
			self.processSelfDrivingRobot=StatesSelfDriving.PROCESS_SELFDRIVING_REGOTOLINE_READYAREA;
			rospy.loginfo(StatesSelfDriving.PROCESS_SELFDRIVING_REGOTOLINE_READYAREA.value);
			self.flag_state_error_detectline=True;
		
		# START DETECT LINE INSIDE READY AREA
		elif self.processSelfDrivingRobot==StatesSelfDriving.PROCESS_SELFDRIVING_DETECTLINE_TO_READYAREA:
			self.linedetectionctrl(RequestCommandLineDetect.REQUEST_LINEDETECT_READYAREA);
			self.flag_state_error_detectline=False;
			self.processSelfDrivingRobot=StatesSelfDriving.PROCESS_SELFDRIVING_WAIT_DETECTLINE_TO_READYAREA;
			rospy.loginfo("PROCESS_SELFDRIVING_WAIT_DETECTLINE_TO_READYAREA");
		elif self.processSelfDrivingRobot==StatesSelfDriving.PROCESS_SELFDRIVING_WAIT_DETECTLINE_TO_READYAREA:
			if self.amclpose_posX>=self.robotAtReadyArea.limitedReadyArea.Xmin:
				self.processSelfDrivingRobot =StatesSelfDriving.PROCESS_SELFDRIVING_FINISH_DETECTLINE_TO_READYAREA;
				self.posPalletCtrl(RequestCommandLineDetect.REQUEST_LINEDETECT_REACHING_POSITION);
		elif self.processSelfDrivingRobot==StatesSelfDriving.PROCESS_SELFDRIVING_FINISH_DETECTLINE_TO_READYAREA:
			self.pubFinishedStates(StatesSelfDriving.PROCESS_SELFDRIVING_FINISH_DETECTLINE_TO_READYAREA);
			self.processSelfDrivingRobot =StatesSelfDriving.PROCESS_SELFDRIVING_IDLE;
			self.cntErrorDetectLine=0;
		# FINISH DETECT LINE INSIDE READY AREA *********************************************************************

		

if __name__ == '__main__':
    try:
	#call("rosrun seldat_robot reachGoal&", shell=True)  
    	selfDriving = SelfDriving()
	selfDriving.spin()
    except rospy.ROSInterruptException:
        pass
