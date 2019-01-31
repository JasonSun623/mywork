#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <sys/types.h>
#include <time.h>
#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Int32.h"
#include "std_msgs/String.h"
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Vector3.h>
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include "actionlib_msgs/GoalStatusArray.h"
#include "std_msgs/Float32MultiArray.h"
#include <nav_msgs/Odometry.h>
#include "rapidjson/document.h"
#include <std_msgs/UInt8MultiArray.h>
#include <turtlesim/Pose.h>
#include <json/json.h>
#include <sstream>
#include <math.h>
#include <pthread.h>
#include <list>
#include <vector>
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
using namespace std;
using namespace rapidjson;
struct Position;
void requestedprogress_test();
void MoveBaseSimple_Goal(double posx,double posy, double angle);
void BatteryChargeRequestSelfDriving_callback(const std_msgs::String::ConstPtr& msg);
void batterysub_callback(const std_msgs::Float32::ConstPtr& msg);
void servergotoPalletAreaCallBack(const std_msgs::String::ConstPtr& msg);
void LineDetection_callback(const std_msgs::Int32& msg);
void moveBaseStatus_Callback(const actionlib_msgs::GoalStatusArrayConstPtr& msg);
void navigationAmclPose_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
void currentgoal_Callback (const geometry_msgs::PoseStamped pose);
void odometry_callback(const nav_msgs::Odometry &odom);
void reachrobotdirection_callback(const std_msgs::String::ConstPtr& msg);
void errorDetectedLine_callback(const std_msgs::Int32& msg);
void process();
void serverRobotGotToLineDockingAreaCallBack(const std_msgs::String::ConstPtr& msg);
void serverRobotGotToLinePutAwayAreaCallBack(const std_msgs::String::ConstPtr& msg);
void serverRobotGotToPalletDockingAreaCallBack(const std_msgs::String::ConstPtr& msg);
void serverRobotGotToPalletPutAwayAreaCallBack(const std_msgs::String::ConstPtr& msg);

void serverRobotGotToCheckInDockingAreaCallBack(const std_msgs::String::ConstPtr& msg);
void serverRobotGotToCheckInPutAwayAreaCallBack(const std_msgs::String::ConstPtr& msg);
void sub_serverRobotGotToFrontReadyAreaCallBack(const std_msgs::String::ConstPtr& msg);

void responsedStandPosInsideReadyArea_callback(const std_msgs::String::ConstPtr& msg);
void serverResetErrorDetectLineCallBack(const std_msgs::Int32& msg);
void updateRobotStatus(int status);
//void servergotoPalletAreaCallBack_bt(const std_msgs::String::ConstPtr& msg);
#define PI 3.14159265
ros::Publisher pub_navigation_setgoal;
ros::Publisher pub_posPallet;
ros::Publisher pub_linedetectionctrl;
ros::Publisher pub_numworkingtimes;
ros::Publisher pub_errorturningrobot;
ros::Publisher pub_batteryRequestDetectLine;
ros::Publisher pub_BatteryChargeResponseStatusSelfDriving;
ros::Publisher pub_ChargeBattery;
ros::Publisher pub_RobotStatus;
ros::Publisher pub_requestReadyAreaPos;
ros::Publisher pub_FinishedStates;

//ros::Publisher pub_PalletLocation;

const int PROCESS_SELFDRIVING_WAIT_GOTO_CHECKINDOCKING = 500021;
const int PROCESS_SELFDRIVING_FINISHED_GOTO_CHECKINDOCKING = 500031;

const int PROCESS_SELFDRIVING_ATREADY =50000;
const int PROCESS_SELFDRIVING_START_GOTO_DOCKINGLINE = 50001;
const int PROCESS_SELFDRIVING_WAIT_GOTO_DOCKINGLINE = 50002;
const int PROCESS_SELFDRIVING_FINISHED_GOTO_DOCKINGLINE = 50003;

const int PROCESS_SELFDRIVING_START_PALLETUP = 50004;
const int PROCESS_SELFDRIVING_ERRORIN_PALLETUP = 50005;
const int PROCESS_SELFDRIVING_WAIT_PALLETUP = 50006;
const int PROCESS_SELFDRIVING_FINISH_PALLETUP = 50007;

const int PROCESS_SELFDRIVING_WAIT_GOTO_CHECKINPUTAWAY = 5000212;
const int PROCESS_SELFDRIVING_FINISHED_GOTO_CHECKINPUTAWAY = 500129;

const int PROCESS_SELFDRIVING_START_GOTO_PUTAWAYLINE = 50008;
const int PROCESS_SELFDRIVING_WAIT_GOTO_PUTAWAYLINE = 50009;
const int PROCESS_SELFDRIVING_FINISHED_GOTO_PUTAWAYLINE = 50010;

const int PROCESS_SELFDRIVING_START_PALLETDOWN = 50011;
const int PROCESS_SELFDRIVING_ERRORIN_PALLETDOWN = 50012;
const int PROCESS_SELFDRIVING_WAIT_PALLETDOWN = 50013;
const int PROCESS_SELFDRIVING_FINISH_PALLETDOWN = 50014;

const int PROCESS_SELFDRIVING_BEGIN_GOTOLINE_CHARGESTATION = 500111;
const int PROCESS_SELFDRIVING_GOTOLINE_CHARGINGSTATION = 500121;
const int PROCESS_SELFDRIVING_WAIT_GOTOLINE_CHARGINGSTATION = 500131;
const int PROCESS_SELFDRIVING_FINISH_GOTOLINE_CHARGINGSTATION  = 500141;
const int PROCESS_SELFDRIVING_DETECTLINE_TO_CHARGINGSTATION = 5001213;
const int PROCESS_SELFDRIVING_WAIT_DETECTLINE_TO_CHARGINGSTATION = 5001313;
const int PROCESS_SELFDRIVING_FINISH_DETECTLINE_TO_CHARGINGSTATION  = 5001413;
const int PROCESS_SELFDRIVING_FINISH_RETURN_THELINNE = 3453334;
const int PROCESS_SELFDRIVING_WAITING_RETURN_THELINNE = 3453339;
const int PROCESS_SELFDRIVING_ERRORIN_DETECTLINE_TO_CHARGINGSTATION = 5001299;


const int PROCESS_SELFDRIVING_START_BATTERYCHARGED = 5001211;
const int PROCESS_SELFDRIVING_WAIT_BATTERYCHARGING = 5001311;
const int PROCESS_SELFDRIVING_FINISH_BATTERYCHARGING  = 5001411;
const int PROCESS_SELFDRIVING_CANCEL_BATTERYCHARGING  = 500654;
const int PROCESS_SELFDRIVING_LOW_BATTERY  = 509654;
const int PROCESS_SELFDRIVING_FULL_BATTERY  = 509654;

const int PROCESS_SELFDRIVING_START_GOTO_STATION= 5000811;
const int PROCESS_SELFDRIVING_WAIT_GOTO_STATION= 5000911;
const int PROCESS_SELFDRIVING_FINISHED_GOTO_STATION= 5001011;


const int PROCESS_SELFDRIVING_RESET = 50015;
const int PROCESS_SELFDRIVING_STOP  = 50016;
const int PROCESS_SELFDRIVING_PAUSE  = 50017;
const int PROCESS_SELFDRIVING_IDLE=9879;

const int ROBOT_STATUS_IDLE 	 =1000;
const int ROBOT_STATUS_READY     =1001;
const int ROBOT_STATUS_ORDERED   =1002;
const int ROBOT_STATUS_WORKING   =1003;
const int ROBOT_STATUS_FINISH    =1004;
const int ROBOT_STATUS_CANCELED  =1005;
const int ROBOT_STATUS_CHARGING  =1006;
int robotstatus=ROBOT_STATUS_READY ;


#define MAXWORKINGTIMES 100
#define MAXNUMBER_OF_ERRORLINE 3
//int processSelfDrivingRobot = PROCESS_SELFDRIVING_START_READY;
int processSelfDrivingRobot=PROCESS_SELFDRIVING_IDLE  ;

double amclpose_posX=10.0,amclpose_posY=10.0,amclpose_posthetaW=0.0,amclpose_posthetaZ=0.0;
double currentgoal_x=0.0,currentgoal_y=0.0,currentgoal_z=0.0,currentgoal_w=0.0;
double current_Vx=0.0,current_Vy=0.0,current_W=0.0;
int cntErrorDetectLine=0;
struct Position{
	double X;
	double Y;
	double Angle;
};

struct BatteryInfo{
	
	double currentBatteryLevel=0.0;
	double defautLowBattery=23.0;	
	
	const int CommandRequest_begin_GoTo_ChargeStation=7600;
	const int CommandRequest_GoTo_ChargeStation=7700;
	const int CommandRequest_GoTo_Station=7701;
	const int CommandRequest_DetectLineCtrl_ChargeStation=1205;
	const int SendRequest_Finish_GoTo_ChargeStation=7800;
	const int SendRequest_Finish_DetectLine_ChargeStation=7801;
	const int SendRequest_Finish_GoTo_Station=7802;
	
	bool flagRequestChargeBattery=false;
	bool flagRobotGoToChargeStation=false;
	std_msgs::String msgdataRequest;
	Position chargeStationPos;
	Position stationPos;
};
BatteryInfo batteryInfo;

struct LimitedPosition{
	double Xmin;
	double Xmax;
	int palletAtNum;
	string iPCharger; 
};
struct RobotOrdered
{
	Position posCheckInDocking;
	Position posDocking;
	LimitedPosition limitedPosDocking;
	Position posCheckInPutAway;
	Position posPutAway;
	LimitedPosition limitedPosPutAway;
	int posPalletPutAWay;
};
struct RobotAtReady
{
	Position ready;
	LimitedPosition limitedReadyArea;
	int readynum;
};
RobotOrdered robotOrdered;
RobotAtReady robotAtReady;


int looptime_progress=0;
int atPos=0;
bool flag_state_reachedgoal_palletup=false;
bool flag_state_reachedgoal_palletdown=false;
bool flag_state_reachedgoal_battery=false;
bool flag_state_reachedgoal_checkindocking=false;
bool flag_state_reachedgoal_checkinputaway=false;
bool flag_state_error_detectline=false; // dam bao request robot request vi tri pallet 1 lần
bool flag_state_atReady=true; // dam bao request robot request vi tri pallet 1 lần

void selfdriving();
// Subcribe From Line Detection
// Subcribe From Navigation
// Subcribe From Manager
int main(int argc, char *argv[])
{
	ros::init(argc, argv,"selfdriving");
	ros::NodeHandle nh;
	//pub = nh.advertise<std_msgs::UInt8MultiArray>("callbridgeNode", 100);
	pub_navigation_setgoal=nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal",1);
	pub_posPallet=nh.advertise<std_msgs::Int32>("pospallet",100);
	//pub_PalletLocation=nh.advertise<std_msgs::Int32>("palletlocation",1000);
	pub_numworkingtimes=nh.advertise<std_msgs::Int32>("numworkingtimes",1);
	pub_linedetectionctrl=nh.advertise<std_msgs::Int32>("linedetectionctrl", 100);
	pub_errorturningrobot=nh.advertise<std_msgs::Int32>("errorturningrobot", 1);
	pub_BatteryChargeResponseStatusSelfDriving=nh.advertise<std_msgs::Int32>("batteryselfdrivingcallback", 1);
	pub_ChargeBattery=nh.advertise<std_msgs::String>("/chargeIP", 1);
	pub_RobotStatus=nh.advertise<std_msgs::Int32>("/robotStatus", 1);
	pub_requestReadyAreaPos=nh.advertise<std_msgs::Int32>("/requestReadyAreaPos", 1);
	
	pub_FinishedStates=nh.advertise<std_msgs::Int32>("/finishedStates", 1);
	
	ros::Subscriber sub_responsedStandPosInsideReadyArea= nh.subscribe("responsedStandPosInsideReadyArea",1,responsedStandPosInsideReadyArea_callback);
	ros::Subscriber sub_batteryvol= nh.subscribe("battery_vol",1,batterysub_callback);
	ros::Subscriber sub_CurrentPos = nh.subscribe("servergotoPalletAreaCallBack",1, servergotoPalletAreaCallBack);
	ros::Subscriber sub_ReachedRobotdirection= nh.subscribe("ReachedRobotdirection",1, reachrobotdirection_callback);
	ros::Subscriber sub_errorDetectedLine= nh.subscribe("errorDetectedLine",1, errorDetectedLine_callback);
	ros::Subscriber sub_fromLineDetection_Node= nh.subscribe("linedetectioncallback",1,LineDetection_callback);
	ros::Subscriber sub_amclpose = nh.subscribe("amcl_pose",100,navigationAmclPose_callback);
	ros::Subscriber sub_currentgoal= nh.subscribe("move_base_simple/goal",1,currentgoal_Callback);
	ros::Subscriber sub_odometry= nh.subscribe("odom",1,odometry_callback);
	ros::Subscriber sub_movebase_status= nh.subscribe("move_base/status",1,moveBaseStatus_Callback);
	
	ros::Subscriber sub_serverRobotGotToLineDockingArea = nh.subscribe("serverRobotGotToLineDockingArea",1,serverRobotGotToLineDockingAreaCallBack);
	ros::Subscriber sub_serverRobotGotToPalletDockingArea = nh.subscribe("serverRobotGotToPalletDockingArea",1,serverRobotGotToPalletDockingAreaCallBack);

	
	ros::Subscriber sub_serverRobotGotToLinePutAwayArea = nh.subscribe("serverRobotGotToLinePutAwayArea",1,serverRobotGotToLinePutAwayAreaCallBack);
	ros::Subscriber sub_serverRobotGotToPalletPutAwayArea = nh.subscribe("serverRobotGotToPalletPutAwayArea",1,serverRobotGotToPalletPutAwayAreaCallBack);

	ros::Subscriber sub_serverRobotGotToCheckInPutAwayArea = nh.subscribe("serverRobotGotToCheckInPutAwayArea",1,serverRobotGotToCheckInPutAwayAreaCallBack);
	ros::Subscriber sub_serverRobotGotToCheckInDockingArea = nh.subscribe("serverRobotGotToCheckInDockingArea",1,serverRobotGotToCheckInDockingAreaCallBack);
	
	ros::Subscriber sub_serverRobotGotToFrontReadyArea = nh.subscribe("serverRobotGotToFrontReadyArea",1,sub_serverRobotGotToFrontReadyAreaCallBack);
	
	ros::Subscriber sub_serverResetErrorDetectLine = nh.subscribe("serverResetErrorDetectLine",1,serverResetErrorDetectLineCallBack);
	

	ros::Rate loop_rate(10);
    ros::spinOnce();
    int nk=0;

	while(ros::ok())
	{
	    process();
	    ros::spinOnce();
		loop_rate.sleep();
	}
}
void batterysub_callback (const std_msgs::Float32::ConstPtr& msg)
{
	batteryInfo.currentBatteryLevel=(double)msg->data;
}
void navigationAmclPose_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg){
	amclpose_posX=msg->pose.pose.position.x;
	amclpose_posY=msg->pose.pose.position.y;
	amclpose_posthetaW=msg->pose.pose.orientation.w;
	amclpose_posthetaZ=msg->pose.pose.orientation.z;
}
void  currentgoal_Callback (const geometry_msgs::PoseStamped pose)
{
	/*currentgoal_x=pose.pose.position.x;
	currentgoal_y=pose.pose.position.y;
	currentgoal_z=pose.pose.orientation.z;
	currentgoal_w=pose.pose.orientation.w;*/
	//printf(" %0.2f / %0.2f / %0.2f / %0.2f",currentgoal_x,currentgoal_y,currentgoal_z,currentgoal_w);
}

void serverResetErrorDetectLineCallBack(const std_msgs::Int32& msg)
{
	// reset detect Line
	cout<<"reset detect Line \n";
	
		cntErrorDetectLine=0;
		if((int)msg.data==1000)
		{
			cout<<"RESET AT PROCESS_SELFDRIVING_WAIT_PALLETUP \n";
				processSelfDrivingRobot=PROCESS_SELFDRIVING_START_GOTO_DOCKINGLINE;
		}
		else if((int)msg.data==1001)	
		{
			cout<<"RESET AT PROCESS_SELFDRIVING_WAIT_PALLETDOWN \n";
				processSelfDrivingRobot=PROCESS_SELFDRIVING_START_GOTO_PUTAWAYLINE;
		}
}
void serverRobotGotToCheckInDockingAreaCallBack(const std_msgs::String::ConstPtr& msg)
{
	cout<<msg->data<<"\n";
	Document document;
	document.Parse(msg->data.c_str());
	robotOrdered.posCheckInDocking.X=document["docking"]["checkin"]["X"].GetDouble();
	robotOrdered.posCheckInDocking.Y=document["docking"]["checkin"]["Y"].GetDouble();
	robotOrdered.posCheckInDocking.Angle=document["docking"]["checkin"]["Angle"].GetDouble();
	MoveBaseSimple_Goal(robotOrdered.posCheckInDocking.X,robotOrdered.posCheckInDocking.Y,robotOrdered.posCheckInDocking.Angle);
	currentgoal_x=robotOrdered.posCheckInDocking.X;
    currentgoal_y=robotOrdered.posCheckInDocking.Y;
	cout<<"PROCESS_SELFDRIVING_WAIT_GOTO_CHECKINDOCKING\n";
	flag_state_atReady=false;
	processSelfDrivingRobot =PROCESS_SELFDRIVING_WAIT_GOTO_CHECKINDOCKING;
}

void serverRobotGotToCheckInPutAwayAreaCallBack(const std_msgs::String::ConstPtr& msg)
{
	cout<<msg->data<<"\n";
	Document document;
	document.Parse(msg->data.c_str());
	robotOrdered.posCheckInPutAway.X=document["putaway"]["checkin"]["X"].GetDouble();
	robotOrdered.posCheckInPutAway.Y=document["putaway"]["checkin"]["Y"].GetDouble();
	robotOrdered.posCheckInPutAway.Angle=document["putaway"]["checkin"]["Angle"].GetDouble();
	MoveBaseSimple_Goal(robotOrdered.posCheckInPutAway.X,robotOrdered.posCheckInPutAway.Y,robotOrdered.posCheckInPutAway.Angle);
	currentgoal_x=robotOrdered.posCheckInPutAway.X;
    currentgoal_y=robotOrdered.posCheckInPutAway.Y;
		cout<<"PROCESS_SELFDRIVING_WAIT_GOTO_CHECKINPUTAWAY \n";
	processSelfDrivingRobot =PROCESS_SELFDRIVING_WAIT_GOTO_CHECKINPUTAWAY;
}
void serverRobotGotToLineDockingAreaCallBack(const std_msgs::String::ConstPtr& msg)
{
		cout<<msg->data<<"\n";
		Document document;
		document.Parse(msg->data.c_str());
		robotOrdered.posDocking.X=document["docking"]["line"]["X"].GetDouble();
		robotOrdered.posDocking.Y=document["docking"]["line"]["Y"].GetDouble();
		robotOrdered.posDocking.Angle=document["docking"]["line"]["Angle"].GetDouble();
		processSelfDrivingRobot = PROCESS_SELFDRIVING_START_GOTO_DOCKINGLINE;
}
void serverRobotGotToPalletDockingAreaCallBack(const std_msgs::String::ConstPtr& msg)
{
		cout<<msg->data<<"\n";
		Document document;
		document.Parse(msg->data.c_str());
		robotOrdered.limitedPosDocking.Xmax=document["docking"]["pallet"]["Xmax"].GetDouble();
		robotOrdered.limitedPosDocking.Xmin=document["docking"]["pallet"]["Xmin"].GetDouble();
		processSelfDrivingRobot = PROCESS_SELFDRIVING_START_PALLETUP;
		//processSelfDrivingRobot = PROCESS_SELFDRIVING_START_GOTO_DOCKINGLINE;
}
void serverRobotGotToLinePutAwayAreaCallBack(const std_msgs::String::ConstPtr& msg)
{
		cout<<msg->data<<"\n";
		Document document;
		document.Parse(msg->data.c_str());
		robotOrdered.posPutAway.X=document["putaway"]["line"]["X"].GetDouble();
		robotOrdered.posPutAway.Y=document["putaway"]["line"]["Y"].GetDouble();
		robotOrdered.posPutAway.Angle=document["putaway"]["line"]["Angle"].GetDouble();
		processSelfDrivingRobot = PROCESS_SELFDRIVING_START_GOTO_PUTAWAYLINE;

}
void serverRobotGotToPalletPutAwayAreaCallBack(const std_msgs::String::ConstPtr& msg)
{
		cout<<msg->data<<"\n";
		Document document;
		document.Parse(msg->data.c_str());
		robotOrdered.limitedPosPutAway.Xmax=document["putaway"]["pallet"]["Xmax"].GetDouble();
		robotOrdered.limitedPosPutAway.Xmin=document["putaway"]["pallet"]["Xmin"].GetDouble();
		processSelfDrivingRobot =PROCESS_SELFDRIVING_START_PALLETDOWN;
		//processSelfDrivingRobot = PROCESS_SELFDRIVING_START_GOTO_PUTAWAYLINE;
}
void sub_serverRobotGotToFrontReadyAreaCallBack(const std_msgs::String::ConstPtr& msg)
{
	cout<<msg->data<<"\n";
	Document document;
	document.Parse(msg->data.c_str());
	robotAtReady.ready.X=document["ready"]["X"].GetDouble();
	robotAtReady.ready.Y=document["ready"]["Y"].GetDouble();
	robotAtReady.ready.Angle=document["ready"]["Angle"].GetDouble();
	processSelfDrivingRobot = PROCESS_SELFDRIVING_BEGIN_GOTOLINE_CHARGESTATION;
}
void odometry_callback(const nav_msgs::Odometry &odom)
{
	current_Vx=odom.twist.twist.linear.x;
	current_Vy =odom.twist.twist.linear.y;
	current_W  =odom.twist.twist.angular.z;
}
void servergotoPalletAreaCallBack(const std_msgs::String::ConstPtr& msg)
{
	robotstatus=ROBOT_STATUS_ORDERED;
	cout<<"ROBOT STATUS"<<robotstatus;
	processSelfDrivingRobot =PROCESS_SELFDRIVING_ATREADY;
	flag_state_reachedgoal_palletup=false;
	flag_state_reachedgoal_palletdown=false;
}
void updateRobotStatus(int status)
{
	std_msgs::Int32 dir;
	dir.data=status;
	pub_RobotStatus.publish(dir);
}
void responsedStandPosInsideReadyArea_callback(const std_msgs::String::ConstPtr& msg)	
{
	Document document;
	document.Parse(msg->data.c_str());
	cout<<msg->data.c_str();
	robotAtReady.limitedReadyArea.Xmax=document["ready"]["Xmax"].GetDouble();
	robotAtReady.limitedReadyArea.Xmin=document["ready"]["Xmin"].GetDouble();
	robotAtReady.limitedReadyArea.iPCharger=document["ready"]["IP"].GetString();
	processSelfDrivingRobot = PROCESS_SELFDRIVING_DETECTLINE_TO_CHARGINGSTATION;
}
void reachrobotdirection_callback(const std_msgs::String::ConstPtr& msg)
{
	if(processSelfDrivingRobot==PROCESS_SELFDRIVING_WAIT_PALLETUP)
	{
		if(amclpose_posY-(-2.1)<0)
		{

			 std_msgs::Int32 dir;
			 dir.data=1;
			 pub_errorturningrobot.publish(dir);
		}
		else
		{

			 std_msgs::Int32 dir;
			 dir.data=-1;
			 pub_errorturningrobot.publish(dir);
		}
	}
	else if(processSelfDrivingRobot==PROCESS_SELFDRIVING_WAIT_PALLETDOWN)
	{
		if(amclpose_posY-(-1.25))
		{

			 std_msgs::Int32 dir;
			 dir.data=-1;
			 pub_errorturningrobot.publish(dir);
		}
		else
		{

			 std_msgs::Int32 dir;
			 dir.data=1;
			 pub_errorturningrobot.publish(dir);
		}
	}

}
void errorDetectedLine_callback(const std_msgs::Int32& msg)
{
	if((int)msg.data==4205) // error line detected
	{
    	if(cntErrorDetectLine++<MAXNUMBER_OF_ERRORLINE)
		{
			if(processSelfDrivingRobot==PROCESS_SELFDRIVING_WAIT_PALLETUP)
			{
				processSelfDrivingRobot=PROCESS_SELFDRIVING_ERRORIN_PALLETUP;
			}
			else if(processSelfDrivingRobot==PROCESS_SELFDRIVING_WAIT_PALLETDOWN)	
			{
				processSelfDrivingRobot=PROCESS_SELFDRIVING_ERRORIN_PALLETDOWN;
			}
			else if(processSelfDrivingRobot==PROCESS_SELFDRIVING_WAIT_DETECTLINE_TO_CHARGINGSTATION)
			{
				processSelfDrivingRobot=PROCESS_SELFDRIVING_ERRORIN_DETECTLINE_TO_CHARGINGSTATION;
			}
		}
	}

}
void LineDetection_callback(const std_msgs::Int32& msg)
{
	cout <<"IN END LINE"<<"\n";
  if((int)msg.data==3203)
  {
    	processSelfDrivingRobot=PROCESS_SELFDRIVING_FINISH_PALLETUP;
    	cout <<"END LINE"<<"\n";
  }
  else if((int)msg.data==3204)
  {
    	processSelfDrivingRobot=PROCESS_SELFDRIVING_FINISH_PALLETDOWN;
    	cout <<"END LINE"<<"\n";
  }
  else if((int)msg.data==3206)
  {
    	processSelfDrivingRobot=PROCESS_SELFDRIVING_FINISH_DETECTLINE_TO_CHARGINGSTATION;
    	cout <<"END LINE"<<"\n";
  }
  else if((int)msg.data==3207)
  {
    	processSelfDrivingRobot=PROCESS_SELFDRIVING_FINISH_RETURN_THELINNE;
    	cout <<"END LINE"<<"\n";
  }
}

void moveBaseStatus_Callback(const actionlib_msgs::GoalStatusArrayConstPtr& msg)
{
	if(!msg->status_list.empty() && msg->status_list.back().status == msg->status_list.back().SUCCEEDED){
	double _currentgoal_Ex = fabs(fabs(amclpose_posX)-fabs(currentgoal_x));
	double _currentgoal_Ey = fabs(fabs(amclpose_posY)-fabs(currentgoal_y));
	double _currentgoal_Ez = fabs(fabs(amclpose_posthetaZ)-fabs(currentgoal_z));
	double _currentgoal_Ew = fabs(fabs(amclpose_posthetaW)-fabs(currentgoal_w));
	 if(fabs(current_Vx)<0.0001 && fabs(current_Vy)<0.0001 && fabs(current_W)<0.0001 && _currentgoal_Ex<=0.8 && _currentgoal_Ey<=0.8)
	{
		// printf(" %0.2f / %0.2f / %0.2f / %0.2f \n",_currentgoal_Ex,_currentgoal_Ey,_currentgoal_Ez,_currentgoal_Ew);
		if(flag_state_reachedgoal_palletup==false)
		{
			if(processSelfDrivingRobot ==PROCESS_SELFDRIVING_WAIT_GOTO_DOCKINGLINE)
			{
				flag_state_reachedgoal_palletup=true;
			}
		}
		if(flag_state_reachedgoal_palletdown==false)
		{
			if(processSelfDrivingRobot ==PROCESS_SELFDRIVING_WAIT_GOTO_PUTAWAYLINE)
			{
					flag_state_reachedgoal_palletdown=true;
			}
		}
		
		if(flag_state_reachedgoal_checkindocking == false)
		{
			if(processSelfDrivingRobot ==PROCESS_SELFDRIVING_WAIT_GOTO_CHECKINDOCKING)
			{
					flag_state_reachedgoal_checkindocking=true;
			}
		}
		if(flag_state_reachedgoal_checkinputaway == false)
		{
			if(processSelfDrivingRobot ==PROCESS_SELFDRIVING_WAIT_GOTO_CHECKINPUTAWAY)
			{
					flag_state_reachedgoal_checkinputaway=true;
			}
		}
		if(batteryInfo.flagRobotGoToChargeStation==false)
		{
			if(processSelfDrivingRobot ==PROCESS_SELFDRIVING_WAIT_GOTOLINE_CHARGINGSTATION)
			{
					batteryInfo.flagRobotGoToChargeStation=true;
			}
		}
	}	

  }
}
void posPalletCtrl(int numctrl)
{
	std_msgs::Int32 tmpp;
	tmpp.data=numctrl; 
	pub_posPallet.publish(tmpp);
}
void linedetectionctrl(int numctrl)
{
	std_msgs::Int32 tmpp;
	tmpp.data=numctrl; 
	pub_linedetectionctrl.publish(tmpp);
}
void pubFinishedStates(int num)
{
	std_msgs::Int32 tmpp;
	tmpp.data=num; 
	pub_FinishedStates.publish(tmpp);
}
void pubChargeIP(string ip)
{
	std_msgs::String tmpp;
	tmpp.data=ip; 
	pub_ChargeBattery.publish(tmpp);
}
int countpos=0;
void process()
{
         switch (processSelfDrivingRobot)
        {
           case PROCESS_SELFDRIVING_STOP:
				robotstatus=ROBOT_STATUS_IDLE;
        	 break;
             case PROCESS_SELFDRIVING_RESET:
             {
        	       //  processSelfDrivingRobot = PROCESS_SELFDRIVING_IDLE;
				   
             }
        	 break;
             case PROCESS_SELFDRIVING_PAUSE:
        	 break;
             case PROCESS_SELFDRIVING_ATREADY:
			 {
					if(batteryInfo.currentBatteryLevel<=batteryInfo.defautLowBattery)
					{
						processSelfDrivingRobot=PROCESS_SELFDRIVING_START_BATTERYCHARGED;
					}
			 }
             break;
			case PROCESS_SELFDRIVING_WAIT_GOTO_CHECKINDOCKING:
				if(flag_state_reachedgoal_checkindocking)
				{
						processSelfDrivingRobot =PROCESS_SELFDRIVING_FINISHED_GOTO_CHECKINDOCKING;
						flag_state_reachedgoal_checkindocking=false;
				}
				break;
			case PROCESS_SELFDRIVING_FINISHED_GOTO_CHECKINDOCKING:
				cout<<"PROCESS_SELFDRIVING_FINISHED_GOTO_CHECKINDOCKING\n";
				processSelfDrivingRobot =PROCESS_SELFDRIVING_IDLE;
				 pubFinishedStates(PROCESS_SELFDRIVING_FINISHED_GOTO_CHECKINDOCKING);
			break;
			case PROCESS_SELFDRIVING_WAIT_GOTO_CHECKINPUTAWAY:
				if(flag_state_reachedgoal_checkinputaway)
				{
						processSelfDrivingRobot =PROCESS_SELFDRIVING_FINISHED_GOTO_CHECKINPUTAWAY;
						flag_state_reachedgoal_checkinputaway=false;
				}
				break;
			break;
			case PROCESS_SELFDRIVING_FINISHED_GOTO_CHECKINPUTAWAY:
			cout<<"PROCESS_SELFDRIVING_FINISHED_GOTO_CHECKINPUTAWAY\n";
				processSelfDrivingRobot =PROCESS_SELFDRIVING_IDLE;
				pubFinishedStates(PROCESS_SELFDRIVING_FINISHED_GOTO_CHECKINPUTAWAY);
				
			break;
            case PROCESS_SELFDRIVING_START_GOTO_DOCKINGLINE:
			{
                	currentgoal_x=robotOrdered.posDocking.X;
                	currentgoal_y=robotOrdered.posDocking.Y;
                    cout<<".PROCESS_SELFDRIVING_START_GOTO_DOCKINGLINE\n";
                    cout<<	currentgoal_x<<"\n";
                    cout<<	currentgoal_y<<"\n";
                    cout<<	"state = "<< flag_state_reachedgoal_palletup<<"\n";
		            MoveBaseSimple_Goal(robotOrdered.posDocking.X,robotOrdered.posDocking.Y,robotOrdered.posDocking.Angle);
                    processSelfDrivingRobot = PROCESS_SELFDRIVING_WAIT_GOTO_DOCKINGLINE;
                    flag_state_reachedgoal_palletup=false;
					
					cout<<"PROCESS_SELFDRIVING_WAIT_GOTO_DOCKINGLINE\n";

			}
            break;
            case PROCESS_SELFDRIVING_WAIT_GOTO_DOCKINGLINE:
			{
					if(flag_state_reachedgoal_palletup)
					{
					cout<<"PROCESS_SELFDRIVING_WAIT_GOTO_DOCKINGLINE\n";
				
					processSelfDrivingRobot = PROCESS_SELFDRIVING_FINISHED_GOTO_DOCKINGLINE;
					flag_state_reachedgoal_palletup=false;
					}
				
			}
            break;
            case PROCESS_SELFDRIVING_FINISHED_GOTO_DOCKINGLINE:
            {
            	cout<<"PROCESS_SELFDRIVING_FINISHED_GOTO_DOCKINGLINE\n";
				if(!flag_state_error_detectline)
				{
					pubFinishedStates(PROCESS_SELFDRIVING_FINISHED_GOTO_DOCKINGLINE);
					processSelfDrivingRobot =PROCESS_SELFDRIVING_IDLE;
				}
				else
				{
					flag_state_error_detectline=false;
					processSelfDrivingRobot=PROCESS_SELFDRIVING_START_PALLETUP;
				}
				for(int i=0;i<10;i++)
				{
						posPalletCtrl(4000);
				}
            }
		    break;
			
			case PROCESS_SELFDRIVING_ERRORIN_PALLETUP:
				cout<<"PROCESS_SELFDRIVING_ERRORIN_PALLETUP\n";
				processSelfDrivingRobot=PROCESS_SELFDRIVING_START_GOTO_DOCKINGLINE;
				flag_state_error_detectline=true;
				break;
            case PROCESS_SELFDRIVING_START_PALLETUP:
                {
					linedetectionctrl(1203);
				flag_state_error_detectline=false;
                    cout<<"PROCESS_SELFDRIVING_START_PALLETUP\n";
                    processSelfDrivingRobot =PROCESS_SELFDRIVING_WAIT_PALLETUP;
                    cout<<"PROCESS_SELFDRIVING_WAIT_PALLETUP\n";
					cout<<"(amclpose_posX"<<amclpose_posX<<" / "<<robotOrdered.limitedPosDocking.Xmax;
					cout<<"(amclpose_posX"<<amclpose_posY<<" / "<<robotOrdered.limitedPosDocking.Xmin;
					countpos=0;
                }
                    break;
                case PROCESS_SELFDRIVING_WAIT_PALLETUP:
                	{
						pubFinishedStates(PROCESS_SELFDRIVING_WAIT_PALLETUP);
						if(countpos++>30)
						{
							cout<<"Ä‘ang tim  : "<<amclpose_posX<<" / "<<robotOrdered.limitedPosDocking.Xmax<<" / "<< robotOrdered.limitedPosDocking.Xmin<<"\n";
							countpos=0;
						}
						if(amclpose_posX<=robotOrdered.limitedPosDocking.Xmax)
						{
							posPalletCtrl(1205);
						}		
                	}
                    break;
                case PROCESS_SELFDRIVING_FINISH_PALLETUP:
				{
                    cout<<"PROCESS_SELFDRIVING_FINISH_PALLETUP\n";
					pubFinishedStates(PROCESS_SELFDRIVING_FINISH_PALLETUP);
					cntErrorDetectLine=0;
					for(int i=0;i<10;i++)
					{
						posPalletCtrl(4000);
					}
					processSelfDrivingRobot =PROCESS_SELFDRIVING_IDLE;
				}
                break;
					
                case PROCESS_SELFDRIVING_START_GOTO_PUTAWAYLINE:
					{
                	currentgoal_x=	robotOrdered.posPutAway.X;
                	currentgoal_y=	robotOrdered.posPutAway.Y;
					MoveBaseSimple_Goal(robotOrdered.posPutAway.X,	robotOrdered.posPutAway.Y,	robotOrdered.posPutAway.Angle);
                    cout<<"PROCESS_SELFDRIVING_START_GOTO_PUTAWAYLINE\n";

                    processSelfDrivingRobot = PROCESS_SELFDRIVING_WAIT_GOTO_PUTAWAYLINE;
                    cout<<"PROCESS_SELFDRIVING_WAIT_GOTO_PUTAWAYLINE\n";
                    flag_state_reachedgoal_palletdown=false;


					}
                    break;
                case PROCESS_SELFDRIVING_WAIT_GOTO_PUTAWAYLINE:
                    if(flag_state_reachedgoal_palletdown)
                    {
                    		processSelfDrivingRobot = PROCESS_SELFDRIVING_FINISHED_GOTO_PUTAWAYLINE;
                    		flag_state_reachedgoal_palletdown=false;
                    }
                    // waiting 
                    break;
                case PROCESS_SELFDRIVING_FINISHED_GOTO_PUTAWAYLINE:
                    cout<<"PROCESS_SELFDRIVING_FINISHED_GOTO_PUTAWAYLINE\n";
					if(!flag_state_error_detectline)
					{
						pubFinishedStates(PROCESS_SELFDRIVING_FINISHED_GOTO_PUTAWAYLINE);
						processSelfDrivingRobot =PROCESS_SELFDRIVING_IDLE;
					}
					else
					{
						flag_state_error_detectline=false;
						processSelfDrivingRobot = PROCESS_SELFDRIVING_START_PALLETDOWN;
					}

					for(int i=0;i<10;i++)
					{
						posPalletCtrl(4000);
					}
                    break;
				case PROCESS_SELFDRIVING_ERRORIN_PALLETDOWN:
					cout<<"PROCESS_SELFDRIVING_ERRORIN_PALLETDOWN\n";
					processSelfDrivingRobot=PROCESS_SELFDRIVING_START_GOTO_PUTAWAYLINE;
					flag_state_error_detectline=true;
				break;
                case PROCESS_SELFDRIVING_START_PALLETDOWN:
                {
                    cout<<"PROCESS_SELFDRIVING_START_DOWN\n";
                   linedetectionctrl(1204);
				   flag_state_error_detectline=false;
                    processSelfDrivingRobot =PROCESS_SELFDRIVING_WAIT_PALLETDOWN;
                    cout<<"PROCESS_SELFDRIVING_WAIT_DOWN\n";
					countpos=0;
                }
                    break;
                case PROCESS_SELFDRIVING_WAIT_PALLETDOWN:
                	{
						pubFinishedStates(PROCESS_SELFDRIVING_WAIT_PALLETDOWN);
						//if(amclpose_posX>=	robotOrdered.limitedPosPutAway.Xmin && amclpose_posX<robotOrdered.limitedPosPutAway.Xmax)
						if(countpos++>30)
						{
							cout<<"Ä‘ang tim  : "<<amclpose_posX<<" / "<<robotOrdered.limitedPosPutAway.Xmax<<" / "<< robotOrdered.limitedPosPutAway.Xmin<<"\n";
							countpos=0;
						}
						if(amclpose_posX>=	robotOrdered.limitedPosPutAway.Xmin)
						{
							 posPalletCtrl(1205);
						
						}
                	}
                    break;
                case PROCESS_SELFDRIVING_FINISH_PALLETDOWN:
                {
					pubFinishedStates(PROCESS_SELFDRIVING_FINISH_PALLETDOWN);
					for(int i=0;i<10;i++)
					{
						posPalletCtrl(4000);
					}
					processSelfDrivingRobot =PROCESS_SELFDRIVING_IDLE;
                }
                    break;
				case PROCESS_SELFDRIVING_BEGIN_GOTOLINE_CHARGESTATION:
					
				cout<<"PROCESS_SELFDRIVING_BEGIN_GOTOLINE_CHARGESTATION\n";
					 processSelfDrivingRobot =PROCESS_SELFDRIVING_GOTOLINE_CHARGINGSTATION;
					break;
				case PROCESS_SELFDRIVING_GOTOLINE_CHARGINGSTATION:
					{
						currentgoal_x=robotAtReady.ready.X;
						currentgoal_y=robotAtReady.ready.Y;
						MoveBaseSimple_Goal(robotAtReady.ready.X,robotAtReady.ready.Y,robotAtReady.ready.Angle);
						processSelfDrivingRobot = PROCESS_SELFDRIVING_WAIT_GOTOLINE_CHARGINGSTATION;
						cout<<"PROCESS_SELFDRIVING_GOTOLINE_CHARGINGSTATION\n";
					}
					break;
				case PROCESS_SELFDRIVING_WAIT_GOTOLINE_CHARGINGSTATION:
				cout<<"PROCESS_SELFDRIVING_WAIT_GOTOLINE_CHARGINGSTATION\n";
					if(batteryInfo.flagRobotGoToChargeStation==true)
					{
						processSelfDrivingRobot = PROCESS_SELFDRIVING_FINISH_GOTOLINE_CHARGINGSTATION;
						robotstatus=ROBOT_STATUS_FINISH;
                    	batteryInfo.flagRobotGoToChargeStation=false;
					}
					break;
				case PROCESS_SELFDRIVING_FINISH_GOTOLINE_CHARGINGSTATION:
				cout<<"PROCESS_SELFDRIVING_FINISH_GOTOLINE_CHARGINGSTATION\n";
				{
					cout<<"ROBOT STATUS"<<robotstatus<<"\n";
					if(!flag_state_error_detectline)
					{
						pubFinishedStates(PROCESS_SELFDRIVING_FINISH_GOTOLINE_CHARGINGSTATION);
						processSelfDrivingRobot =PROCESS_SELFDRIVING_IDLE;
					}
					else
					{
						flag_state_error_detectline=false;
						processSelfDrivingRobot=PROCESS_SELFDRIVING_DETECTLINE_TO_CHARGINGSTATION;
					}
					
				}
					break;
				case PROCESS_SELFDRIVING_ERRORIN_DETECTLINE_TO_CHARGINGSTATION:
						cout<<"PROCESS_SELFDRIVING_ERRORIN_DETECTLINE_TO_CHARGINGSTATION\n";
						processSelfDrivingRobot=PROCESS_SELFDRIVING_GOTOLINE_CHARGINGSTATION;
						flag_state_error_detectline=true;
					break;
				case PROCESS_SELFDRIVING_DETECTLINE_TO_CHARGINGSTATION:
				{	
					linedetectionctrl(1208);
					flag_state_error_detectline=false;
					processSelfDrivingRobot=PROCESS_SELFDRIVING_WAIT_DETECTLINE_TO_CHARGINGSTATION;
				}
					break;
				case PROCESS_SELFDRIVING_WAIT_DETECTLINE_TO_CHARGINGSTATION:
				{
					cout<<"PROCESS_SELFDRIVING_WAIT_DETECTLINE_TO_CHARGINGSTATION\n";
					//if(amclpose_posX>=robotAtReady.limitedReadyArea.Xmin && amclpose_posX<robotAtReady.limitedReadyArea.Xmax)
					
					if(amclpose_posX>=robotAtReady.limitedReadyArea.Xmin)
					{
						processSelfDrivingRobot =PROCESS_SELFDRIVING_FINISH_DETECTLINE_TO_CHARGINGSTATION;
						posPalletCtrl(1205);
					}
				}
					break;
				case PROCESS_SELFDRIVING_FINISH_DETECTLINE_TO_CHARGINGSTATION:
					{
						cout<<"PROCESS_SELFDRIVING_FINISH_DETECTLINE_TO_CHARGINGSTATION\n";
						pubChargeIP(robotAtReady.limitedReadyArea.iPCharger);
						pubFinishedStates(PROCESS_SELFDRIVING_FINISH_DETECTLINE_TO_CHARGINGSTATION);
						processSelfDrivingRobot =PROCESS_SELFDRIVING_ATREADY;
						flag_state_atReady=true;
					}
					break;
				case PROCESS_SELFDRIVING_START_BATTERYCHARGED:
					linedetectionctrl(1206);
					pubFinishedStates(PROCESS_SELFDRIVING_START_BATTERYCHARGED);
					processSelfDrivingRobot =PROCESS_SELFDRIVING_WAIT_BATTERYCHARGING;
					break;
				case PROCESS_SELFDRIVING_WAIT_BATTERYCHARGING:
					break;
				case PROCESS_SELFDRIVING_FINISH_BATTERYCHARGING:
					pubFinishedStates(PROCESS_SELFDRIVING_FINISH_BATTERYCHARGING);
					break;
				case PROCESS_SELFDRIVING_CANCEL_BATTERYCHARGING:
					break;
            }
}
void MoveBaseSimple_Goal(double posx,double posy, double angle)
{
	geometry_msgs::PoseStamped pose;
	pose.header.frame_id = "map";
	pose.pose.position.x=posx;
	//currentgoal_x=FRAME_CMD[7];
	pose.pose.position.y=posy;
	//currentgoal_y=FRAME_CMD[8];
	pose.pose.position.z=0;
	double th=angle*PI/180;
	pose.pose.orientation.z=sin(th/2);
	pose.pose.orientation.w=cos(th/2);
	pub_navigation_setgoal.publish(pose);

}
