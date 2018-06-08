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

void servergotoPalletAreaCallBack(const std_msgs::Int32::ConstPtr& msg);
void LineDetection_callback(const std_msgs::Int32& msg);
void moveBaseStatus_Callback(const actionlib_msgs::GoalStatusArrayConstPtr& msg);
void navigationAmclPose_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
void currentgoal_Callback (const geometry_msgs::PoseStamped pose);
void odometry_callback(const nav_msgs::Odometry &odom);
void reachrobotdirection_callback(const std_msgs::String::ConstPtr& msg);
void errorDetectedLine_callback(const std_msgs::Int32& msg);
void process();
void serverRobotGotToDockingAreaCallBack(const std_msgs::String::ConstPtr& msg);
void serverRobotGotToPutAwayAreaCallBack(const std_msgs::String::ConstPtr& msg);
#define PI 3.14159265
ros::Publisher pub_navigation_setgoal;
ros::Publisher pub_posPallet;
ros::Publisher pub_linedetectionctrl;
ros::Publisher pub_numworkingtimes;
ros::Publisher pub_errorturningrobot;
ros::Publisher pub_batteryRequestDetectLine;
ros::Publisher pub_BatteryChargeResponseStatusSelfDriving;
ros::Publisher pub_ChargeBattery;

//ros::Publisher pub_PalletLocation;
const int PROCESS_SELFDRIVING_START_READY =50000;
const int PROCESS_SELFDRIVING_START_GOTO_DOCKINGLINE = 50001;
const int PROCESS_SELFDRIVING_WAIT_GOTO_DOCKINGLINE = 50002;
const int PROCESS_SELFDRIVING_FINISHED_GOTO_DOCKINGLINE = 50003;

const int PROCESS_SELFDRIVING_START_PALLETUP = 50004;
const int PROCESS_SELFDRIVING_ERRORIN_PALLETUP = 50005;
const int PROCESS_SELFDRIVING_WAIT_PALLETUP = 50006;
const int PROCESS_SELFDRIVING_FINISH_PALLETUP = 50007;

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


const int PROCESS_SELFDRIVING_START_BATTERYCHARGED = 5001211;
const int PROCESS_SELFDRIVING_WAIT_BATTERYCHARGING = 5001311;
const int PROCESS_SELFDRIVING_FINISH_BATTERYCHARGING  = 5001411;

const int PROCESS_SELFDRIVING_START_GOTO_STATION= 5000811;
const int PROCESS_SELFDRIVING_WAIT_GOTO_STATION= 5000911;
const int PROCESS_SELFDRIVING_FINISHED_GOTO_STATION= 5001011;

const int PROCESS_SELFDRIVING_RESET = 50015;
const int PROCESS_SELFDRIVING_STOP  = 50016;
const int PROCESS_SELFDRIVING_PAUSE  = 50017;
const int PROCESS_SELFDRIVING_IDLE   =50018;

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
	double angle;
};
struct BatteryInfo{
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
	double infPos;
	double sufPos;
};
struct PalletInfo
{
   Position startPalletUpPos;
   LimitedPosition limitedPalletUpPos;
   Position endPalletDownPos;
   LimitedPosition limitedPalletDownPos;
   Position stationPos;
};
// variable from parse Json
struct PalletInfo palletInfos[MAXWORKINGTIMES];
struct RobotOrdered
{
	Position posDocking;
	int posPalletDock;
	Position posPutAway;
	int posPalletPutAWay;
};
std::list<RobotOrdered> robotOrderedlist;

double posXStartPalletUp[100];
double posYStartPalletUp[100];
double posStartAngleDirection[100];
double pospallet[100];
double posXEndPalletDown[100];
double posYEndPalletDown[100];
double posEndAngleDirection[100];
double posXStation[100];
double posYStation[100];

double posXBatteryStation;
double posYBatteryStation;
double posAngleDirectionBatteryStation;

int looptime_progress=0;
int atPos=0;
bool flag_state_reachedgoal_palletup=false;
bool flag_state_reachedgoal_palletdown=false;
bool flag_state_reachedgoal_battery=false;

void selfdriving();
// Subcribe From Line Detection
// Subcribe From Navigation
// Subcribe From Manager
int main(int argc, char *argv[])
{
	ros::init(argc, argv,"selfdriving");
	ros::NodeHandle nh;
	//pub = nh.advertise<std_msgs::UInt8MultiArray>("callbridgeNode", 100);
	pub_navigation_setgoal=nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal",1000);
	pub_posPallet=nh.advertise<std_msgs::Int32>("pospallet",1000);
	//pub_PalletLocation=nh.advertise<std_msgs::Int32>("palletlocation",1000);
	pub_numworkingtimes=nh.advertise<std_msgs::Int32>("numworkingtimes",1000);
	pub_linedetectionctrl=nh.advertise<std_msgs::Int32>("linedetectionctrl", 1000);
	pub_errorturningrobot=nh.advertise<std_msgs::Int32>("errorturningrobot", 1000);
	pub_BatteryChargeResponseStatusSelfDriving=nh.advertise<std_msgs::Int32>("batteryselfdrivingcallback", 1000);
	pub_ChargeBattery=nh.advertise<std_msgs::String>("/chargeIP", 1000);
	
	
	ros::Subscriber sub_CurrentPos = nh.subscribe("servergotoPalletAreaCallBack",1000, servergotoPalletAreaCallBack);
	ros::Subscriber sub_ReachedRobotdirection= nh.subscribe("ReachedRobotdirection",1000, reachrobotdirection_callback);
	ros::Subscriber sub_errorDetectedLine= nh.subscribe("errorDetectedLine",1000, errorDetectedLine_callback);
	ros::Subscriber sub_fromLineDetection_Node= nh.subscribe("linedetectioncallback",1,LineDetection_callback);
	ros::Subscriber sub_amclpose = nh.subscribe("amcl_pose",1,navigationAmclPose_callback);
	ros::Subscriber sub_currentgoal= nh.subscribe("move_base_simple/goal",1,currentgoal_Callback);
	ros::Subscriber sub_odometry= nh.subscribe("odom",1,odometry_callback);
	ros::Subscriber sub_movebase_status= nh.subscribe("move_base/status",1,moveBaseStatus_Callback);
	ros::Subscriber sub_batteryrequest= nh.subscribe("BatteryChargeRequestSelfDriving",1000,BatteryChargeRequestSelfDriving_callback);
	
	ros::Subscriber sub_serverRobotGotToDockingArea = nh.subscribe("serverRobotGotToDockingAreaCallBack",1000,serverRobotGotToDockingAreaCallBack);
	ros::Subscriber sub_serverRobotGotToPutAwayArea = nh.subscribe("serverRobotGotToPutAwayAreaCallBack",1000,serverRobotGotToPutAwayAreaCallBack);
	
	
	ros::Rate loop_rate(10);
    ros::spinOnce();
    int nk=0;

	while(ros::ok())
	{
		/*if(nk++<50)
		{
			 requestedprogress_test();
		}*/
	    //cout<<"hello \n";	    
	    process();
	    //usleep(100);
	    ros::spinOnce();
		loop_rate.sleep();
	}
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
void BatteryChargeRequestSelfDriving_callback(const std_msgs::String::ConstPtr& msg)
{
	cout<<msg->data.c_str()<<"\n";
	Document document;
	document.Parse(msg->data.c_str());
	int cmd=document["cmd"].GetInt();
	if(cmd==batteryInfo.CommandRequest_GoTo_ChargeStation)
	{
		batteryInfo.chargeStationPos.X=document["BatteryStation"]["X"].GetDouble();
		batteryInfo.chargeStationPos.Y=document["BatteryStation"]["Y"].GetDouble();
		batteryInfo.chargeStationPos.angle=document["BatteryStation"]["Angle"].GetDouble();
		processSelfDrivingRobot = PROCESS_SELFDRIVING_BEGIN_GOTOLINE_CHARGESTATION;
	}
	else if(cmd==batteryInfo.CommandRequest_GoTo_Station)
	{
		batteryInfo.stationPos.X=document["Station"]["X"].GetDouble();
		batteryInfo.stationPos.Y=document["Station"]["Y"].GetDouble();
		batteryInfo.stationPos.angle=document["Station"]["Angle"].GetDouble();
		processSelfDrivingRobot = PROCESS_SELFDRIVING_FINISH_BATTERYCHARGING;
	}
}

void serverRobotGotToDockingAreaCallBack(const std_msgs::String::ConstPtr& msg)
{
}
void serverRobotGotToPutAwayAreaCallBack(const std_msgs::String::ConstPtr& msg)
{
}
void odometry_callback(const nav_msgs::Odometry &odom)
{
	current_Vx=odom.twist.twist.linear.x;
	current_Vy =odom.twist.twist.linear.y;
	current_W  =odom.twist.twist.angular.z;
}
void servergotoPalletAreaCallBack(const std_msgs::Int32::ConstPtr& msg)
{
	cout<<msg->data<<"\n";
	/*Document document;
	document.Parse(msg->data.c_str());
	looptime_progress=document["package"]["info"]["numofpallets"].GetInt();
	const Value& palletinfo=document["package"]["info"]["pallets"];
	atPos=0;
	for(rapidjson::SizeType index=0; index<palletinfo.Size();index++)
	{

		const Value& _palletinfo=palletinfo[index];
		posXStartPalletUp[index]=_palletinfo["posXStartPalletUp"].GetDouble();
		posYStartPalletUp[index]=_palletinfo["posYStartPalletUp"].GetDouble();
		posStartAngleDirection[index]=_palletinfo["posStartAngleDirection"].GetDouble();
		pospallet[index]=_palletinfo["pospallet"].GetInt();
		posXEndPalletDown[index]=_palletinfo["posXEndPalletDown"].GetDouble();
		posYEndPalletDown[index]=_palletinfo["posYEndPalletDown"].GetDouble();
		posEndAngleDirection[index]=_palletinfo["posEndAngleDirection"].GetDouble();
		posXStation[index]=_palletinfo["posXStation"].GetDouble();
		posYStation[index]=_palletinfo["posYStation"].GetDouble();
		cout <<"ok  "<< posXStartPalletUp[index]<<"\n";
		cout <<"ok  "<< posYStartPalletUp[index]<<"\n";
		cout <<"ok  "<< posStartAngleDirection[index]<<"\n";
		cout <<"ok  "<< pospallet[index]<<"\n";

		cout <<"ok  "<< posXEndPalletDown[index]<<"\n";
		cout <<"ok  "<< posYEndPalletDown[index]<<"\n";
		cout <<"ok  "<< posEndAngleDirection[index]<<"\n";
		cout <<"ok  "<< posXStation[index]<<"\n";
		cout <<"ok  "<< posYStation[index]<<"\n";

		cout <<"-----------------------------------"<<"\n";
	}
	//  processSelfDrivingRobot =PROCESS_SELFDRIVING_START_READY;
	flag_state_reachedgoal_palletup=false;
	flag_state_reachedgoal_palletdown=false;
	 //printf("i = %d\n", document["command"].GetInt());
	*/
	if(msg->data==100)
	{
		cout<<"AAAAAAAA \n";
		posXStartPalletUp[0]=-9.88;
		posYStartPalletUp[0]=-1.75;
		posStartAngleDirection[0]=180;
		pospallet[0]=10;
		posXEndPalletDown[0]=7.72;
		posYEndPalletDown[0]=-1.46;
		posEndAngleDirection[0]=0.0;
		posXStation[0]=7.48;
		posYStation[0]=-6.08;
		looptime_progress=1;
	}
	else if(msg->data==101)
	{
		cout<<"BBBBBBBB \n";
		posXStartPalletUp[0]=-9.88;
		posYStartPalletUp[0]=0.17;
		posStartAngleDirection[0]=180;
		pospallet[0]=10;
		posXEndPalletDown[0]=7.72;
		posYEndPalletDown[0]=0.45;
		posEndAngleDirection[0]=0.0;
		posXStation[0]=7.48;
		posYStation[0]=-6.08;
		looptime_progress=1;
	}
	processSelfDrivingRobot =PROCESS_SELFDRIVING_START_READY;
	flag_state_reachedgoal_palletup=false;
	flag_state_reachedgoal_palletdown=false;
}
void requestedprogress_test()
{
	atPos=0;
	looptime_progress=6;
	/*for(int index=0; index<10;index++)
	{
		posXStartPalletUp[index]=8.0;
		posYStartPalletUp[index]=-2.1;
		posStartAngleDirection[index]=0.0;
		if(index%2==0)
		{
			pospallet[index]=3;
		}
		else
		{
			pospallet[index]=10;
		}
		posXEndPalletDown[index]=-10.2;
		posYEndPalletDown[index]=-1.25;
		posEndAngleDirection[index]=0.0;
		posXStation[index]=0.0;
		posYStation[index]=0.0;
		
		
		cout <<"ok  "<< posXStartPalletUp[index]<<"\n";
		cout <<"ok  "<< posYStartPalletUp[index]<<"\n";
		cout <<"ok  "<< posStartAngleDirection[index]<<"\n";
		cout <<"ok  "<< pospallet[index]<<"\n";

	    cout <<"ok  "<< posXEndPalletDown[index]<<"\n";
		cout <<"ok  "<< posYEndPalletDown[index]<<"\n";
		cout <<"ok  "<< posEndAngleDirection[index]<<"\n";
		cout <<"ok  "<< posXStation[index]<<"\n";
		cout <<"ok  "<< posYStation[index]<<"\n";

		cout <<"-----------------------------------"<<"\n";


	}*/
	/*
	// pallet 0 A1
	posXStartPalletUp[0]=7.75;
	posYStartPalletUp[0]=-1.23;
	posStartAngleDirection[0]=150;
	pospallet[0]=3;
	posXEndPalletDown[0]=-10.21;
	posYEndPalletDown[0]=-1.8;
	posEndAngleDirection[0]=-30;
	posXStation[0]=0.0;
	posYStation[0]=0.0;
	
	// pallet 1
	posXStartPalletUp[1]=7.75;
	posYStartPalletUp[1]=-1.23;
	posStartAngleDirection[1]=150;
	pospallet[1]=10;
	posXEndPalletDown[1]=-10.21;
	posYEndPalletDown[1]=-1.8;
	posEndAngleDirection[1]=-30;
	posXStation[1]=0.0;
	posYStation[1]=0.0;
	
	// pallet 2
	posXStartPalletUp[2]=7.55;
	posYStartPalletUp[2]=0.1;
	posStartAngleDirection[2]=150;
	pospallet[2]=3;
	posXEndPalletDown[2]=-10.21;
	posYEndPalletDown[2]=0.72;
	posEndAngleDirection[2]=-30;
	posXStation[2]=0.0;
	posYStation[2]=0.0;
	
		// pallet 3
	posXStartPalletUp[3]=7.55;
	posYStartPalletUp[3]=0.1;
	posStartAngleDirection[2]=150;
	pospallet[3]=10;
	posXEndPalletDown[3]=-10.21;
	posYEndPalletDown[3]=0.72;
	posEndAngleDirection[3]=-30;
	posXStation[3]=0.0;
	posYStation[3]=0.0;

	
		// pallet 4
	posXStartPalletUp[4]=7.75;
	posYStartPalletUp[4]=2.71;
	posStartAngleDirection[4]=150;
	pospallet[4]=3;
	posXEndPalletDown[4]=-10.22;
	posYEndPalletDown[4]=2.54;
	posEndAngleDirection[4]=30;
	posXStation[4]=0.0;
	posYStation[4]=0.0;
		// pallet 5
	posXStartPalletUp[5]=7.75;
	posYStartPalletUp[5]=2.71;
	posStartAngleDirection[5]=150;
	pospallet[5]=10;
	posXEndPalletDown[5]=-10.22;
	posYEndPalletDown[5]=2.54;
	posEndAngleDirection[5]=30;
	posXStation[5]=0.0;
	posYStation[5]=0.0;
	*/
	
		// pallet 0 A1
	posXStartPalletUp[0]=7.7;
	posYStartPalletUp[0]=-1.6;
	posStartAngleDirection[0]=180;
	pospallet[0]=3;
	posXEndPalletDown[0]=-10.08;
	posYEndPalletDown[0]=-1.55;
	posEndAngleDirection[0]=0.0;
	posXStation[0]=0.0;
	posYStation[0]=0.0;
	
	// pallet 1 get it
	posXStartPalletUp[1]=7.7;
	posYStartPalletUp[1]=-1.6;
	posStartAngleDirection[1]=180;
	pospallet[1]=10;
	posXEndPalletDown[1]=-10.08;
	posYEndPalletDown[1]=-1.55;
	posEndAngleDirection[1]=0.0;
	posXStation[1]=0.0;
	posYStation[1]=0.0;
	
	// pallet 2 
	posXStartPalletUp[2]=7.55;
	posYStartPalletUp[2]=0.41;
	posStartAngleDirection[2]=180;
	pospallet[2]=3;
	posXEndPalletDown[2]=-9.89;
	posYEndPalletDown[2]=0.37;
	posEndAngleDirection[2]=0.0;
	posXStation[2]=0.0;
	posYStation[2]=0.0;
	
		// pallet 3 cuoi get it
	posXStartPalletUp[3]=7.55;
	posYStartPalletUp[3]=0.41;
	posStartAngleDirection[2]=180;
	pospallet[3]=10;
	posXEndPalletDown[3]=-9.89;
	posYEndPalletDown[3]=0.37;
	posEndAngleDirection[3]=0.0;
	posXStation[3]=0.0;
	posYStation[3]=0.0;

	
		// pallet 4
	posXStartPalletUp[4]=7.5;
	posYStartPalletUp[4]=2.4;
	posStartAngleDirection[4]=180;
	pospallet[4]=3;
	posXEndPalletDown[4]=-9.94;
	posYEndPalletDown[4]=2.25;
	posEndAngleDirection[4]=0.0;
	posXStation[4]=0.0;
	posYStation[4]=0.0;
		// pallet 5
	posXStartPalletUp[5]=7.5;
	posYStartPalletUp[5]=2.4;
	posStartAngleDirection[5]=0.0;
	pospallet[5]=10;
	posXEndPalletDown[5]=-9.94;
	posYEndPalletDown[5]=2.25;
	posEndAngleDirection[5]=0.0;
	posXStation[5]=0.0;
	posYStation[5]=0.0;
	
	
		
	processSelfDrivingRobot =PROCESS_SELFDRIVING_START_READY;
	flag_state_reachedgoal_palletup=false;
	flag_state_reachedgoal_palletdown=false;
	 //printf("i = %d\n", document["command"].GetInt());

}
void parseInformaion(char *inf)
{
}
void presentIndexofPallet()
{

}
void updatenewprocess()
{
}
void canceledprocess()
{
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
  //printf("size=%d\n",msg->status_list.size());
  if(!msg->status_list.empty() && msg->status_list.back().status == msg->status_list.back().SUCCEEDED){
  //  printf("we are at the goal!\n");
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
		if(flag_state_reachedgoal_palletup==false)
		{
			if(processSelfDrivingRobot ==PROCESS_SELFDRIVING_WAIT_GOTO_PUTAWAYLINE)
			{
					flag_state_reachedgoal_palletdown=true;
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
void process()
{
         switch (processSelfDrivingRobot)
            {
             case PROCESS_SELFDRIVING_STOP:
        	 break;
             case PROCESS_SELFDRIVING_RESET:
             {
        	         processSelfDrivingRobot = PROCESS_SELFDRIVING_IDLE;
             }
        	 break;
             case PROCESS_SELFDRIVING_PAUSE:
        	 break;
             case PROCESS_SELFDRIVING_START_READY:
             //  cout<<"PROCESS_SELFDRIVING_START_READY "<<looptime_progress << "\n";
		  
					if(looptime_progress>0 )
					{
						 cout<<"PROCESS_SELFDRIVING_HERE \n";
						// update start and end
						processSelfDrivingRobot = PROCESS_SELFDRIVING_START_GOTO_DOCKINGLINE;
						cntErrorDetectLine=0;
						looptime_progress--;


					}
					else
					{
						batteryInfo.chargeStationPos.X=posXStation[0];
						batteryInfo.chargeStationPos.Y=posYStation[0];
						batteryInfo.chargeStationPos.angle=180;
						processSelfDrivingRobot = PROCESS_SELFDRIVING_BEGIN_GOTOLINE_CHARGESTATION;
						looptime_progress=-1;
						atPos=0;
						/*int bb= atPos+1;
						if(bb>1)
						{
							cout<<"SELFDRIVING CHARGE\n" ;
							std_msgs::Int32 msgnum;
							msgnum.data=6; // command detect line and pallet up
							cout<<"Working Times : "<<atPos+1<<"\n";
							pub_numworkingtimes.publish(msgnum);
						}
						else
						{
							atPos=0;
							looptime_progress=6;
						}*/
					} 
                    break;
                case PROCESS_SELFDRIVING_START_GOTO_DOCKINGLINE:
                	currentgoal_x=posXStartPalletUp[atPos];
                	currentgoal_y=posYStartPalletUp[atPos];
                    cout<<".PROCESS_SELFDRIVING_START_GOTO_DOCKINGLINE\n";
                    cout<<	currentgoal_x<<"\n";
                    cout<<	currentgoal_y<<"\n";
                    cout<<	"state = "<< flag_state_reachedgoal_palletup<<"\n";
                    //double []loA={POINTA_X, POINTA_Y};
                   // sendLocation(loA, POINTA_Theta);
		            MoveBaseSimple_Goal(posXStartPalletUp[atPos],posYStartPalletUp[atPos],posStartAngleDirection[atPos]);
                    processSelfDrivingRobot = PROCESS_SELFDRIVING_WAIT_GOTO_DOCKINGLINE;
                    flag_state_reachedgoal_palletup=false;
					
                    cout<<"PROCESS_SELFDRIVING_WAIT_GOTO_DOCKINGLINE\n";
                    break;
                case PROCESS_SELFDRIVING_WAIT_GOTO_DOCKINGLINE:
                    //cout<<("PROCESS_SELFDRIVING_WAIT_GOTO_DOCKINGLINE
                    // waiting 
					{
					if(flag_state_reachedgoal_palletup)
					{
					processSelfDrivingRobot = PROCESS_SELFDRIVING_FINISHED_GOTO_DOCKINGLINE;
					flag_state_reachedgoal_palletup=false;
					}
					std_msgs::Int32 tmpp;
					tmpp.data=4000; 
                    pub_posPallet.publish(tmpp);
					}
                    break;
            case PROCESS_SELFDRIVING_FINISHED_GOTO_DOCKINGLINE:
            {
               // temp to test bateery
            	//processSelfDrivingRobot = PROCESS_SELFDRIVING_START_GOTO_PUTAWAYLINE;
               //---------------------------

            	 cout<<"PROCESS_SELFDRIVING_FINISHED_GOTO_DOCKINGLINE\n";
                processSelfDrivingRobot = PROCESS_SELFDRIVING_START_PALLETUP;
                std_msgs::Int32 msg;
	            msg.data=1203; // command detect line and pallet up
				cout<<"palletnum= "<<pospallet[atPos]<<"\n";
	            pub_linedetectionctrl.publish(msg);
	
				
				
	            //std_msgs::Int32 palletnum;
	           // palletnum.data=pospallet[atPos];
	       
	            //pub_posPalletup.publish(palletnum);
            }
		    break;
			case PROCESS_SELFDRIVING_ERRORIN_PALLETUP:
				cout<<"PROCESS_SELFDRIVING_ERRORIN_PALLETUP\n";
				processSelfDrivingRobot=PROCESS_SELFDRIVING_START_GOTO_DOCKINGLINE;
				break;
            case PROCESS_SELFDRIVING_START_PALLETUP:
                {

                    cout<<"PROCESS_SELFDRIVING_START_PALLETDOWN\n";
                    processSelfDrivingRobot =PROCESS_SELFDRIVING_WAIT_PALLETUP;
                    cout<<"PROCESS_SELFDRIVING_WAIT_PALLETUP\n";
                }
                    break;
                case PROCESS_SELFDRIVING_WAIT_PALLETUP:
                	//cout<<"PROCESS_SELFDRIVING_WAIT_PALLETUP\n"<<amclpose_posX<<"\n";
//###########################################################################
                	if(pospallet[atPos]==3)
                	{
						/*if(amclpose_posX>=11.2 && amclpose_posX<11.7 ) // slow motion
						{
							 std_msgs::Int32 palletnum;
							 palletnum.data=1205;
							 pub_posPallet.publish(palletnum);
						}
						else if(amclpose_posX>=11.7)
						{
							// std_msgs::Int32 palletnum;
							// palletnum.data=1206;
							 //pub_posPallet.publish(palletnum);
						}*/
						if(amclpose_posX>=11.5 && amclpose_posX<13 ) // slow motion
						{
							 std_msgs::Int32 palletnum;
							 palletnum.data=1205;
							 pub_posPallet.publish(palletnum);
						}
						else if(amclpose_posX>=11.7)
						{
							// std_msgs::Int32 palletnum;
							// palletnum.data=1206;
							 //pub_posPallet.publish(palletnum);
						}
						
                	}
                	if(pospallet[atPos]==10)
                	{
						/*if(amclpose_posX>=12.6 && amclpose_posX<13.1 ) // slow motion
						{
							 std_msgs::Int32 palletnum;
							 palletnum.data=1205;
							 pub_posPallet.publish(palletnum);
						}
						if(amclpose_posX>=13.1)
						{
							// std_msgs::Int32 palletnum;
							// palletnum.data=1206;
							// pub_posPallet.publish(palletnum);
						}*/
						
						if(amclpose_posX<=-13.5 && amclpose_posX>-13.8) // slow motion
						{
							 std_msgs::Int32 palletnum;
							 palletnum.data=1205;
							 pub_posPallet.publish(palletnum);
						}
						else if(amclpose_posX<=-15.3)
                		{
                		                       // std_msgs::Int32 palletnum;//
                		                      // palletnum.data=1206;
                		                      //  pub_posPallet.publish(palletnum);
                		}
						
						
                	}
//###########################################################################
                   //
                    // waiting 
                    break;
                case PROCESS_SELFDRIVING_FINISH_PALLETUP:
                    cout<<"PROCESS_SELFDRIVING_FINISH_PALLETUP\n";
                    processSelfDrivingRobot = PROCESS_SELFDRIVING_START_GOTO_PUTAWAYLINE;
					cntErrorDetectLine=0;
                    break;
                case PROCESS_SELFDRIVING_START_GOTO_PUTAWAYLINE:
					{
                    cout<<"PROCESS_SELFDRIVING_START_GOTO_PUTAWAYLINE\n";
                    currentgoal_x=posXEndPalletDown[atPos];
                    currentgoal_y=posYEndPalletDown[atPos];
                    MoveBaseSimple_Goal(posXEndPalletDown[atPos],posYEndPalletDown[atPos],posEndAngleDirection[atPos]);
                    processSelfDrivingRobot = PROCESS_SELFDRIVING_WAIT_GOTO_PUTAWAYLINE;
                    cout<<"PROCESS_SELFDRIVING_WAIT_GOTO_PUTAWAYLINE\n";
                    flag_state_reachedgoal_palletdown=false;

                    std_msgs::Int32 tmmp;
					tmmp.data=4000;
					pub_posPallet.publish(tmmp);
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
                	 // temp to test bateery
					 //atPos++;
					 
					  // processSelfDrivingRobot =processSelfDrivingRobot =PROCESS_SELFDRIVING_START_READY;
                	//  processSelfDrivingRobot =PROCESS_SELFDRIVING_START_GOTO_DOCKINGLINE;
                	 //---------------------------

                    cout<<"PROCESS_SELFDRIVING_FINISHED_GOTO_PUTAWAYLINE\n";
                       processSelfDrivingRobot =PROCESS_SELFDRIVING_START_PALLETDOWN;
                    break;
				case PROCESS_SELFDRIVING_ERRORIN_PALLETDOWN:
					cout<<"PROCESS_SELFDRIVING_ERRORIN_PALLETDOWN\n";
					processSelfDrivingRobot=PROCESS_SELFDRIVING_START_GOTO_PUTAWAYLINE;
				break;
                case PROCESS_SELFDRIVING_START_PALLETDOWN:
                {
                    cout<<"PROCESS_SELFDRIVING_START_DOWN\n";
                    std_msgs::Int32 msg;
    	            msg.data=1204; // command detect line and pallet up
                    pub_linedetectionctrl.publish(msg);
                    processSelfDrivingRobot =PROCESS_SELFDRIVING_WAIT_PALLETDOWN;
                    cout<<"PROCESS_SELFDRIVING_WAIT_DOWN\n";
                }
                    break;
                case PROCESS_SELFDRIVING_WAIT_PALLETDOWN:
                	//cout<<"PROCESS_SELFDRIVING_WAIT_DOWN\n"<<amclpose_posX<<"\n";
                	if(pospallet[atPos]==3)
                	{
						/*if(amclpose_posX<=-16.2 && amclpose_posX>-16.7) // slow motion
						{
							 std_msgs::Int32 palletnum;
							 palletnum.data=1205;
							 pub_posPallet.publish(palletnum);
						}
						else if(amclpose_posX<=-16.7)
                		{
                		      //std_msgs::Int32 palletnum;//
                		     // palletnum.data=1206;
                		    //  pub_posPallet.publish(palletnum);
                		}*/
						
						if(amclpose_posX<=-15.3 && amclpose_posX>-15.8) // slow motion
						{
							 std_msgs::Int32 palletnum;
							 palletnum.data=1205;
							 pub_posPallet.publish(palletnum);
						}
						else if(amclpose_posX<=-16.7)
                		{
                		      //std_msgs::Int32 palletnum;//
                		     // palletnum.data=1206;
                		    //  pub_posPallet.publish(palletnum);
                		}
                	}
                	if(pospallet[atPos]==10)
                	{
						/*if(amclpose_posX<=-14.8 && amclpose_posX>-15.3) // slow motion
						{
							 std_msgs::Int32 palletnum;
							 palletnum.data=1205;
							 pub_posPallet.publish(palletnum);
						}
						else if(amclpose_posX<=-15.3)
                		{
                		                       // std_msgs::Int32 palletnum;//
                		                      // palletnum.data=1206;
                		                      //  pub_posPallet.publish(palletnum);
                		}*/
						
						if(amclpose_posX>=13 && amclpose_posX<13.5) // slow motion
						{
							 std_msgs::Int32 palletnum;
							 palletnum.data=1205;
							 pub_posPallet.publish(palletnum);
						}
						if(amclpose_posX>=13.1)
						{
							// std_msgs::Int32 palletnum;
							// palletnum.data=1206;
							// pub_posPallet.publish(palletnum);
						}
                	}
                    break;
                case PROCESS_SELFDRIVING_FINISH_PALLETDOWN:
                {
                    cout<<"PROCESS_SELFDRIVING_FINISH_DOWN\n";
                    processSelfDrivingRobot =PROCESS_SELFDRIVING_START_READY;
					std_msgs::Int32 msgnum;
					msgnum.data=atPos+1; // command detect line and pallet up
					cout<<"Working Times : "<<atPos+1<<"\n";
					pub_numworkingtimes.publish(msgnum);
                    atPos++;
                }
                    break;
				case PROCESS_SELFDRIVING_BEGIN_GOTOLINE_CHARGESTATION:
				cout<<"PROCESS_SELFDRIVING_BEGIN_GOTOLINE_CHARGESTATION\n";
					 processSelfDrivingRobot =PROCESS_SELFDRIVING_GOTOLINE_CHARGINGSTATION;
					break;
				case PROCESS_SELFDRIVING_GOTOLINE_CHARGINGSTATION:
				cout<<"PROCESS_SELFDRIVING_GOTOLINE_CHARGINGSTATION\n";
				    currentgoal_x=batteryInfo.chargeStationPos.X;
                    currentgoal_y=batteryInfo.chargeStationPos.Y;
                    MoveBaseSimple_Goal(batteryInfo.chargeStationPos.X,batteryInfo.chargeStationPos.Y,batteryInfo.chargeStationPos.angle);
                    processSelfDrivingRobot = PROCESS_SELFDRIVING_WAIT_GOTOLINE_CHARGINGSTATION;
                    batteryInfo.flagRobotGoToChargeStation=false;
					break;
				case PROCESS_SELFDRIVING_WAIT_GOTOLINE_CHARGINGSTATION:
				cout<<"PROCESS_SELFDRIVING_WAIT_GOTOLINE_CHARGINGSTATION\n";
					if(batteryInfo.flagRobotGoToChargeStation==true)
					{
						processSelfDrivingRobot = PROCESS_SELFDRIVING_FINISH_GOTOLINE_CHARGINGSTATION;
                    	batteryInfo.flagRobotGoToChargeStation=false;
					}
					break;
				case PROCESS_SELFDRIVING_FINISH_GOTOLINE_CHARGINGSTATION:
				cout<<"PROCESS_SELFDRIVING_WAIT_GOTOLINE_CHARGINGSTATION\n";
				{

					//processSelfDrivingRobot = PROCESS_SELFDRIVING_DETECTLINE_TO_CHARGINGSTATION;
				}
					break;
				case PROCESS_SELFDRIVING_DETECTLINE_TO_CHARGINGSTATION:
				{	
					cout<<"PROCESS_SELFDRIVING_DETECTLINE_TO_CHARGINGSTATION\n";
					std_msgs::String msgBattery;
					std::stringstream ss;
					ss <<"192.168.0.10";		
					msgBattery.data=ss.str();		
					std_msgs::Int32 msg;
    	            msg.data=1206; // command detect line and pallet up
                    pub_linedetectionctrl.publish(msg);
					pub_ChargeBattery.publish( msgBattery);
                    processSelfDrivingRobot =PROCESS_SELFDRIVING_WAIT_DETECTLINE_TO_CHARGINGSTATION;
				}
					break;
				case PROCESS_SELFDRIVING_WAIT_DETECTLINE_TO_CHARGINGSTATION:
					cout<<"PROCESS_SELFDRIVING_WAIT_DETECTLINE_TO_CHARGINGSTATION\n";
					/*if(batteryInfo.flagRobotGoToChargeStation==true)
					{
						//processSelfDrivingRobot = PROCESS_SELFDRIVING_FINISH_DETECTLINE_TO_CHARGINGSTATION;
						processSelfDrivingRobot = PROCESS_SELFDRIVING_FINISH_RETURN_THELINNE;
						batteryInfo.flagRobotGoToChargeStation==false;
					}*/
					break;
				/*case PROCESS_SELFDRIVING_FINISH_DETECTLINE_TO_CHARGINGSTATION:
					{
							cout<<"PROCESS_SELFDRIVING_FINISH_DETECTLINE_TO_CHARGINGSTATION\n";
					std_msgs::Int32 msgnum;
					msgnum.data=batteryInfo.SendRequest_Finish_GoTo_ChargeStation; 
					pub_BatteryChargeResponseStatusSelfDriving.publish(msgnum);
					processSelfDrivingRobot = PROCESS_SELFDRIVING_WAIT_BATTERYCHARGING;
					}
					break;
				case PROCESS_SELFDRIVING_WAIT_BATTERYCHARGING:
					cout<<"PROCESS_SELFDRIVING_WAIT_BATTERYCHARGING\n";
					break;
				case PROCESS_SELFDRIVING_FINISH_BATTERYCHARGING:
				{
					cout<<"PROCESS_SELFDRIVING_WAIT_BATTERYCHARGING\n";
					std_msgs::Int32 msg;
    	            msg.data=1207; // command detect line and pallet up
                    pub_linedetectionctrl.publish(msg);
                    processSelfDrivingRobot =PROCESS_SELFDRIVING_WAITING_RETURN_THELINNE;
				}
					break;
				case PROCESS_SELFDRIVING_WAITING_RETURN_THELINNE:		
					break;*/
				case PROCESS_SELFDRIVING_FINISH_RETURN_THELINNE:
					//cout<<"PROCESS_SELFDRIVING_FINISH_RETURN_THELINNE\n";
					processSelfDrivingRobot=PROCESS_SELFDRIVING_IDLE;
					//atPos=0;
					//looptime_progress=6;
					break;
					//processSelfDrivingRobot = PROCESS_SELFDRIVING_WAIT_BATTERYCHARGING;
			/*	case PROCESS_SELFDRIVING_START_GOTO_STATION:
					currentgoal_x=batteryInfo.stationPos.X;
                    currentgoal_y=batteryInfo.stationPos.Y;
                    MoveBaseSimple_Goal(batteryInfo.stationPos.X,batteryInfo.stationPos.Y,batteryInfo.stationPos.angle);
                    processSelfDrivingRobot = PROCESS_SELFDRIVING_WAIT_GOTOLINE_CHARGINGSTATION;
                    cout<<"PROCESS_SELFDRIVING_WAIT_GOTO_PUTAWAYLINE\n";
                    batteryInfo.flagRobotGoToChargeStation=false;
					break;
				case PROCESS_SELFDRIVING_WAIT_GOTO_STATION:
					break;
				case PROCESS_SELFDRIVING_FINISHED_GOTO_STATION:
				{
					std_msgs::Int32 msgnum;
					msgnum.data=batteryInfo.SendRequest_Finish_GoTo_Station; 
					pub_BatteryChargeResponseStatusSelfDriving.publish(msgnum);
					processSelfDrivingRobot = PROCESS_SELFDRIVING_WAIT_BATTERYCHARGING;
				}
					break;*/
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

