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
#include <json/json.h>
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

#include <std_msgs/UInt8MultiArray.h>
#include <turtlesim/Pose.h>

#include <sstream>
#include <math.h>
#include <pthread.h>
#define PI 3.14159265
#define ROBOTVERSION 2018002
// Script
//#define scriptBridenode "rosrun odo self_driving&"
//#define scriptBridenode "rosrun seldat_robot bridgenode &"
//function
void MergeCmdData(const unsigned char *Arr,int length);
void MoveBaseSimple_Goal();
void batteryhardware_callback(const std_msgs::Int32& msg);
void batteryselfdriving_callback(const std_msgs::Int32& msg);
void serverCtrlcallback(const std_msgs::String msg);
void chargerCtrlCallBack(const std_msgs::String msg);

void serverRobotInfoCallBack(const std_msgs::String msg);
void serverRobotParamsCallBack(const std_msgs::String msg);
void serverCtrlRobotHardwareCallBack(const std_msgs::String msg);
void serverEstimateRobotLocationCallBack(const std_msgs::String msg);
void serverDriveRobotCallBack(const std_msgs::String msg);
void serverBatteryRegisterCallBack(const std_msgs::String msg);
void serverEmergencyRobotCallBack(const std_msgs::String msg);

void controlRobotHardware(int cmd);
void *task_Battery (void *dummyPt);

std::string CreateDataJson();
using namespace std;
using namespace rapidjson;
int listenfd = 0, connfd = 0;
struct sockaddr_in serv_addr, client;
char sendBuff[1025];
double amclpose_posX=10.0,amclpose_posY=10.0,amclpose_posthetaW=0.0,amclpose_posthetaZ=0.0;
double Vel_posX=0;
double W_theta=0;
bool flag_connected=true;
bool flag_reachedgoal=false;
bool flag_state=false;
bool flag_datapathfilder=false;
int numworkingtime_sub=0;
int countfullcharger =0;
std_msgs::Float32MultiArray arrayfinder;

// COMMAND
double FRAME_CMD[256];
//Publishes
ros::Publisher pub_responseInf;
ros::Publisher pub_manualctrl;
ros::Publisher pub_linedetectionctrl;
ros::Publisher pub_navigation_Initpose;
ros::Publisher pub_navigation_setgoal;
ros::Publisher pub_getsensordata;
ros::Publisher pub_selfdriving;
ros::Publisher pub_RequestCmdServer;
ros::Publisher pub_BatteryChargeRequestServer;
ros::Publisher pub_ctrlRobotHardware;
ros::Publisher pub_BatteryChargeRequestSelfDriving;
ros::Publisher pub_peripheralManagerToCharger;

ros::Publisher pub_robotInfoResponse;
ros::Publisher pub_robotParamsResponse;
ros::Publisher pub_ctrlRobotHardwareStatus;
ros::Publisher pub_EstimateRobotLocationStatus;
ros::Publisher pub_serverDriveRobotStatus;
ros::Publisher pub_batteryRegisterResponse;
ros::Publisher pub_emergencyRobotResponse;

struct RobotInfo{
	const int version;
};

struct BatteryInfo {
  
  const int SendReq_hardware_poweroff=5000;
  const int SendReq_hardware_poweron=5001;
  const int RecvResp_hardware_poweredoff=5101;
  const int RecvResp_hardware_poweredon=5100;
  double currentBatteryLevel=0.0;
  double defautLowBattery=29.0;	
  bool flagResponseHardwareCallback=false;
  std_msgs::Int32 msgResponseHardwareCallback;
  bool flagChargerResponse=false;
  bool flagChargeFinished=false;
  bool flagServerResponse=false;
  std_msgs::String msgServerResponse;
  bool flagSelfDrivingCallback=false;
  std_msgs::Int32 msgSelfDrivingCallback;
  
} ;
BatteryInfo batteryInfo;
// STATE MANAGER
#define MANAGER_STATE_INIT_MANAGERNODE          01
#define MANAGER_STATE_INIT_PUBLISHES            02
#define MANAGER_STATE_INIT_SUBCRIBES            03
#define MANAGER_STATE_INIT_SERVER               04
#define MANAGER_STATE_SERVER_CONNECTED          04
#define MANAGER_STATE_READY                     04
#define MANAGER_STATE_ERROR                     05

#define MANAGER_STATE_BATTERY_NORMALBATTERY             06
#define MANAGER_STATE_BATTERY_LOWBATTERY                07
#define MANAGER_STATE_BATTERY_REQUESTSERVER_BATTERYSTATION  0733
#define MANAGER_STATE_BATTERY_WAITINGRESPONSESERVER_BATTERYSTATION  734
#define MANAGER_STATE_BATTERY_REQUEST_GOTO_CHARGESTATION  833
#define MANAGER_STATE_BATTERY_WAITING_GOTO_CHARGESTATION  989
#define MANAGER_STATE_BATTERY_FINISH_GOTO_CHARGESTATION  8234
#define MANAGER_STATE_BATTERY_REQUEST_GOTO_STATION  834
#define MANAGER_STATE_BATTERY_WAITING_GOTO_STATION  9891
#define MANAGER_STATE_BATTERY_FINISH_GOTO_STATION  812
#define MANAGER_STATE_BATTERY_REQUEST_START_CHARGE  9
#define MANAGER_STATE_BATTERY_REQUEST_FINISH_CHARGE 10
#define MANAGER_STATE_BATTERY_REQUEST_POWEROFF_HARWDWARE 897
#define MANAGER_STATE_BATTERY_WAITING_POWEROFF_HARWDWARE 89723
#define MANAGER_STATE_BATTERY_FINISH_POWEROFF_HARWDWARE 899
#define MANAGER_STATE_BATTERY_WAITINGRESPONSESERVER_CHARGING 765
#define MANAGER_STATE_BATTERY_REQUEST_POWERON_HARWDWARE 8975
#define MANAGER_STATE_BATTERY_WAITING_POWERON_HARWDWARE 8977
#define MANAGER_STATE_BATTERY_FINISH_POWERON_HARWDWARE 8996
#define MANAGER_STATE__REQUEST_CHARGER_START_CHARGE 776
#define MANAGER_STATE_BATTERY_START_CHARGE 778
#define MANAGER_STATE_BATTERY_FINISH_CHARGE 7781
#define MANAGER_STATE_BATTERY_WAITING_CHARGE 6565

// STATES THAT CLIENT REQUESTED TO MANAGER
#define SERVER_CALLBACK_CMD_MANUALCTRL_STOP         1100
#define SERVER_CALLBACK_CMD_MANUALCTRL_FORWARD      1101
#define SERVER_CALLBACK_CMD_MANUALCTRL_BACKWARDWARD 1102
#define SERVER_CALLBACK_CMD_MANUALCTRL_RESET        1103
#define SERVER_CALLBACK_CMD_MANUALCTRL_ROTATIONCW   1104  
#define SERVER_CALLBACK_CMD_MANUALCTRL_ROTATIONCCW  1105

#define SERVER_CALLBACK_CMD_LINEDETECTION_STOP      1200
#define SERVER_CALLBACK_CMD_LINEDETECTION_RUN       1201
#define SERVER_CALLBACK_CMD_LINEDETECTION_UPPALLET        1203
#define SERVER_CALLBACK_CMD_LINEDETECTION_DOWNPALLET      1204

#define SERVER_CALLBACK_CMD_LINEDETECTION_DETECTRF  1205
#define SERVER_CALLBACK_CMD_LINEDETECTION_OUTLINE   1206
#define SERVER_CALLBACK_CMD_LINEDETECTION_DETECTPALLET  1207


#define SERVER_CALLBACK_CMD_NAVIGATION_STOP         1300
#define SERVER_CALLBACK_CMD_NAVIGATION_START        1301
#define SERVER_CALLBACK_CMD_NAVIGATION_SETGOALPOS   1302
#define SERVER_CALLBACK_CMD_NAVIGATION_ESTIMPOS     1303
#define SERVER_CALLBACK_CMD_NAVIGATION_AMCLPOS_INF  1304
#define SERVER_CALLBACK_CMD_NAVIGATION_AMCLPOS_FIN  1305
#define SERVER_CALLBACK_CMD_NAVIGATION_SELFDRIVING  1306

#define SERVER_CALLBACK_CMD_CHARGER  1307
#define SERVER_CALLBACK_CMD_POWERON  1308
#define SERVER_CALLBACK_CMD_POWEROFF 1309
#define SERVER_CALLBACK_ROBOTVERSION 1408




#define SERVER_CALLBACK_CMD_PUBLISH_CURRENTTOPICSLIST 1400
#define SERVER_CALLBACK_CMD_PUBLISH_ALLINFORMS        1401
#define SERVER_CALLBACK_CMD_PUBLISH_SENSORS           1402

#define SERVER_CALLBACK_CMD_STATE_WAITREADY           1500
#define SERVER_CALLBACK_CMD_STATE_ERROR               1501

static int SERVER_CALLBACK_CMD_INSTATE=SERVER_CALLBACK_CMD_STATE_WAITREADY;
static int batterystate= MANAGER_STATE_BATTERY_NORMALBATTERY  ;

// STATES THAT CLIENT IS RESPONDED TO MANAGER

#define SERVER_RESPONSED_MANUALCTRL_STOP         2100
#define SERVER_RESPONSED_MANUALCTRL_FORWARD      2101
#define SERVER_RESPONSED_MANUALCTRL_BACKWARDWARD 2102
#define SERVER_RESPONSED_MANUALCTRL_RESET        2103
#define SERVER_RESPONSED_MANUALCTRL_ROTATIONCW   2104  
#define SERVER_RESPONSED_MANUALCTRL_ROTATIONCCW  2105

#define SERVER_RESPONSED_LINEDETECTION_STOP      2200
#define SERVER_RESPONSED_LINEDETECTION_RUN       2201

#define SERVER_RESPONSED_NAVIGATION_STOP         2300
#define SERVER_RESPONSED_NAVIGATION_START        2301
#define SERVER_RESPONSED_NAVIGATION_SETGOALPOS   2302
#define SERVER_RESPONSED_NAVIGATION_ESTIMPOS     2303
#define SERVER_RESPONSED_NAVIGATION_AMCLPOS_INF  2304
#define SERVER_RESPONSED_NAVIGATION_AMCLPOS_FIN  2305


#define SERVER_RESPONSED_PUBLISH_CURRENTTOPICSLIST 2400
#define SERVER_RESPONSED_PUBLISH_ALLINFORMS        2401
#define SERVER_RESPONSED_PUBLISH_SENSORS           2402


#define PROCESS_WORKING                         9000
#define PROCESS_DETECTIONLINE_FINISED_UP        9001
#define PROCESS_DETECTIONLINE_FINISED_DOWN      9002
#define PROCESS_NAVIGATION_REACHEDGOAL          9003


static int STATE_PROCESS= PROCESS_WORKING;

double current_Vx=0.0;
double current_Vy=0.0;
double current_W=0.0;
double currentgoal_x=0.0;
double currentgoal_y=0.0;
double currentgoal_z=0.0;
double currentgoal_w=0.0;

std_msgs::String msgJson;

void pathfinder_callback(const std_msgs::Float32MultiArray::ConstPtr& array)
{

	int i = 0;
	// print all the remaining numbers
	for(std::vector<float>::const_iterator it = array->data.begin(); it != array->data.end(); ++it)
	{
		//Arr[i] = *it;

		arrayfinder.data.push_back((float)*it);
		//i++;
	}
	flag_datapathfilder=true;
	return;
}

void Odometry_callback(const nav_msgs::Odometry &odom)
{
	current_Vx=odom.twist.twist.linear.x;
	current_Vy =odom.twist.twist.linear.y;
	current_W  =odom.twist.twist.angular.z;
}
void NavigationAmclPose_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg){
	amclpose_posX=msg->pose.pose.position.x;
	amclpose_posY=msg->pose.pose.position.y;
	amclpose_posthetaW=msg->pose.pose.orientation.w;
	amclpose_posthetaZ=msg->pose.pose.orientation.z;
}
void moveBaseStatus_Callback(const actionlib_msgs::GoalStatusArrayConstPtr& msg){
  //printf("size=%d\n",msg->status_list.size());
  if(!msg->status_list.empty() && msg->status_list.back().status == msg->status_list.back().SUCCEEDED){
    //printf("we are at the goal!\n");
	double _currentgoal_Ex = fabs(fabs(amclpose_posX)-fabs(currentgoal_x));
	double _currentgoal_Ey = fabs(fabs(amclpose_posY)-fabs(currentgoal_y));
	double _currentgoal_Ez = fabs(fabs(amclpose_posthetaZ)-fabs(currentgoal_z));
	double _currentgoal_Ew = fabs(fabs(amclpose_posthetaW)-fabs(currentgoal_w));
	//printf(" %0.2f  / %0.2f / %0.2f / %0.2f \n",_currentgoal_Ex,_currentgoal_Ey,_currentgoal_Ez,_currentgoal_Ew);
    if(fabs(current_Vx)<0.0001 && fabs(current_Vy)<0.0001 && fabs(current_W)<0.0001 && _currentgoal_Ex<=0.8 && _currentgoal_Ey<=0.8)
	{
		STATE_PROCESS=PROCESS_NAVIGATION_REACHEDGOAL;
		flag_state=true;
	}	

  }
}
void batteryhardware_callback(const std_msgs::Int32& msg)
{
	batteryInfo.flagResponseHardwareCallback=true;
}
void batteryselfdriving_callback(const std_msgs::Int32& msg)
{
	batteryInfo.flagSelfDrivingCallback=true;
}
void batterysub_callback (const std_msgs::Float32::ConstPtr& msg)
{
	batteryInfo.currentBatteryLevel=(double)msg->data;
}
void NumWorkingTimes_callback(const std_msgs::Int32& msg){
	numworkingtime_sub=msg.data;
}
void LineDetection_callback(const std_msgs::Int32& msg){
    if((int)msg.data==3203)
    	STATE_PROCESS=PROCESS_DETECTIONLINE_FINISED_UP;
    if((int)msg.data==3204) 
    	STATE_PROCESS=PROCESS_DETECTIONLINE_FINISED_DOWN;
    flag_state=true;
}
void chargerCtrlCallBack(const std_msgs::String msg)
{
	Document document;
	cout << "\n"<<msg.data<<"\n";	
	if (false == document.Parse <0>(msg.data.c_str()).HasParseError())
	{
		int status=document["status"].GetInt();
		if(status==7005 || status==7006)
		{
			batteryInfo.flagChargeFinished=true;

		}
		//batteryInfo.flagChargerResponse=true;
	 }else{
	 // error
	 cout<<"error Json charger \n";
	}
	
}
void SensorData_callback(const std_msgs::String& msg){

}
const char* CreateJSON_AMCL(int amclmode)
{
  json_object * jobj_amcl = json_object_new_object();
  json_object * jobj_curpos = json_object_new_object();
  json_object *jdouble_curpos_X = json_object_new_double(amclpose_posX);
  json_object *jdouble_curpos_Y = json_object_new_double(amclpose_posY);
  json_object *jdouble_curpos_Z = json_object_new_double(amclpose_posthetaZ);
  json_object *jdouble_curpos_W = json_object_new_double(amclpose_posthetaW);
  json_object_object_add(jobj_curpos,"X",jdouble_curpos_X);
  json_object_object_add(jobj_curpos,"Y",jdouble_curpos_Y);
  json_object_object_add(jobj_curpos,"Z",jdouble_curpos_Z);
  json_object_object_add(jobj_curpos,"W",jdouble_curpos_W);

  json_object *j_id_client = json_object_new_int(1000);
  json_object *jstate_proceduretimes = json_object_new_int((int)numworkingtime_sub);
  json_object *jdouble_battery = json_object_new_double(batteryInfo.currentBatteryLevel);

  json_object_object_add(jobj_amcl,"clientResponse",j_id_client);
  json_object_object_add(jobj_amcl,"ProcedureTimes",jstate_proceduretimes);
  json_object_object_add(jobj_amcl,"CurrentPos",jobj_curpos);
  json_object_object_add(jobj_amcl,"BatteryInfo",jdouble_battery);
  const char* result=json_object_to_json_string(jobj_amcl);
  return result;

}
const char* CreateJSON_ChargerRequest(string cmd, string ipAddress, int port)
{
  json_object * jobj_chargerInfoRequest = json_object_new_object();
  json_object *j_cmd = json_object_new_string(cmd.c_str());
  json_object *j_ipAddress = json_object_new_string(ipAddress.c_str());
  json_object *j_port= json_object_new_int(port);
  
  json_object_object_add(jobj_chargerInfoRequest,"command",j_cmd);
  json_object_object_add(jobj_chargerInfoRequest,"ipAddress",j_ipAddress);
  json_object_object_add(jobj_chargerInfoRequest,"port",j_port);
  const char* result=json_object_to_json_string(jobj_chargerInfoRequest);
  return result;
}
const char* CreateJSON_RobotVersion()
{
  json_object * jobj_robotInfo = json_object_new_object(); 
  json_object *j_cmd= json_object_new_int(SERVER_CALLBACK_ROBOTVERSION);
  json_object *j_version= json_object_new_int(ROBOTVERSION);
  json_object *j_accessCode= json_object_new_int(12345689);
  json_object_object_add(jobj_robotInfo,"clientCallBack",j_cmd);
  json_object_object_add(jobj_robotInfo,"version",j_version);
  json_object_object_add(jobj_robotInfo,"AccessCode",j_accessCode);
  const char* result=json_object_to_json_string(jobj_robotInfo);
  return result;
}
/*
const char* CreateJSON_ChargerRequest(string cmd, string ipAddress, int port)
{
  json_object * jobj_chargerInfoRequest = json_object_new_object();
  json_object *j_cmd = json_object_new_string(cmd.c_str());
  json_object *j_ipAddress = json_object_new_string(ipAddress.c_str());
  json_object *j_port= json_object_new_int(port);
  
  json_object_object_add(jobj_chargerInfoRequest,"hostname",j_cmd);
  json_object_object_add(jobj_chargerInfoRequest,"version",j_ipAddress);
  json_object_object_add(jobj_chargerInfoRequest,"key",j_port);
  const char* result=json_object_to_json_string(jobj_chargerInfoRequest);
  return result;

}*/
const char* CreateJSON_RosTopics()
{
  
  json_object * jobj_rostopic = json_object_new_object();
  ros::master::V_TopicInfo master_topics;
  ros::master::getTopics(master_topics);
  std::stringstream ss;
  ss<<SERVER_RESPONSED_PUBLISH_CURRENTTOPICSLIST;
  json_object *jcode = json_object_new_string(ss.str().c_str());

  /*Form the json object*/
  /*Each of these is like a key value pair*/
  json_object_object_add(jobj_rostopic,"Response",jcode);
  for (ros::master::V_TopicInfo::iterator it = master_topics.begin() ; it != master_topics.end(); it++) 
  {
	    const ros::master::TopicInfo& info = *it;
	    const char *_datatype=(const char*)info.datatype.c_str();
	    const char *_infoname=(const char*)info.name.c_str();
	    json_object *jstring = json_object_new_string(_datatype);
	    json_object_object_add(jobj_rostopic,_infoname,jstring);
	    //std::cout << "Topic : " << it - master_topics.begin() << ": " << info.name << " -> " << info.datatype <<       std::endl;
  }
  const char* result=json_object_to_json_string(jobj_rostopic);
//  printf ("The json object created: %s\n",result);
  return result;

}

//const
void checkBattery()
{
	switch(batterystate)
	{
		case MANAGER_STATE_BATTERY_NORMALBATTERY:
				//cout<<"MANAGER_STATE_BATTERY_NORMALBATTERY\n";
			//if(batteryInfo.currentBatteryLevel<batteryInfo.defautLowBattery)
				//batterystate=MANAGER_STATE__REQUEST_CHARGER_START_CHARGE;
			if(numworkingtime_sub>=6)	
			{
				batterystate=MANAGER_STATE_BATTERY_LOWBATTERY;
			}
			break;
		case MANAGER_STATE_BATTERY_LOWBATTERY:
			cout<<"MANAGER_STATE_BATTERY_LOWBATTERY\n";
			batterystate=MANAGER_STATE_BATTERY_REQUEST_GOTO_CHARGESTATION;
			break;
		case MANAGER_STATE_BATTERY_REQUESTSERVER_BATTERYSTATION:
			{
			std_msgs::String msg;
			std::stringstream ss;
			ss<<"{\"request\":8000}\n"<<"\n\n";
			msg.data=ss.str();
			pub_responseInf.publish(msg);
			batterystate=MANAGER_STATE_BATTERY_WAITINGRESPONSESERVER_BATTERYSTATION;
			cout<<"MANAGER_STATE_BATTERY_WAITING_GOTO_CHARGESTATION\n";
			}

			break;
		case MANAGER_STATE_BATTERY_WAITINGRESPONSESERVER_BATTERYSTATION:
				//cout<<" MANAGER_STATE_BATTERY_WAITINGRESPONSESERVER_BATTERYSTATION\n";
			if(batteryInfo.flagServerResponse)
			{
				batterystate=MANAGER_STATE_BATTERY_REQUEST_GOTO_CHARGESTATION;
				batteryInfo.flagServerResponse=false;
			}
			break;
		case MANAGER_STATE_BATTERY_REQUEST_GOTO_CHARGESTATION:
		{
			cout<<"MANAGER_STATE_BATTERY_REQUEST_GOTO_CHARGESTATION\n";
			std_msgs::String msg;
			std::stringstream ss;
			ss<<"{\"cmd\":7700,\"BatteryStation\":{\"X\":3.4, \"Y\":-0.33,\"Angle\":0}}"<<"\n\n";
			msg.data=ss.str();
			pub_BatteryChargeRequestSelfDriving.publish(msg);
			batterystate=MANAGER_STATE_BATTERY_WAITING_GOTO_CHARGESTATION;
			cout<<"MANAGER_STATE_BATTERY_WAITING_GOTO_CHARGESTATION\n";
		}
		case MANAGER_STATE_BATTERY_WAITING_GOTO_CHARGESTATION:				
			if(batteryInfo.flagSelfDrivingCallback)
			{
				batterystate=MANAGER_STATE_BATTERY_FINISH_GOTO_STATION;
				batteryInfo.flagSelfDrivingCallback=false;
			}
		break;
	/*	case MANAGER_STATE_BATTERY_FINISH_GOTO_CHARGESTATION:
				cout<<"MANAGER_STATE_BATTERY_FINISH_GOTO_CHARGESTATION\n";
		    batterystate=MANAGER_STATE_BATTERY_REQUEST_POWEROFF_HARWDWARE;
			break;
		case MANAGER_STATE_BATTERY_REQUEST_POWEROFF_HARWDWARE:
		{
				cout<<"MANAGER_STATE_BATTERY_REQUEST_POWEROFF_HARWDWARE\n";
				std_msgs::Int32 msg;
				msg.data=batteryInfo.SendReq_hardware_poweroff;
				pub_BatteryChargeRequestRobothardware.publish(msg);
				batterystate=MANAGER_STATE_BATTERY_WAITING_POWEROFF_HARWDWARE;
				cout<<"MANAGER_STATE_BATTERY_WAITING_POWEROFF_HARWDWARE\n";
		}
			break;
		case MANAGER_STATE_BATTERY_WAITING_POWEROFF_HARWDWARE:				
			if(batteryInfo.flagResponseHardwareCallback)
			{
				batterystate=MANAGER_STATE_BATTERY_FINISH_POWEROFF_HARWDWARE;
				batteryInfo.flagResponseHardwareCallback=false;
			}
			break;
		case MANAGER_STATE_BATTERY_FINISH_POWEROFF_HARWDWARE:
					cout<<"MANAGER_STATE_BATTERY_FINISH_POWEROFF_HARWDWARE\n";
			batterystate=MANAGER_STATE__REQUEST_CHARGER_START_CHARGE;
			break;
		case MANAGER_STATE__REQUEST_CHARGER_START_CHARGE:
		{
			std_msgs::String msg;
			std::stringstream ss;
			ss<<CreateJSON_ChargerRequest("c6001e", "192.168.1.200", 8081);
			msg.data=ss.str();
			pub_peripheralManagerToCharger.publish(msg);
			cout<<"MANAGER_STATE__REQUEST_CHARGER_START_CHARGE\n";
			batterystate=MANAGER_STATE_BATTERY_WAITINGRESPONSESERVER_CHARGING;
			cout<< "WAITING_SERVER_RESPONSE";
			usleep(5000000);
		}
			break;
		case MANAGER_STATE_BATTERY_WAITINGRESPONSESERVER_CHARGING:		
		{
			//if(batteryInfo.flagChargerResponse==true)
			{
				if(	batteryInfo.flagChargeFinished)
				{
							batterystate=MANAGER_STATE_BATTERY_FINISH_CHARGE;
							batteryInfo.flagChargeFinished=false;
							usleep(3000000);
							countfullcharger=0;
							
				}
				else
				{
					if(countfullcharger++<5)
					{
						cout<<"MANAGER_STATE__REQUEST_CHARGER_START_CHARGE\n"<<countfullcharger ;
						usleep(1000000);
						//
					}
					else
					{
						std_msgs::String msg;
						std::stringstream ss;
						ss<<CreateJSON_ChargerRequest("c6003e", "192.168.1.200", 8081);
						msg.data=ss.str();
						pub_peripheralManagerToCharger.publish(msg);
						cout<<"MANAGER_STATE__REQUEST_CHARGER_START_CHARGE\n"<<countfullcharger ;
						usleep(10000);
					}
					
				}
				//batteryInfo.flagChargerResponse=false;
			}
		}
			break;
		case MANAGER_STATE_BATTERY_FINISH_CHARGE:
			cout<<"MANAGER_STATE_BATTERY_FINISH_CHARGE\n";
			batterystate=MANAGER_STATE_BATTERY_REQUEST_POWERON_HARWDWARE;
			break;
			case MANAGER_STATE_BATTERY_REQUEST_POWERON_HARWDWARE:
		    {
			cout<<"MANAGER_STATE_BATTERY_REQUEST_POWERON_HARWDWARE\n";
				std_msgs::Int32 msg;
				msg.data=batteryInfo.SendReq_hardware_poweron;
				pub_BatteryChargeRequestRobothardware.publish(msg);
				batterystate=MANAGER_STATE_BATTERY_WAITING_POWERON_HARWDWARE;
				cout<<"MANAGER_STATE_BATTERY_WAITING_POWERON_HARWDWARE\n";
		   }
			break;
		case MANAGER_STATE_BATTERY_WAITING_POWERON_HARWDWARE:
			if(batteryInfo.flagResponseHardwareCallback)
			{
				batterystate=MANAGER_STATE_BATTERY_FINISH_POWERON_HARWDWARE;
				batteryInfo.flagResponseHardwareCallback=false;
			}
			break;
		case MANAGER_STATE_BATTERY_FINISH_POWERON_HARWDWARE:
			cout<<"MANAGER_STATE_BATTERY_FINISH_POWERON_HARWDWARE\n";
			batterystate=MANAGER_STATE_BATTERY_REQUEST_GOTO_STATION;
			break;
		case MANAGER_STATE_BATTERY_REQUEST_GOTO_STATION:
		{
			cout<<"MANAGER_STATE_BATTERY_REQUEST_GOTO_STATION\n";
			std_msgs::String msg;
			std::stringstream ss;
			ss<<"{\"cmd\":7701,\"Station\":{\"X\":3.4, \"Y\":-0.33,\"Angle\":0}}"<<"\n\n";
			msg.data=ss.str();
			pub_BatteryChargeRequestSelfDriving.publish(msg);
			batterystate=MANAGER_STATE_BATTERY_NORMALBATTERY;
			numworkingtime_sub=0;
		}
		break;
		case MANAGER_STATE_BATTERY_WAITING_GOTO_STATION:
			cout<<"MANAGER_STATE_BATTERY_WAITING_GOTO_STATION\n";
			if(batteryInfo.flagSelfDrivingCallback)
			{
				batterystate=MANAGER_STATE_BATTERY_FINISH_GOTO_STATION;
				batteryInfo.flagSelfDrivingCallback=false;
			}
		break;*/
		case MANAGER_STATE_BATTERY_FINISH_GOTO_STATION:
			batterystate=MANAGER_STATE_BATTERY_NORMALBATTERY;
			break;
	}

}
void ServerResponse(std_msgs::String msg)
{
	Document document;
	document.Parse(msg.data.c_str());
	int _clientreq=document["command"].GetInt();
	switch(_clientreq)
	{
		
		case SERVER_CALLBACK_CMD_NAVIGATION_STOP:
			printf("9 \n");
			break;
		case SERVER_CALLBACK_CMD_NAVIGATION_START:
			break;
		case SERVER_CALLBACK_CMD_NAVIGATION_SETGOALPOS:
			//move base simple goal
			{
				printf("Move Base Simple /Goal \n");
				//flag_reachedgoal=false;
				MoveBaseSimple_Goal();
			}
			break;
		case SERVER_CALLBACK_CMD_NAVIGATION_ESTIMPOS:
			
			{
				printf("InitialPose NAV \n");
				double x= document["estimate"]["posEstX"].GetDouble();
				double y= document["estimate"]["posEstY"].GetDouble();
				double angle= document["estimate"]["posEst_angle"].GetDouble();
				geometry_msgs::PoseWithCovarianceStamped pose;
				pose.header.frame_id = "map";
				pose.pose.pose.position.x=x;
				pose.pose.pose.position.y=y;
				pose.pose.pose.position.z=0;
				double th=angle*PI/180;
				pose.pose.pose.orientation.z=sin(th/2);
				pose.pose.pose.orientation.w=cos(th/2);
    				//rospy.loginfo(pose)
    			pub_navigation_Initpose.publish(pose);
			}
			break;
		case SERVER_CALLBACK_CMD_NAVIGATION_AMCLPOS_INF:
			//printf("13 \n");
			{
				/*std_msgs::String msg;
				std::stringstream ss;
				ss<<CreateJSON_AMCL(SERVER_RESPONSED_NAVIGATION_AMCLPOS_INF)<<"\n\n";
				msg.data=ss.str();
				pub_responseInf.publish(msg);*/
				//printf("____1dd \n");
			}
			break;
		case SERVER_CALLBACK_CMD_NAVIGATION_SELFDRIVING:
			printf("selfdriving \n");
			{
				pub_selfdriving.publish(msgJson);

			}
			break;
		case SERVER_CALLBACK_CMD_NAVIGATION_AMCLPOS_FIN:
			printf("14 \n");
			break;
		case SERVER_CALLBACK_CMD_PUBLISH_CURRENTTOPICSLIST:
			printf("15 \n");
			{
				
			}
			break;
		case SERVER_CALLBACK_CMD_PUBLISH_ALLINFORMS:
			printf("16 \n");
			break;
		case SERVER_CALLBACK_CMD_PUBLISH_SENSORS:
			printf("17 \n");
			break;
		case SERVER_CALLBACK_CMD_POWERON:
		{
			/*cout<<"MANAGER_STATE_BATTERY_REQUEST_POWEROFF_HARWDWARE\n";
			std_msgs::Int32 msg;
			msg.data=batteryInfo.SendReq_hardware_poweron;
			pub_BatteryChargeRequestRobothardware.publish(msg);
			batterystate=MANAGER_STATE_BATTERY_WAITING_POWEROFF_HARWDWARE;*/

		}
			break;
		case SERVER_CALLBACK_CMD_POWEROFF:
		{
			/*cout<<"MANAGER_STATE_BATTERY_REQUEST_POWEROFF_HARWDWARE\n";
			std_msgs::Int32 msg;
			msg.data=batteryInfo.SendReq_hardware_poweroff;
			pub_BatteryChargeRequestRobothardware.publish(msg);
			batterystate=MANAGER_STATE_BATTERY_WAITING_POWEROFF_HARWDWARE;*/

		}
			break;
		case SERVER_CALLBACK_ROBOTVERSION:
		{
			std_msgs::String msg;
			std::stringstream ss;
			ss<<CreateJSON_RobotVersion()<<"\n\n";
			msg.data=ss.str();
			pub_responseInf.publish(msg);
		}
			break;
	}
}
void  Currentgoal_Callback (const geometry_msgs::PoseStamped pose)
{
	currentgoal_x=pose.pose.position.x;
	currentgoal_y=pose.pose.position.y;
	currentgoal_z=pose.pose.orientation.z;
	currentgoal_w=pose.pose.orientation.w;
	//printf(" %0.2f / %0.2f / %0.2f / %0.2f",currentgoal_x,currentgoal_y,currentgoal_z,currentgoal_w);
}
void MoveBaseSimple_Goal()
{
	geometry_msgs::PoseStamped pose;
	pose.header.frame_id = "map";
	pose.pose.position.x=FRAME_CMD[7];
	currentgoal_x=FRAME_CMD[7];
	pose.pose.position.y=FRAME_CMD[8];
	currentgoal_y=FRAME_CMD[8];
	pose.pose.position.z=0;
	double th=FRAME_CMD[9]*PI/180;
	pose.pose.orientation.z=sin(th/2);
	currentgoal_z=FRAME_CMD[8]=sin(th/2);
	pose.pose.orientation.w=cos(th/2);
	currentgoal_w=cos(th/2);
	printf(" AA %0.2f / %0.2f / %0.2f / %0.2f \n",currentgoal_x,currentgoal_y,currentgoal_z,currentgoal_w);
   				//rospy.loginfo(pose)
	pub_navigation_setgoal.publish(pose);

}

void serverCtrlcallback(const std_msgs::String msg)
{
	msgJson=msg;
	Document document;
	if (false == document.Parse <0>(msg.data.c_str()).HasParseError())
	{
		//cout<<"\ndata pub22"<<msg.data.c_str()<<"\n";
		ServerResponse(msg);
	 }else{
	 // error
	 cout<<"error Json";
	}
}

void serverRobotInfoCallBack(const std_msgs::String msg)
{
	std_msgs::String data;
	std::stringstream ss;
	ss<<CreateJSON_AMCL(SERVER_RESPONSED_NAVIGATION_AMCLPOS_INF)<<"\n\n";
	data.data=ss.str();
	pub_robotInfoResponse.publish(data);
}
void serverInfoCallBack(const std_msgs::String msg)
{
	std_msgs::String data;
	std::stringstream ss;
	ss<<CreateJSON_AMCL(SERVER_RESPONSED_NAVIGATION_AMCLPOS_INF)<<"\n\n";
	data.data=ss.str();
	pub_robotInfoResponse.publish(data);
}
void serverRobotParamsCallBack(const std_msgs::String msg)
{
	std_msgs::String data;
	std::stringstream ss;
	ss<<CreateJSON_RobotVersion()<<"\n\n";
	cout<<CreateJSON_RobotVersion()<<"\n";
	data.data=ss.str();
	pub_robotParamsResponse.publish(data);
}
void serverCtrlRobotHardwareCallBack(const std_msgs::String msg)
{
	Document document;
	if (false == document.Parse <0>(msg.data.c_str()).HasParseError())
	{
		document.Parse(msg.data.c_str());
		int cmd=document["command"].GetInt();
		controlRobotHardware(cmd);
	}
	 else{
		cout<<"error Json in serverCtrlRobotHardwareCallBack()";
	}

}
void serverEstimateRobotLocationCallBack(const std_msgs::String msg)
{
	Document document;
	if (false == document.Parse <0>(msg.data.c_str()).HasParseError())
	{
		document.Parse(msg.data.c_str());
		printf("InitialPose NAV \n");
		double x= document["estimate"]["posEstX"].GetDouble();
		double y= document["estimate"]["posEstY"].GetDouble();
		double angle= document["estimate"]["posEst_angle"].GetDouble();
		geometry_msgs::PoseWithCovarianceStamped pose;
		pose.header.frame_id = "map";
		pose.pose.pose.position.x=x;
		pose.pose.pose.position.y=y;
		pose.pose.pose.position.z=0;
		double th=angle*PI/180;
		pose.pose.pose.orientation.z=sin(th/2);
		pose.pose.pose.orientation.w=cos(th/2);
    				//rospy.loginfo(pose)
    	pub_navigation_Initpose.publish(pose);
	}
	 else{
		cout<<"error Json in serverEstimateRobotLocationCallBack()";
	}
}
void serverDriveRobotCallBack(const std_msgs::String msg)
{
	Document document;
	if (false == document.Parse <0>(msg.data.c_str()).HasParseError())
	{
		document.Parse(msg.data.c_str());
		int cmd=document["command"].GetInt();
	}
	 else{
		cout<<"error Json in serverDriveRobotCallBack()";
	}
}
void serverBatteryRegisterCallBack(const std_msgs::String msg)
{
}
void serverEmergencyRobotCallBack(const std_msgs::String msg)
{
}
void controlRobotHardware(int cmd)
{
	switch(cmd)
	{
		case SERVER_CALLBACK_CMD_POWERON:
		{
			cout<<"MANAGER_STATE_BATTERY_REQUEST_POWEROFF_HARWDWARE\n";
			std_msgs::Int32 msg;
			msg.data=batteryInfo.SendReq_hardware_poweron;
			pub_ctrlRobotHardware.publish(msg);
			batterystate=MANAGER_STATE_BATTERY_WAITING_POWEROFF_HARWDWARE;

		}
			break;
		case SERVER_CALLBACK_CMD_POWEROFF:
		{
			cout<<"MANAGER_STATE_BATTERY_REQUEST_POWEROFF_HARWDWARE\n";
			std_msgs::Int32 msg;
			msg.data=batteryInfo.SendReq_hardware_poweroff;
			pub_ctrlRobotHardware.publish(msg);
			batterystate=MANAGER_STATE_BATTERY_WAITING_POWEROFF_HARWDWARE;

		}
	}
}
int main(int argc, char *argv[])
{
	//CreateJSON_AMCL(0);
	//subcribe all states
	//system(scriptBridenode);
	ros::init(argc, argv,"Manager");
	ros::NodeHandle nh;
	ros::Subscriber sub_numworkingtime = nh.subscribe("numworkingtimes",1,NumWorkingTimes_callback);
	//ros::Subscriber sub_CurrentPos = nh.subscribe("callbridgeNode",1, RequestedInf_callback);
	ros::Subscriber sub_amclpose = nh.subscribe("amcl_pose",1,NavigationAmclPose_callback);
	
	ros::Subscriber sub_movebase_status= nh.subscribe("move_base/status",1,moveBaseStatus_Callback);
	ros::Subscriber sub_currentgoal= nh.subscribe("move_base_simple/goal",1,Currentgoal_Callback);
	ros::Subscriber sub_odometry= nh.subscribe("odom",1,Odometry_callback);
	
	// publish command
	
	// Request Robot current Information
	ros::Subscriber sub_serverRobotInfo=nh.subscribe("serverRobotInfoCallBack",1,serverRobotInfoCallBack);
	pub_robotInfoResponse =nh.advertise<std_msgs::String>("robotInfoResponse", 1);
	
	// Request Robot Params
	ros::Subscriber sub_serverRobotParams=nh.subscribe("serverRobotParamsCallBack",1,serverRobotParamsCallBack);
	pub_robotParamsResponse =nh.advertise<std_msgs::String>("robotParamsResponse", 1);
	
	// Control Robot Hardware
	ros::Subscriber sub_serverCtrlRobotHardware=nh.subscribe("serverCtrlRobotHardwareCallBack",1,serverCtrlRobotHardwareCallBack);
	pub_ctrlRobotHardwareStatus =nh.advertise<std_msgs::String>("ctrlRobotHardwareStatusResponse", 1);
	
	// Estimate Robot Navigation
	ros::Subscriber sub_serverEstimateRobotLocation=nh.subscribe("serverEstimateRobotLocationCallBack",1,serverEstimateRobotLocationCallBack);
	pub_EstimateRobotLocationStatus =nh.advertise<std_msgs::String>("EstimateRobotLocationStatusResponse", 1);
	pub_navigation_Initpose=nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose", 10);
	
	// Drive Robot
	ros::Subscriber sub_serverDriveRobot=nh.subscribe("serverDriveRobotCallBack",1,serverDriveRobotCallBack);
	pub_serverDriveRobotStatus =nh.advertise<std_msgs::String>("driveRobotStatusResponse", 1);
	
	// Battery Register Robot
	ros::Subscriber sub_serverBatteryRegister=nh.subscribe("serverBatteryRegisterCallBack",1,serverBatteryRegisterCallBack);
	pub_batteryRegisterResponse =nh.advertise<std_msgs::String>("batteryRegisterResponse", 1);
	
	// Emergency Robot
	ros::Subscriber sub_serverEmergencyRobot=nh.subscribe("serverEmergencyRobotCallBack",1,serverEmergencyRobotCallBack);
	pub_emergencyRobotResponse=nh.advertise<std_msgs::String>("emergencyRobotResponse", 1);
	
	
	ros::Rate loop_rate(2);
	pthread_t threadBattery;
	pthread_create(&threadBattery, NULL,task_Battery, NULL);
	

	
		while(ros::ok())
	{
			std_msgs::String msg;
	std::stringstream ss;
	ss<<CreateJSON_AMCL(SERVER_RESPONSED_NAVIGATION_AMCLPOS_INF)<<"\n\n";
	msg.data=ss.str();
	pub_robotInfoResponse.publish(msg);
	    ros::spinOnce();
		loop_rate.sleep();
	}
	    
}
void *task_Battery (void *dummyPt)
{
				for(int i =0;i<1000;i++)
				{
					usleep(10000);
				}
	while(1)
	{
				checkBattery();
				for(int i =0;i<100;i++)
				{
					usleep(10000);
				}
	}
}
