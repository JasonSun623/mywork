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
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
using namespace std;
using namespace rapidjson;
void requestedprogress_test();
void MoveBaseSimple_Goal(double posx,double posy, double angle);
void requestedprogress_callback(const std_msgs::String::ConstPtr& msg);
void LineDetection_callback(const std_msgs::Int32& msg);
void moveBaseStatus_Callback(const actionlib_msgs::GoalStatusArrayConstPtr& msg);
void navigationAmclPose_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
void  currentgoal_Callback (const geometry_msgs::PoseStamped pose);
void odometry_callback(const nav_msgs::Odometry &odom);
void reachrobotdirection_callback(const std_msgs::String::ConstPtr& msg);
void process();
#define PI 3.14159265
ros::Publisher pub_navigation_setgoal;
ros::Publisher pub_posPallet;
ros::Publisher pub_linedetectionctrl;
ros::Publisher pub_numworkingtimes;
ros::Publisher pub_errorturningrobot;

//ros::Publisher pub_PalletLocation;
const int PROCESS_SELFDRIVING_START_READY =50000;
const int PROCESS_SELFDRIVING_START_GOTO_A_POINT = 50001;
const int PROCESS_SELFDRIVING_WAIT_GOTO_A_POINT = 50002;
const int PROCESS_SELFDRIVING_FINISHED_GOTO_A_POINT = 50003;

const int PROCESS_SELFDRIVING_START_DETECTLINE_UP = 50004;
const int PROCESS_SELFDRIVING_WAIT_DETECTLINE_UP = 50005;
const int PROCESS_SELFDRIVING_END_DETECTLINE_UP = 50006;
const int PROCESS_SELFDRIVING_START_GOTO_B_POINT = 50007;
const int PROCESS_SELFDRIVING_WAIT_GOTO_B_POINT = 50008;
const int PROCESS_SELFDRIVING_FINISHED_GOTO_B_POINT = 50009;

const int PROCESS_SELFDRIVING_START_PALLETDOWN = 50010;
const int PROCESS_SELFDRIVING_WAIT_PALLETDOWN = 50011;
const int PROCESS_SELFDRIVING_FINISH_PALLETDOWN = 50012;

const int PROCESS_SELFDRIVING_RESET = 50013;
const int PROCESS_SELFDRIVING_STOP  = 50014;
const int PROCESS_SELFDRIVING_PAUSE  = 50015;
const int PROCESS_SELFDRIVING_IDLE   =50016;

int PROCESS_SELFDRIVING_ROBOT = PROCESS_SELFDRIVING_START_READY;


double amclpose_posX=10.0,amclpose_posY=10.0,amclpose_posthetaW=0.0,amclpose_posthetaZ=0.0;
double currentgoal_x=0.0,currentgoal_y=0.0,currentgoal_z=0.0,currentgoal_w=0.0;
double current_Vx=0.0,current_Vy=0.0,current_W=0.0;


// variable from parse Json

double posXStartPalletUp[100];
double posYStartPalletUp[100];
double posStartAngleDirection[100];
double pospallet[100];
double posXEndPalletDown[100];
double posYEndPalletDown[100];
double posEndAngleDirection[100];
double posXStation[100];
double posYStation[100];
int looptime_progress=0;
int atPos=0;
bool flag_state_reachedgoal_palletup=false;
bool flag_state_reachedgoal_palletdown=false;

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
	ros::Subscriber sub_CurrentPos = nh.subscribe("RequestedProgress_SelfDriving",1000, requestedprogress_callback);
	ros::Subscriber sub_ReachedRobotdirection= nh.subscribe("ReachedRobotdirection",1000, reachrobotdirection_callback);

	ros::Subscriber sub_fromLineDetection_Node= nh.subscribe("linedetectioncallback",1,LineDetection_callback);
	ros::Subscriber sub_amclpose = nh.subscribe("amcl_pose",1,navigationAmclPose_callback);
	ros::Subscriber sub_currentgoal= nh.subscribe("move_base_simple/goal",1,currentgoal_Callback);
	ros::Subscriber sub_odometry= nh.subscribe("odom",1,odometry_callback);
	ros::Subscriber sub_movebase_status= nh.subscribe("move_base/status",1,moveBaseStatus_Callback);
	ros::Rate loop_rate(40);
    ros::spinOnce();
    int nk=0;

	while(ros::ok())
	{
		if(nk++<5000)
		{
			 requestedprogress_test();
		}

	    //cout<<"hello \n";	    
	    process();
	    usleep(100);
	    ros::spinOnce();
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
void odometry_callback(const nav_msgs::Odometry &odom)
{
	current_Vx=odom.twist.twist.linear.x;
	current_Vy =odom.twist.twist.linear.y;
	current_W  =odom.twist.twist.angular.z;
}
void requestedprogress_callback(const std_msgs::String::ConstPtr& msg)
{
	cout<<msg->data.c_str()<<"\n";
	Document document;
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
	PROCESS_SELFDRIVING_ROBOT =PROCESS_SELFDRIVING_START_READY;
	flag_state_reachedgoal_palletup=false;
	flag_state_reachedgoal_palletdown=false;
	 //printf("i = %d\n", document["command"].GetInt());
	 
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
	
	// pallet 0 A1
	posXStartPalletUp[0]=7.7;
	posYStartPalletUp[0]=-1.42;
	posStartAngleDirection[0]=180;
	pospallet[0]=3;
	posXEndPalletDown[0]=-10.08;
	posYEndPalletDown[0]=-1.55;
	posEndAngleDirection[0]=0.0;
	posXStation[0]=0.0;
	posYStation[0]=0.0;
	
	// pallet 1
	posXStartPalletUp[1]=7.7;
	posYStartPalletUp[1]=-1.42;
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
	
		// pallet 3
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
	
	
	
		
	PROCESS_SELFDRIVING_ROBOT =PROCESS_SELFDRIVING_START_READY;
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
	if(PROCESS_SELFDRIVING_ROBOT==PROCESS_SELFDRIVING_WAIT_DETECTLINE_UP)
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
	else if(PROCESS_SELFDRIVING_ROBOT==PROCESS_SELFDRIVING_WAIT_PALLETDOWN)
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
void LineDetection_callback(const std_msgs::Int32& msg)
{
	cout <<"IN END LINE"<<"\n";
  if((int)msg.data==3203)
  {
    	PROCESS_SELFDRIVING_ROBOT=PROCESS_SELFDRIVING_END_DETECTLINE_UP;
    	cout <<"END LINE"<<"\n";
  }
  else if((int)msg.data==3204)
  {
    	PROCESS_SELFDRIVING_ROBOT=PROCESS_SELFDRIVING_FINISH_PALLETDOWN;
    	cout <<"END LINE"<<"\n";
  }
}
void moveBaseStatus_Callback(const actionlib_msgs::GoalStatusArrayConstPtr& msg)
{
  //printf("size=%d\n",msg->status_list.size());
  if(!msg->status_list.empty() && msg->status_list.back().status == msg->status_list.back().SUCCEEDED){
    //printf("we are at the goal!\n");
	double _currentgoal_Ex = fabs(fabs(amclpose_posX)-fabs(currentgoal_x));
	double _currentgoal_Ey = fabs(fabs(amclpose_posY)-fabs(currentgoal_y));
	double _currentgoal_Ez = fabs(fabs(amclpose_posthetaZ)-fabs(currentgoal_z));
	double _currentgoal_Ew = fabs(fabs(amclpose_posthetaW)-fabs(currentgoal_w));
	 if(fabs(current_Vx)<0.0001 && fabs(current_Vy)<0.0001 && fabs(current_W)<0.0001 && _currentgoal_Ex<=0.8 && _currentgoal_Ey<=0.8)
	{
		 //printf(" %0.2f / %0.2f / %0.2f / %0.2f \n",_currentgoal_Ex,_currentgoal_Ey,_currentgoal_Ez,_currentgoal_Ew);
		 if(amclpose_posX>5)
		 {
			if(PROCESS_SELFDRIVING_ROBOT ==PROCESS_SELFDRIVING_WAIT_GOTO_A_POINT)
			{
				flag_state_reachedgoal_palletup=true;
			}
		 }
		 else
		 {
			if(PROCESS_SELFDRIVING_ROBOT ==PROCESS_SELFDRIVING_WAIT_GOTO_B_POINT)
			{
					flag_state_reachedgoal_palletdown=true;
			}
		 }
	}	

  }
}
void process()
{
         switch (PROCESS_SELFDRIVING_ROBOT)
            {
             case PROCESS_SELFDRIVING_STOP:
        	 break;
             case PROCESS_SELFDRIVING_RESET:
             {
        	         PROCESS_SELFDRIVING_ROBOT = PROCESS_SELFDRIVING_IDLE;
             }
        	 break;
             case PROCESS_SELFDRIVING_PAUSE:
        	 break;
             case PROCESS_SELFDRIVING_START_READY:
              // cout<<"PROCESS_SELFDRIVING_START_READY\n";
		     {
				if(looptime_progress>=1)
				{
					// update start and end
					PROCESS_SELFDRIVING_ROBOT = PROCESS_SELFDRIVING_START_GOTO_A_POINT;
					looptime_progress--;
					std_msgs::Int32 msgnum;
					msgnum.data=atPos+1; // command detect line and pallet up
					pub_numworkingtimes.publish(msgnum);

				}
				else
				{
					atPos=0;
					looptime_progress=6;
				}
		      }
                    break;
                case PROCESS_SELFDRIVING_START_GOTO_A_POINT:
                	currentgoal_x=posXStartPalletUp[atPos];
                	currentgoal_y=posYStartPalletUp[atPos];
                    cout<<".PROCESS_SELFDRIVING_START_GOTO_A_POINT\n";
                    cout<<	currentgoal_x<<"\n";
                    cout<<	currentgoal_y<<"\n";
                    cout<<	"state = "<< flag_state_reachedgoal_palletup<<"\n";
                    //double []loA={POINTA_X, POINTA_Y};
                   // sendLocation(loA, POINTA_Theta);
		            MoveBaseSimple_Goal(posXStartPalletUp[atPos],posYStartPalletUp[atPos],posStartAngleDirection[atPos]);
                    PROCESS_SELFDRIVING_ROBOT = PROCESS_SELFDRIVING_WAIT_GOTO_A_POINT;
                    flag_state_reachedgoal_palletup=false;
                    cout<<"PROCESS_SELFDRIVING_WAIT_GOTO_A_POINT\n";
                    break;
                case PROCESS_SELFDRIVING_WAIT_GOTO_A_POINT:
                    //cout<<("PROCESS_SELFDRIVING_WAIT_GOTO_A_POINT
                    // waiting 
					if(flag_state_reachedgoal_palletup)
					{
					PROCESS_SELFDRIVING_ROBOT = PROCESS_SELFDRIVING_FINISHED_GOTO_A_POINT;
					flag_state_reachedgoal_palletup=false;
					}
                    break;
            case PROCESS_SELFDRIVING_FINISHED_GOTO_A_POINT:
            {
               // temp to test bateery
            	PROCESS_SELFDRIVING_ROBOT = PROCESS_SELFDRIVING_START_GOTO_B_POINT;
               //---------------------------

            	 /*cout<<"PROCESS_SELFDRIVING_FINISHED_GOTO_A_POINT\n";
                PROCESS_SELFDRIVING_ROBOT = PROCESS_SELFDRIVING_START_DETECTLINE_UP;
                std_msgs::Int32 msg;
	            msg.data=1203; // command detect line and pallet up
		     cout<<"palletnum= "<<pospallet[atPos]<<"\n";
	            pub_linedetectionctrl.publish(msg);
	*/
				
				
	            //std_msgs::Int32 palletnum;
	           // palletnum.data=pospallet[atPos];
	       
	            //pub_posPalletup.publish(palletnum);
            }
		    break;
                case PROCESS_SELFDRIVING_START_DETECTLINE_UP:
                {

                    cout<<"PROCESS_SELFDRIVING_START_DETECTLINE_UP\n";
                    PROCESS_SELFDRIVING_ROBOT =PROCESS_SELFDRIVING_WAIT_DETECTLINE_UP;
                    cout<<"PROCESS_SELFDRIVING_WAIT_DETECTLINE_UP\n";
                }
                    break;
                case PROCESS_SELFDRIVING_WAIT_DETECTLINE_UP:
                	cout<<"PROCESS_SELFDRIVING_WAIT_DETECTLINE_UP\n"<<amclpose_posX<<"\n";
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
						if(amclpose_posX>=posXStartPalletUp[atPos]+3.2 && amclpose_posX<posXStartPalletUp[atPos]+3.7 ) // slow motion
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
						
						if(amclpose_posX>=posXStartPalletUp[atPos]+4.6 && posXStartPalletUp[atPos]+5.1) // slow motion
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
//###########################################################################
                   //
                    // waiting 
                    break;
                case PROCESS_SELFDRIVING_END_DETECTLINE_UP:
                    cout<<"PROCESS_SELFDRIVING_END_DETECTLINE_UP\n";
                    PROCESS_SELFDRIVING_ROBOT = PROCESS_SELFDRIVING_START_GOTO_B_POINT;
                    break;
                case PROCESS_SELFDRIVING_START_GOTO_B_POINT:
                    cout<<"PROCESS_SELFDRIVING_START_GOTO_B_POINT\n";
                    currentgoal_x=posXEndPalletDown[atPos];
                    currentgoal_y=posYEndPalletDown[atPos];
                    MoveBaseSimple_Goal(posXEndPalletDown[atPos],posYEndPalletDown[atPos],posEndAngleDirection[atPos]);
                    PROCESS_SELFDRIVING_ROBOT = PROCESS_SELFDRIVING_WAIT_GOTO_B_POINT;
                    cout<<"PROCESS_SELFDRIVING_WAIT_GOTO_B_POINT\n";
                    flag_state_reachedgoal_palletdown=false;
                    break;
                case PROCESS_SELFDRIVING_WAIT_GOTO_B_POINT:
                    if(flag_state_reachedgoal_palletdown)
                    {
                    		PROCESS_SELFDRIVING_ROBOT = PROCESS_SELFDRIVING_FINISHED_GOTO_B_POINT;
                    		flag_state_reachedgoal_palletdown=false;
                    }
                    // waiting 
                    break;
                case PROCESS_SELFDRIVING_FINISHED_GOTO_B_POINT:
                	 // temp to test bateery
					 atPos++;
					 
					   PROCESS_SELFDRIVING_ROBOT =PROCESS_SELFDRIVING_ROBOT =PROCESS_SELFDRIVING_START_READY;
                	//  PROCESS_SELFDRIVING_ROBOT =PROCESS_SELFDRIVING_START_GOTO_A_POINT;
                	 //---------------------------
/*
                    cout<<"PROCESS_SELFDRIVING_FINISHED_GOTO_B_POINT\n";
                       PROCESS_SELFDRIVING_ROBOT =PROCESS_SELFDRIVING_START_PALLETDOWN;*/
                    break;
                case PROCESS_SELFDRIVING_START_PALLETDOWN:
                {
                    cout<<"PROCESS_SELFDRIVING_START_DOWN\n";
                    std_msgs::Int32 msg;
    	            msg.data=1204; // command detect line and pallet up
                    pub_linedetectionctrl.publish(msg);
                    PROCESS_SELFDRIVING_ROBOT =PROCESS_SELFDRIVING_WAIT_PALLETDOWN;
                    cout<<"PROCESS_SELFDRIVING_WAIT_DOWN\n";
                }
                    break;
                case PROCESS_SELFDRIVING_WAIT_PALLETDOWN:
                	cout<<"PROCESS_SELFDRIVING_WAIT_DOWN\n"<<amclpose_posX<<"\n";
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
						
						if(amclpose_posX<=posXEndPalletDown[atPos]-6 && amclpose_posX>posXEndPalletDown[atPos]-6.7) // slow motion
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
						if(amclpose_posX<=posXEndPalletDown[atPos]-4.6 && amclpose_posX>posXEndPalletDown[atPos]-5.1) // slow motion
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
                    break;
                case PROCESS_SELFDRIVING_FINISH_PALLETDOWN:
                {
                    cout<<"PROCESS_SELFDRIVING_FINISH_DOWN\n";
                    PROCESS_SELFDRIVING_ROBOT =PROCESS_SELFDRIVING_START_READY;
                    atPos++;
                }
                    break;
                
                    //PROCESS_SELFDRIVING_START_DOWN
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

