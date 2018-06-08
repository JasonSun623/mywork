#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

#include "std_msgs/Float32.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Int32.h"
#include "std_msgs/String.h"
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Vector3.h>
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "actionlib_msgs/GoalStatusArray.h"

#include <std_msgs/UInt8MultiArray.h>
#include <math.h>

//void MoveBaseSimple_Goal();
//void Stop_Goal();

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

//double goal_posX = 0.0, goal_posY = 0.0, goal_posthetaW = 0.0, goal_posthetaZ = 0.0;
//double amclpose_posX = 0.0,amclpose_posY = 0.0,amclpose_posthetaW = 0.0,amclpose_posthetaZ = 0.0;
//double goal_posX_bk = 0.0, goal_posY_bk = 0.0, goal_posthetaW_bk = 0.0, goal_posthetaZ_bk = 0.0;

float data_lms[181],check_data[5],check = 0;//, check_data_front[5];
int flag = 0, pos = 0, check_pos = 0, check_data_pos[5], pos_front = 0;
bool flag_state = true, flag_info = true;

//Publishes
//ros::Publisher pub_navigation_setgoal;
ros::Publisher pub_stop_flag;
//Done
/*
void GoalCallBack(const geometry_msgs::PoseStamped::ConstPtr& msg){
	goal_posX=msg->pose.position.x;
	goal_posY=msg->pose.position.y;
	goal_posthetaW=msg->pose.orientation.w;
	goal_posthetaZ=msg->pose.orientation.z;
	//ROS_INFO("x: [%f]", goal_posX);
    //ROS_INFO("y: [%f]", goal_posY);
}
*/
//
void ScanCallBack(const sensor_msgs::LaserScan::ConstPtr& scan){
     //scan->ranges[]
	 if (flag_state == true){
		//flag_info = false;
	     for(int i = 0; i < 181; i++){
	    	 data_lms[i] = scan->ranges[i];
	    	 //ROS_INFO("Data: [%f]", scan->ranges[i]);
	    	 //ROS_INFO("Data i: [%d]", i);
	     }
	     pos = 0;
	     pos_front = 0;
	     std::fill_n(check_data, 4, 100);
	     //======================================
	     for (int j = 0; j<4; j++){
	         for(int k = 0; k < 181; k++){
	        	 if (check_data[pos] >= data_lms[k]){
	        		check_data[pos] = data_lms[k];
	        		check_data_pos[pos] = k;
	            	check_pos = k;
	        	 }
	         }
	         data_lms[check_pos] = 100;
	    	 pos++;
	     }
	     //======================================
	     for (int l = 0; l<4; l++){
	    	 check = check + check_data[l];
	    	 if ((check_data_pos[l] <= 135)&&(check_data_pos[l] >= 45)) pos_front++;

	    	 //ROS_INFO("Data: [%f]", check_data[l]);
	    	 //ROS_INFO("Data i: [%d]", check_data_pos[l]);
	     }
	     //ROS_INFO("===================================");
	     check = check/4;
	     //======================================
	     //MoveBaseClient ac("move_base", false);
		/*
	     	if ((check > 1.2)&&(check <= 2.5)&&(pos_front == 5)){ //warn 1 //front
			 //ac.cancelAllGoals();
			 //goal_posX_bk=goal_posX;
			 //goal_posY_bk=goal_posY;
			 //goal_posthetaW_bk=goal_posthetaW;
			 //goal_posthetaZ_bk=goal_posthetaZ;
			 //Stop_Goal();
			 ROS_INFO("Front: [%d]", pos_front);
			 std_msgs::Int32 msg;
			 msg.data = 1402;
			 pub_stop_flag.publish(msg);
			 ROS_INFO("SLOWDOWN ROBOT");
			 //ROS_INFO("Data check: [%f]", check);
		 }
		 
		 else if ((check <= 1.2)&&(pos_front == 5)){  //stop //front
			 ROS_INFO("Front: [%d]", pos_front);
			 std_msgs::Int32 msg;
			 msg.data = 1002;
			 pub_stop_flag.publish(msg);
			 ROS_INFO("STOP ROBOT");
		 }
		 
		 else if ((check > 1.0)&&(check <= 1.5)&&(pos_front != 5)){ //warn 1 //side
			 ROS_INFO("Front: [%d]", pos_front);
			 std_msgs::Int32 msg;
			 msg.data = 1402;
			 pub_stop_flag.publish(msg);
			 ROS_INFO("SLOWDOWN ROBOT");
		 }
		
		 else if ((check <= 1.0)&&(pos_front != 5)){  //stop //side	
			 ROS_INFO("Front: [%d]", pos_front);
			 std_msgs::Int32 msg;
			 msg.data = 1002;
			 pub_stop_flag.publish(msg);
			 ROS_INFO("STOP ROBOT");
		 }
		 
		 else if ((check >= 1.7)&&(pos_front != 5)){  //run
			 ROS_INFO("Front: [%d]", pos_front);
			 std_msgs::Int32 msg;
			 msg.data = 1802;
			 pub_stop_flag.publish(msg);
			 ROS_INFO("NO OBSTACLE");
		 }

		 else if ((check >= 2.7)&&(pos_front == 5)){  //run	
			 ROS_INFO("Front: [%d]", pos_front);
			 std_msgs::Int32 msg;
			 msg.data = 1802;
			 pub_stop_flag.publish(msg);
		 	 ROS_INFO("NO OBSTACLE");
		 }
		 */
		std_msgs::Int32 msg;
		if (pos_front == 4){
			 //ROS_INFO("Front: [%d]", pos_front);
			 if (check > 3.0){
			 	msg.data = 1802;
			 	pub_stop_flag.publish(msg);
				//ROS_INFO("=======OK=======");
			 }
			 else if ((check > 2.5)&&(check <= 3.0)){
			 	msg.data = 1402;
			 	pub_stop_flag.publish(msg);
				//ROS_INFO("WARNING ZONE 1");
			 	ROS_INFO("Slowdown");
			 }
			 else if ((check > 1.7)&&(check <= 2.5)){
			 	msg.data = 1202;
			 	pub_stop_flag.publish(msg);
				//ROS_INFO("WARNING ZONE 2");
			 	ROS_INFO("Slowdown");
			 }
			 else if (check <= 1.7){
			 	msg.data = 1002;
			 	pub_stop_flag.publish(msg);
				//ROS_INFO("DANGER ZONE");
			 	ROS_INFO("Stop");
			 }
		 }
		else if (pos_front != 4){
			 //ROS_INFO("Front: [%d]", pos_front);
			 if (check > 1.5){
			 	msg.data = 1802;
			 	pub_stop_flag.publish(msg);
				//ROS_INFO("========OK========");
			 }
			 else if ((check > 1.5)&&(check <= 1.7)){
			 	msg.data = 1402;
			 	pub_stop_flag.publish(msg);
			 	ROS_INFO("Slowdown");
			 }
			 else if ((check > 1.2)&&(check <= 1.5)){
			 	msg.data = 1202;
			 	pub_stop_flag.publish(msg);
			 	ROS_INFO("Slowdown");
			 }
			 else if (check <= 1.2){
			 	msg.data = 1002;
			 	pub_stop_flag.publish(msg);
				//ROS_INFO("DANGER ZONE");
			 	ROS_INFO("Stop");
			 }
		 }
		

	     //ROS_INFO("angle min: [%f]", scan->angle_min);
	     //ROS_INFO("angle max: [%f]", scan->angle_max);
	     //ROS_INFO("scan time: [%f]", scan->time);
	 }
	 else if (flag_state == false){
		 //if (flag_info == false){
		 //	ROS_INFO("PAUSE SAFETY NODE");
			// flag_info = true;
		 //}
	 }
		 
}

void LineDetection_Callback(const std_msgs::Int32& msg){ //on
    if((int)msg.data==3203||(int)msg.data==3204){
    	flag_state = true;
        ROS_INFO("FLAG 3203 3204");
	ROS_INFO("SAFETY NODE START");
    }
}

void LineDetectionCtrl_Callback(const std_msgs::Int32& msg){ //tat
    if((int)msg.data==1203||(int)msg.data==1204){
    	flag_state = false;
	ROS_INFO("FLAG 1203 1204");
	ROS_INFO("PAUSE SAFETY NODE");
    }
    //else if ((int)msg.data==1200) 
    //	flag_state = true;
    //	ROS_INFO("FLAG 1200");
}

void ErrorDetectedLine_Callback(const std_msgs::Int32& msg){ //tat
    if((int)msg.data==4205){
	flag_state = true;
	ROS_INFO("FLAG 4205");
	ROS_INFO("SAFETY NODE START");
    }
}
/*
void AmclCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg){
	amclpose_posX=msg->pose.pose.position.x;
	amclpose_posY=msg->pose.pose.position.y;
	amclpose_posthetaW=msg->pose.pose.orientation.w;
	amclpose_posthetaZ=msg->pose.pose.orientation.z;
	//ROS_INFO("x amcl: [%f]", amclpose_posX);
    //ROS_INFO("y amcl: [%f]", amclpose_posY);
}

void Stop_Goal()
{
	geometry_msgs::PoseStamped pose;
	
	pose.header.frame_id = "map";
	pose.pose.position.x = amclpose_posX;
	pose.pose.position.y = amclpose_posY;
	pose.pose.position.z = 0.0;

	pose.pose.orientation.z = amclpose_posthetaZ;
	pose.pose.orientation.w = amclpose_posthetaW;
    //rospy.loginfo(pose)
    pub_navigation_setgoal.publish(pose);
}

void MoveBaseSimple_Goal()
{
	geometry_msgs::PoseStamped pose;
	
	pose.header.frame_id = "map";
	pose.pose.position.x = goal_posX_bk;
	pose.pose.position.y = goal_posY_bk;
	pose.pose.position.z = 0.0;

	pose.pose.orientation.z = goal_posthetaZ_bk;
	pose.pose.orientation.w = goal_posthetaW_bk;
    //rospy.loginfo(pose)
    pub_navigation_setgoal.publish(pose);
}
*/
int main(int argc, char** argv){
	
	/*
	ros::init(argc, argv, "nav_simple");

	//tell the action client that we want to spin a thread by default
	MoveBaseClient ac("move_base", true);

	//wait for the action server to come up
	while(!ac.waitForServer(ros::Duration(5.0))){
		ROS_INFO("Waiting for the move_base action server to come up");
	}

	move_base_msgs::MoveBaseGoal goal;

	//we'll send a goal to the robot to move 1 meter forward
	goal.target_pose.header.frame_id = "base_link";
	goal.target_pose.header.stamp = ros::Time::now();

	goal.target_pose.pose.position.x = 3.0;
	goal.target_pose.pose.orientation.w = 1.0;

	ROS_INFO("Sending goal");
	ac.sendGoal(goal);

	ac.waitForResult();

	if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
		ROS_INFO("Hooray, the base moved 1 meter forward");
	else
		ROS_INFO("The base failed to move forward 1 meter for some reason");

	return 0;
	*/
	ros::init(argc, argv,"safety_rule");
	ros::NodeHandle nh;
	//ros::Subscriber  sub_amclpose = nh.subscribe("amcl_pose",1,AmclCallback); //get pose
	ros::Subscriber  sub_laserscan = nh.subscribe("/scan",10,ScanCallBack);					//get laser data
	//ros::Subscriber  sub_goal = nh.subscribe("/move_base_simple/goal",1,GoalCallBack);	//get goal
	ros::Subscriber  sub_LineDetectionCtrl_Node = nh.subscribe("linedetectionctrl",10,LineDetectionCtrl_Callback);
	ros::Subscriber  sub_LineDetection_Node = nh.subscribe("linedetectioncallback",10,LineDetection_Callback);	
	ros::Subscriber  sub_ErrorDetectedLine_Node = nh.subscribe("errorDetectedLine",10,ErrorDetectedLine_Callback);
	
	//pub_navigation_setgoal=nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal",1000);		//pub goal
	pub_stop_flag=nh.advertise<std_msgs::Int32>("stop_flag",10);
	ros::Rate loop_rate(50);
	while(ros::ok())
	{
           ros::spinOnce();
	}
	
}
