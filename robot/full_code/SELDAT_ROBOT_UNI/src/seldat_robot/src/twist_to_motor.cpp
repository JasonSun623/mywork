#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/Twist.h"

#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <math.h>

#define flag_enable  1002
#define flag_disable 1802
#define flag_1202    1202
#define flag_1402    1402
#define slowdown_lvl_1   0.3
#define slowdown_lvl_2   0.15

class TwistToMotors
{

public:
	TwistToMotors();
	void spin();

private:
	ros::NodeHandle n;
	
	ros::Publisher pub_fmotor;
	ros::Publisher pub_smotor;

	ros::Subscriber cmd_vel_sub;
	ros::Subscriber stop_flag_sub;
	ros::Subscriber stop_rad_sub;
	ros::Subscriber manual_sub;
	ros::Subscriber stop_flag_2_sub;
	ros::Subscriber server_speed_sub;	

	float front;
	float angle;
	float d,r;
	#define PI 3.14159265

	float ticks_since_target;
	double timeout_ticks;

	double rate;
	float dx,dy,dr,stop_rad;
	
	int stop_flag, manual_flag, stop_flag_2;
	int server_speed;
	
	void init_variables();
	void get_parameters();

	void spinOnce();
	void twistCallback(const geometry_msgs::Twist &twist_aux);
	void stopCallback(const std_msgs::Int32 &msg);
	void stop2Callback(const std_msgs::Int32 &msg);
	void stopRadCallback(const std_msgs::Float32 &msg);
	void manualCallback(const std_msgs::Int32 &msg);
	void serverCallback(const std_msgs::Int32 &msg);
};

TwistToMotors::TwistToMotors()
{
	init_variables();
	get_parameters();
	
	ROS_INFO("Started Twist to Motor node");
	
	cmd_vel_sub = n.subscribe("cmd_vel_mux/input/navi",10, &TwistToMotors::twistCallback, this);
	stop_flag_sub = n.subscribe("stop_flag",10, &TwistToMotors::stopCallback,this);
	stop_rad_sub = n.subscribe("line_stop_rad",10, &TwistToMotors::stopRadCallback,this);
	manual_sub = n.subscribe("manual",10, &TwistToMotors::manualCallback,this);
	stop_flag_2_sub = n.subscribe("stop_flag_2",10, &TwistToMotors::stop2Callback,this);
	server_speed_sub = n.subscribe("ctrlRobotDriving",10,&TwistToMotors::serverCallback,this);		
	pub_fmotor = n.advertise<std_msgs::Float32>("fwheel_vtarget", 10);
	pub_smotor = n.advertise<std_msgs::Float32>("swheel_rtarget", 10);
}

void TwistToMotors::init_variables()
{
	front = 0;
	angle = 0;	
	d = 1.136;
	r = 0.125;
	dx = dy = dr =0;
	stop_flag_2 = 1802;
	stop_flag = 1802;
	manual_flag = 1802;
	stop_rad = 0;
	server_speed = 100;
	//w = 0.55 ; //distance between two wheels
	rate = 20;
	timeout_ticks = 2;
}


void TwistToMotors::get_parameters()
{
        if(n.getParam("rate", rate)){
	 
		ROS_INFO_STREAM("Rate from param" << rate);	       
	}
	
        if(n.getParam("timeout_ticks", timeout_ticks)){
	 
		ROS_INFO_STREAM("timeout_ticks from param" << timeout_ticks);	       
	}
	
	// if(n.getParam("base_width", w)){
	//	ROS_INFO_STREAM("Base_width from param" << w);	       
	//}
}


void TwistToMotors::spin()
{
	ros::Rate r(rate);
	ros::Rate idle(10);

	ros::Time then = ros::Time::now();
	ticks_since_target = timeout_ticks;

	while (ros::ok())
	{
	while (ros::ok() && (ticks_since_target <= timeout_ticks))	
		{		

		spinOnce();
		r.sleep();

		}
	ros::spinOnce();
        idle.sleep();	

	}

}

void TwistToMotors::spinOnce()
{
	///////////////////////////////////////////////////////////////////
	if (dx != 0){
		angle = atan((d*dr)/dx);

		//if (dx < 0)
		//	angle = angle * -1;
		front = (dx/cos(angle))*(server_speed/100);
		
		if ((angle > 0.25)||(angle < -0.25))
			front = front/1.25;
	}
	else if ((dx == 0)&&(stop_rad == 0)){
		angle = 0;
		front = 0;
	}
	else if ((dx == 0)&&(stop_rad != 0)){
		angle = stop_rad;
		front = 0;
	}
	///////////////////////////////////////////////////////////////////
	if (manual_flag == flag_enable){
		front = 0;}
	else if (manual_flag == flag_disable){
		if ((stop_flag == flag_1402)&&(server_speed > slowdown_lvl_1)){
			if (front > slowdown_lvl_1) front = slowdown_lvl_1;}
		else if ((stop_flag == flag_1202)&&(server_speed > slowdown_lvl_2)){
			if (front > slowdown_lvl_2) front = slowdown_lvl_2;}
		else if (stop_flag == flag_enable){
			front = 0;}
		if (stop_flag_2 == flag_enable)
			front = 0;
	}


	/////////////////////////////////
	//printf("Twist front---- [%f]\n", front);
	//printf("Twist angle---- [%f]\n", angle);
	/////////////////////////////////

	std_msgs::Float32 front_;
	std_msgs::Float32 angle_;

	front_.data = front;
	angle_.data = angle;

	pub_fmotor.publish(front_);
	pub_smotor.publish(angle_);

	ticks_since_target += 1;
	ros::spinOnce();
}

void TwistToMotors::twistCallback(const geometry_msgs::Twist &msg)
{

	ticks_since_target = 0;
	
	dx = msg.linear.x;
	dy = msg.linear.y;
	dr = msg.angular.z;

	ROS_DEBUG("NAVIGATION => velocity: [%f]\n", dx);
	ROS_DEBUG("NAVIGATION => angular velocity: [%f] \n", dr);
}

void TwistToMotors::stopCallback(const std_msgs::Int32 &msg)
{
	stop_flag = msg.data;
	ROS_DEBUG("STOP FLAG: [%d]\n", stop_flag);
}

void TwistToMotors::stop2Callback(const std_msgs::Int32 &msg)
{
        stop_flag_2 = msg.data;
        ROS_DEBUG("STOP FLAG 2: [%d]\n", stop_flag_2);
}

void TwistToMotors::stopRadCallback(const std_msgs::Float32 &msg)
{
	stop_rad = msg.data;
	ROS_DEBUG("STOP RAD: [%f]\n", stop_rad);
}

void TwistToMotors::serverCallback(const std_msgs::Int32 &msg)
{
	server_speed = msg.data;
	ROS_DEBUG("SERVER SPEED: [%d]\n", server_speed);
}

void TwistToMotors::manualCallback(const std_msgs::Int32 &msg)
{
	manual_flag = msg.data;
	ROS_DEBUG("MANUAL FLAG: [%d]\n", manual_flag);
}

int main(int argc, char **argv)
{

	ros::init(argc, argv,"twist_to_motor");
	TwistToMotors obj;

	obj.spin();

}
