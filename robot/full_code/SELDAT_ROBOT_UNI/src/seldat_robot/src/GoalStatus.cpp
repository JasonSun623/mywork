#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include <move_base_msgs/MoveBaseAction.h>
#include "actionlib_msgs/GoalStatusArray.h"
using namespace std;
ros::Publisher pub_statusgoal;
void moveBaseStatus_Callback(const actionlib_msgs::GoalStatusArrayConstPtr& msg);
int main(int argc, char *argv[])
{
	ros::init(argc, argv,"goalstatus");
	ros::NodeHandle nh;
	pub_statusgoal=nh.advertise<std_msgs::Bool>("reachedGoal",1);
	ros::Subscriber sub_movebase_status= nh.subscribe("move_base/status",1,moveBaseStatus_Callback);
	ros::Rate loop_rate(10);
    ros::spinOnce();
	while(ros::ok())
	{
	    ros::spinOnce();
		loop_rate.sleep();
	}
}
void moveBaseStatus_Callback(const actionlib_msgs::GoalStatusArrayConstPtr& msg)
{
	cout<<msg->status_list.back().status<<" ---------- \n";
	if(!msg->status_list.empty() && msg->status_list.back().status == msg->status_list.back().SUCCEEDED)
	{
		std_msgs::Bool reached;
		reached.data=true;
		pub_statusgoal.publish(reached);
	}
	else	
	{
		std_msgs::Bool reached;
		reached.data=false;
		pub_statusgoal.publish(reached);
	}
}
