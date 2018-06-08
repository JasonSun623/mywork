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
#include "std_msgs/String.h"
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Vector3.h>
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "std_msgs/UInt8MultiArray.h"
#include <turtlesim/Pose.h>
#include <json/json.h>
#include <pthread.h>
using namespace std;
#define AMOUTS_CLIENT_CONNECTED 1
int connfd_client[AMOUTS_CLIENT_CONNECTED];
std::string CreateDataJson();
void *task_createserver (void *dummyPt);
int readline(int fd, char* pResponse);
void *task_recvRequest (void *dummyPt);
int listenfd = 0, connfd = 0;
int noThread = 0;
struct sockaddr_in serv_addr, client;
ros::Publisher pub;
char sendBuff[10025];
double posX=10;
double posY=10;
double posthetaW=0.80;
double posthetaZ=0.80;
double Vel_posX=0;
double W_theta=0;
bool flag_connected=true;
bool flag_resp=false;
std_msgs::String msgData;
void ResponseInf_callback(const std_msgs::String & msg)
{
	msgData=msg;
}

int initTcpIp()
{
	listenfd = socket(AF_INET, SOCK_STREAM, 0);
	memset(&serv_addr, '0', sizeof(serv_addr));
	memset(sendBuff, '0', sizeof(sendBuff));

	serv_addr.sin_family = AF_INET;
	serv_addr.sin_addr.s_addr =INADDR_ANY;
	serv_addr.sin_port = htons(12000);
	printf("waiting connect....\n");
	if(bind(listenfd, (struct sockaddr*)&serv_addr, sizeof(serv_addr))<0)
	{
		   //print the error message

		   perror("bind failed. Error");
		   return -1;
	}
	//int c = sizeof(struct sockaddr_in); //

	listen(listenfd, 3);
	printf("binded....\n");
	return 1;
}
void *task_createserver (void *dummyPt)
{
   initTcpIp();
}
int main(int argc, char *argv[])
{
	ros::init(argc, argv,"StreamData");
	ros::NodeHandle nh;
	//pub = nh.advertise<std_msgs::UInt8MultiArray>("callbridgeNode", 100);
	pub = nh.advertise<std_msgs::String>("callbridgeNode", 100);
	ros::Subscriber sub_CurrentPos = nh.subscribe("ResponseInf",1000, ResponseInf_callback);
	ros::Rate loop_rate(40);
    ros::spinOnce();
	//pthread_t thread_server;
	//pthread_create(&thread_server, NULL,task_createserver, NULL);
	initTcpIp();
	while(ros::ok())
	{
		int c=sizeof(struct sockaddr_in);
		connfd = accept(listenfd, (struct sockaddr *)&client, (socklen_t*)&c);
		if (connfd < 0)
		{
			printf(" Accept is error !\n");
		}
		else
		{
			char buffer[2048*2048];
			int nr=read(connfd,&buffer, sizeof(buffer));
			if(nr<0)
			{
				printf(" read connection is error !\n");
			}
			else if(nr>0)
			{
				char *ack="hello\n";
				//printf("ok....\n");
				//write(connfd,ack, strlen(ack)); // sendack
			//	msgData.data="{ \"ClientID\": 10, \"Process\": 9000, \"Response\": \"2304\", \"CurrentPos\": { \"X\": 10, \"Y\": 10, \"Z\": 0.009666, \"W\": 0.999953 }, \"GoalPos\": { \"X\": 2.140000, \"Y\": 2.140000, \"Z\": 2.140000, \"W\": 2.140000 }, \"PathFinder\":  [0.0, 0.0, 0.10000000149011612, 0.0, 0.20000000298023224, 0.0, 0.30000001192092896, 0.0, 0.4000000059604645, 0.0, 0.5, 0.0, 0.6000000238418579, 0.0, 0.7000000476837158, 0.0, 0.8000000715255737, 0.0, 0.9000000953674316, 0.0, 1.0000001192092896, 0.0, 1.1000001430511475, 0.0, 1.2000001668930054, 0.0, 1.599500298500061, -0.019991667941212654, 1.9950087070465088, -0.07976692169904709, 2.3825736045837402, -0.17872850596904755, 2.7583227157592773, -0.3158876299858093, 3.118501663208008, -0.48987382650375366, 3.4595115184783936, -0.6989487409591675, 3.777945041656494, -0.9410233497619629, 4.070620536804199, -1.2136788368225098, 4.334613800048828, -1.5141910314559937, 4.567286968231201, -1.8395572900772095, 4.766315460205078, -2.1865265369415283, 4.929710388183594, -2.5516321659088135, 5.0558390617370605, -2.9312260150909424, 5.143441677093506, -3.3215153217315674, 5.191642761230469, -3.7186005115509033, 5.204137325286865, -3.9682881832122803, 5.241497039794922, -4.215480804443359, 5.303348064422607, -4.457708835601807, 5.389072418212891, -4.692552089691162, 5.497813701629639, -4.917664051055908, 5.628485679626465, -5.130795001983643, 5.779782295227051, -5.329815864562988, 5.950191974639893, -5.512738227844238, 6.138011932373047, -5.677733898162842, 6.341365814208984, -5.823154449462891, 6.558221817016602, -5.94754695892334, 6.786412715911865, -6.049668788909912, 7.023658752441406, -6.128499507904053, 7.267589569091797, -6.183250904083252, 7.367589473724365, -6.183250904083252, 7.467589378356934, -6.183250904083252, 7.567589282989502, -6.183250904083252, 7.66758918762207, -6.183250904083252, 7.767589092254639, -6.183250904083252, 7.867588996887207, -6.183250904083252, 7.967588901519775, -6.183250904083252, 8.067588806152344, -6.183250904083252, 8.16758918762207, -6.183250904083252, 8.267589569091797, -6.183250904083252, 8.367589950561523, -6.183250904083252, 8.46759033203125, -6.183250904083252, 8.567590713500977, -6.183250904083252, 8.667591094970703, -6.183250904083252, 8.76759147644043, -6.183250904083252, 8.867591857910156, -6.183250904083252, 8.967592239379883, -6.183250904083252, 9.06759262084961, -6.183250904083252, 9.167593002319336, -6.183250904083252, 9.267593383789062, -6.183250904083252, 9.367593765258789, -6.183250904083252, 9.467594146728516, -6.183250904083252, 9.567594528198242, -6.183250904083252]}";

				write(connfd, msgData.data.c_str(), strlen(msgData.data.c_str()));
			}
		}
		close(connfd);
		connfd=-1;
		ros::spinOnce();
	}
	close(connfd);
	connfd=-1;
	close(listenfd);
}
