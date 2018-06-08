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

void ResponseInf_callback(const std_msgs::String & msg)
{
	//cout<<msg.data<<"\n";
	if(connfd>0)	
	write(connfd, msg.data.c_str(), strlen(msg.data.c_str()));
}

int initTcpIp()
{
	int optval;
	listenfd = socket(AF_INET, SOCK_STREAM, 0);
	memset(&serv_addr, '0', sizeof(serv_addr));
	memset(sendBuff, '0', sizeof(sendBuff));

	serv_addr.sin_family = AF_INET;
	serv_addr.sin_addr.s_addr =INADDR_ANY;
	serv_addr.sin_port = htons(11000);
	printf("waiting connect....\n");
	optval = 1;
   	if(setsockopt(listenfd, SOL_SOCKET, SO_KEEPALIVE, &optval, sizeof(optval)) < 0) {
      		perror("setsockopt()");
      		close(listenfd);
      		exit(EXIT_FAILURE);
   	}
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
        //system("p=$(pidof bridgenode)");
        //system("kill -9 $p");
//	cout<< value<<"\n";
	ros::init(argc, argv,"bridgeTcpIp");
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
		bool flag_connect=false;
		connfd = accept(listenfd, (struct sockaddr *)&client, (socklen_t*)&c);
	    printf("accepted\n");
		flag_connect=true;
		while(flag_connect)
		{
			string data="";
			int total_size=0;
				while(1)
				{
					char c;
					int nr=read(connfd,&c, sizeof(c));
					fflush(stdin);
					if(nr>0)
					{
						if(c=='#')
						{
							break;
						}
						else
						{
							data+=c;
							total_size++;

						}
					}
					else
					{
						flag_connect=false;
						break;
					}
				}
				if(flag_connect)
				{
					char *ack="hello\n";
					int nw=write(connfd,ack, strlen(ack)); // sendack
					std_msgs::String msg;
					std::stringstream ss;
					ss <<data;
					msg.data = ss.str();
					printf("%s\n",msg.data.c_str());
					printf("%d\n",total_size);
					pub.publish(msg);
					ros::spinOnce();
				}
		}
		ros::spinOnce();
		close(connfd);
	}
	close(listenfd);
}
