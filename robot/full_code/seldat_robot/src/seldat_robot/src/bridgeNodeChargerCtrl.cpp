#include <iostream>
#include <string>
#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <netdb.h>
#include <sys/uio.h>
#include <sys/time.h>
#include <sys/wait.h>
#include <fcntl.h>
#include <fstream>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
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
#include "rapidjson/document.h"
#include <pthread.h>
using namespace std;
using namespace rapidjson;
struct chargerInfoResquested{
	bool statusFlagFinishedSENREC=false;
	bool statusFlagRequestFromManager=false;
	string command;
	string IpAddress;
	int port;
};
chargerInfoResquested  chargerInfoResquestedSruct;
int connfd = 0;
ros::Publisher pub;
void *task_createserver (void *dummyPt);
void PeripheralManagerToCharger(const std_msgs::String & msg)
{
	Document document;
	cout<<msg.data<<"\n";
	if (false == document.Parse <0>(msg.data.c_str()).HasParseError())
	{
	//	cout<<msg.data<<"\n";
		document.Parse(msg.data.c_str());
		string command=document["command"].GetString();
		string ipAddress=document["ipAddress"].GetString();
		int port=document["port"].GetInt();
		chargerInfoResquestedSruct.command=command;
		chargerInfoResquestedSruct.IpAddress=ipAddress;
		chargerInfoResquestedSruct.port=port;
		chargerInfoResquestedSruct.statusFlagRequestFromManager=true;
	 }else{
	 // error
	 cout<<"error Json";
	}

}

int initTcpIp()
{
	char *serverIp = "192.168.1.200"; int port = 8081; 
	struct hostent* host = gethostbyname(serverIp); 
    sockaddr_in sendSockAddr;   
	int clientSd = socket(AF_INET, SOCK_STREAM, 0);
    sendSockAddr.sin_family = AF_INET; 
    sendSockAddr.sin_addr.s_addr = inet_addr(serverIp);
    sendSockAddr.sin_port = htons(port);
	struct timeval tv;
 	tv.tv_sec = 5;       /* Timeout in seconds */ 
	tv.tv_usec = 0;        // Not init'ing this can cause strange errors
	setsockopt(clientSd, SOL_SOCKET, SO_SNDTIMEO,(struct timeval *)&tv,sizeof(struct timeval));
	setsockopt(clientSd, SOL_SOCKET, SO_RCVTIMEO,(struct timeval *)&tv,sizeof(struct timeval));
	cout<<"waiting server connect \n";
	while(1)
	{
		int status = connect(clientSd,(sockaddr*) &sendSockAddr, sizeof(sendSockAddr));
		if(status>=0)
			break;
	}

    cout << "Connected to the server!" << endl;
    int bytesRead, bytesWritten = 0;
    struct timeval start1, end1;
    gettimeofday(&start1, NULL);
	return clientSd;
}
void *task_createserver (void *dummyPt)
{
	//int c=sizeof(struct sockaddr_in);
		while(1)
		{
			//
			if(chargerInfoResquestedSruct.statusFlagRequestFromManager)
			{
				connfd =initTcpIp();
				bool flag_connect=true;
				bool flag_readTimeout=false;
				printf("sending data\n");
				while(flag_connect)
				{
					string data="";
					std_msgs::String  msg;
					msg.data=chargerInfoResquestedSruct.command;
					write(connfd, msg.data.c_str(), strlen(msg.data.c_str()));
					while(1)
					{
						char c;
						int nr= recv(connfd,&c,1,0);
						fflush(stdin);
						if(nr>0)
						{
							if(c=='\n')
							{
								flag_connect=true;
								break;
							}
							else
							{
								data+=c;
							}
						}
						else
						{
							flag_readTimeout=true;
							break;
						}
					}
					if(flag_connect)
					{			
						std_msgs::String msg;
						std::stringstream ss;
						ss <<data;
						msg.data = ss.str();
						printf("\n%s \n",msg.data.c_str());
						pub.publish(msg);
						break;
					}	
					if(flag_readTimeout)
					{
						break;
					}
				}
				if(flag_readTimeout)
				{
					printf("\n timeout true !");
					chargerInfoResquestedSruct.statusFlagRequestFromManager=true;
				}
				else
				{
					chargerInfoResquestedSruct.statusFlagRequestFromManager=false;
				}
				close(connfd);
				connfd=-1;
			}
			usleep(1000000);
		}
}
int main(int argc, char *argv[])
{
	ros::init(argc, argv,"bridgeNodeChargerCrl");
	ros::NodeHandle nh;
	//pub = nh.advertise<std_msgs::UInt8MultiArray>("callbridgeNode", 100);
	pub = nh.advertise<std_msgs::String>("chargerCtrlCallBack", 1);
	ros::Subscriber sub_PeripheralManagerToCharger= nh.subscribe("peripheralManagerToCharger",1, PeripheralManagerToCharger);
	ros::Rate loop_rate(10);
    ros::spinOnce();
	//connfd =initTcpIp();
	pthread_t thread_server;
	pthread_create(&thread_server, NULL,task_createserver, NULL);
	ros::spin();
}
