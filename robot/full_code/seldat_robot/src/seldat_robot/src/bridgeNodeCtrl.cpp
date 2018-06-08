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
#include <pthread.h>
using namespace std;
int listenfd = 0, connfd = 0;
int noThread = 0;
struct sockaddr_in serv_addr, client;
ros::Publisher pub;
char sendBuff[10025];
void *task_createserver (void *dummyPt);
void ResponseInf_callback(const std_msgs::String & msg)
{
	cout<<"data send : "<<msg.data;
	/*std::string str;
	str.append(msg.data);
	str.append("\n");*/
	if(connfd>0)	
		write(connfd, msg.data.c_str(), strlen(msg.data.c_str()));
}

int initTcpIp()
{
	char *serverIp = "192.168.1.20"; int port = 19000; 
	struct hostent* host = gethostbyname(serverIp); 
    sockaddr_in sendSockAddr;   
	int clientSd = socket(AF_INET, SOCK_STREAM, 0);
    sendSockAddr.sin_family = AF_INET; 
    sendSockAddr.sin_addr.s_addr = inet_addr(serverIp);
    sendSockAddr.sin_port = htons(port);
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
int c=sizeof(struct sockaddr_in);
		while(1)
		{
		bool flag_connect=false;
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
					
					//char *ack="hello\n";
					//int nw=write(connfd,ack, strlen(ack)); // sendack
					std_msgs::String msg;
					std::stringstream ss;
					ss <<data;
					msg.data = ss.str();
					printf("\n%s",msg.data.c_str());
					//printf("%d\n",total_size);
					pub.publish(msg);
				}
		}

		close(connfd);
		connfd=-1;
		printf("waiting connect again !\n");
		connfd =initTcpIp();
		}
		//printf("closed connection\n");
}
int main(int argc, char *argv[])
{
	ros::init(argc, argv,"bridgeServerCrl");
	ros::NodeHandle nh;
	//pub = nh.advertise<std_msgs::UInt8MultiArray>("callbridgeNode", 100);
	pub = nh.advertise<std_msgs::String>("serverCtrlCallBack", 1);
	ros::Subscriber sub_serverRequest = nh.subscribe("serverRequest",1, ResponseInf_callback);
	ros::Rate loop_rate(10);
    ros::spinOnce();
	connfd =initTcpIp();
	pthread_t thread_server;
	pthread_create(&thread_server, NULL,task_createserver, NULL);
	ros::spin();
	close(listenfd);
}
