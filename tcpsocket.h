#ifndef TCPSOCKET_H
#define TCPSOCKET_H

#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <cstring>
#include <arpa/inet.h>
#include <iostream>
#include <math.h>
#include <time.h>
#include <sstream>
#include <stdio.h>
#include <pthread.h>
#include <string.h>

#include <iostream>

namespace TCP{
const int MAXCONNECTIONS = 5;
const int MAXWAITBUFSIZE = 4096;
const int MAXRECEIVEBUFSIZE = 4096;
const int MAXSENDBUFSIZE = 100;

class TcpSocket
{
public:
	TcpSocket();
	~TcpSocket();

	void start();
	bool isConnected(){return connected;}
	void setPort(int num){port = num;}
	void setIP(std::string addr){ip = addr;}

	void sendData(unsigned char data);
	bool recvData();

private:
	int port;
	std::string ip;
	bool connected;
	int listenSockFD, clientSockFD;
	sockaddr_in server_addr, client_addr;
	char bufWait[MAXWAITBUFSIZE];
	char *ptrRecvBufIndx;
	unsigned char bufRecv[MAXRECEIVEBUFSIZE];
	unsigned char bufSend[MAXSENDBUFSIZE];
	unsigned char lenSend, lenRecv;

	void initSocket();
	void connectSocket();

	pthread_t manager_thread, comm_thread;
	bool manager_thread_run, comm_thread_run;

	static void* manager_func(void *arg);
	static void* comm_func(void *arg);

	bool send_flag, recv_flag, recv_complete;
};
}

#endif // TCPSOCKET_H
