#include "tcpsocket.h"

using namespace TCP;

TcpSocket::TcpSocket()
{
	manager_thread_run = false;
	comm_thread_run = false;

	port = 0;

	send_flag = false;
	recv_flag = false;
	recv_complete = false;

	connected = false;
}

TcpSocket::~TcpSocket()
{
	std::cout << "TcpSocket Destructure" << std::endl;
	if(comm_thread_run && manager_thread_run){
		comm_thread_run = false;
		manager_thread_run = false;

		pthread_join(manager_thread, NULL);
	}
	std::cout << "TcpSocket Finished" << std::endl;
}

void TcpSocket::start()
{
	if(port != 0){
		pthread_create(&manager_thread, NULL, manager_func, this);
	}
	else{
		std::cout << "Please check socket port number" << std::endl;
	}
}

void *TcpSocket::manager_func(void *arg)
{
	TcpSocket *pThis = static_cast<TcpSocket*>(arg);

	pThis->manager_thread_run = true;

	pThis->initSocket();

	while(pThis->manager_thread_run){
		pThis->connectSocket();

		if(pThis->connected){
			pthread_create(&pThis->comm_thread, NULL, pThis->comm_func, pThis);

			pthread_join(pThis->comm_thread, NULL);

			pthread_cancel(pThis->comm_thread);
		}
	}

	std::cout << "Finished manager func" << std::endl;

	return nullptr;
}

void *TcpSocket::comm_func(void *arg)
{
	TcpSocket *pThis = static_cast<TcpSocket*>(arg);

	pThis->comm_thread_run = true;

	while(pThis->comm_thread_run){
		if(pThis->send_flag){
			int sendByteLen = send(pThis->clientSockFD, pThis->bufSend, static_cast<size_t>(pThis->lenSend), 0);
			std::cout << "send byte len : " << (int)sendByteLen << std::endl;
			if(sendByteLen > 0){
			}
			else{
				std::cout << "Send error!!!" << std::endl;
				pThis->comm_thread_run = false;
				break;
			}
			pThis->send_flag = false;
		}
		if(pThis->recv_flag){
			pThis->recv_complete = false;
			memset(pThis->bufRecv, 0, TCP::MAXRECEIVEBUFSIZE);
			int recvByteLen = recv(pThis->clientSockFD, pThis->bufRecv, TCP::MAXRECEIVEBUFSIZE, 0);
			std::cout << "recv byte len : " << (int)recvByteLen << std::endl;
			if(recvByteLen > 0 && pThis->bufRecv[0] == '0'){
				pThis->recv_flag = false;
				pThis->recv_complete = true;
			}
//			else if(recvByteLen == -1){
//				pThis->recv_complete = false;
//			}
			else if(recvByteLen == 0){
				std::cout << "Recv error!!!" << std::endl;
				pThis->comm_thread_run = false;
				break;
			}
			pThis->recv_flag = false;
		}
		usleep(2000000);
	}

	std::cout << "Finished comm func" << std::endl;

	return nullptr;
}

void TcpSocket::sendData(unsigned char data)
{
	if(!send_flag){
		memset(bufSend, 0, TCP::MAXSENDBUFSIZE);
		bufSend[0] = data;
		lenSend = 1;
		send_flag = true;
	}
}

bool TcpSocket::recvData()
{
	if(!recv_flag){
		recv_flag = true;
	}

	usleep(2000000);

	return recv_complete;
}

void TcpSocket::initSocket()
{
	listenSockFD = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
	if(listenSockFD < 0){
		std::cout << std::endl << "socket create error" << std::endl;
		return;
	}

	int on = 1;
	if(setsockopt(listenSockFD, SOL_SOCKET, SO_REUSEADDR, reinterpret_cast<const char*>(&on), sizeof(on)) < 0){
		std::cout << std::endl << "set option curLen = 0; error!!" << std::endl;
		return;
	}
	if(setsockopt(listenSockFD, SOL_SOCKET, SO_KEEPALIVE, reinterpret_cast<const char*>(&on), sizeof(on)) < 0){
		std::cout << std::endl << "set option curLen = 0; error!!" << std::endl;
		return;
	}
	timeval tv;
	tv.tv_sec = 1;
	tv.tv_usec = 0;
	if(setsockopt(listenSockFD, SOL_SOCKET, SO_RCVTIMEO, reinterpret_cast<char*>(&tv), sizeof(timeval))){
		std::cout << std::endl << "set option curLen = 0; error!!" << std::endl;
		return;
	}

	// server_addr.sin_addr.s_addr = inet_addr("192.168.0.100");
	server_addr.sin_addr.s_addr = htonl(INADDR_ANY);
	server_addr.sin_family = AF_INET;
	server_addr.sin_port = htons(port);

	std::cout << "server binded" << std::endl;
	std::cout << std::endl << "address : " << inet_ntoa(server_addr.sin_addr) << std::endl;
	std::cout << "port : " << ntohs(server_addr.sin_port) << std::endl << std::endl;

	if(bind(listenSockFD, reinterpret_cast<struct sockaddr*>(&server_addr), sizeof(server_addr)) < 0){
		std::cout << std::endl << "bind error" << std::endl;
		return;
	}
}

void TcpSocket::connectSocket()
{
	std::cout << "server running waiting. waiting client..." << std::endl;

	if(listen(listenSockFD, MAXCONNECTIONS) < 0){
		std::cout << std::endl << "listen error" << std::endl;
	}
	int clientAddrSize = sizeof(client_addr);

	memset(bufWait, 0, MAXWAITBUFSIZE);
	ptrRecvBufIndx = bufWait;
	clientSockFD = accept(listenSockFD, reinterpret_cast<struct sockaddr*>(&client_addr), reinterpret_cast<socklen_t*>(&clientAddrSize));

	if(clientSockFD < 0){
		std::cout << std::endl << "accept error" << std::endl;
		return;
	}

	std::cout << "(" << port << ") " << "client accepted" << std::endl;
	std::cout << "(" << port << ") " << "address : " << inet_ntoa(client_addr.sin_addr) << std::endl;
	std::cout << "(" << port << ") " << "port : " << ntohs(client_addr.sin_port) << std::endl;

	connected = true;
}
