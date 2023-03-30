#include"udp_common/UDPInterfaceBase.h"

namespace socketTools {
	SOCKET netServerInit(const char* ip, unsigned short port, int sizeBuffSend, int sizeBuffRecv)
	{
		SOCKET socketMy;
		struct sockaddr_in addLocal;

		socketMy = socket(AF_INET, SOCK_DGRAM, 0);
		//if (socketMy == INVALID_SOCKET) {
		if (socketMy == SOCKET_ERROR) {
			printf("Cannot create socket.\n");
			//goto err;
			return (SOCKET)SOCKET_ERROR;
		}

		bool bOpt = true;
		//set UDP to Broadcast  mode	
		setsockopt(socketMy, SOL_SOCKET, SO_BROADCAST, (char*)&bOpt, sizeof(bOpt));
		bOpt = true;
		setsockopt(socketMy, SOL_SOCKET, SO_REUSEADDR, (char*)&bOpt, sizeof(bOpt));
		setsockopt(socketMy, SOL_SOCKET, SO_SNDBUF, (char*)&sizeBuffSend, sizeof(sizeBuffSend));
		setsockopt(socketMy, SOL_SOCKET, SO_RCVBUF, (char*)&sizeBuffRecv, sizeof(sizeBuffRecv));

		// memset((void *)&addLocal, 0, sizeof(addLocal));
		// addLocal.sin_family = AF_INET;
		// addLocal.sin_port = htons(port);
		// addLocal.sin_addr.s_addr = inet_addr(ip);

		addLocal = addressInit(ip, port);

		/* Bind to port and interface */
		if (bind(socketMy, (struct sockaddr *)&addLocal, sizeof(addLocal)) == SOCKET_ERROR)
		{
			printf("Cannot bind with %s:%d.\n", ip, port);
			//goto err;
			return (SOCKET)SOCKET_ERROR;
		}


		return socketMy;
		//err:
		//    return (SOCKET)SOCKET_ERROR;
	}
	SOCKADDR_IN addressInit(const char* ip, unsigned short port)
	{
		struct sockaddr_in address;

		memset((void *)&(address), 0, sizeof(address));
		address.sin_family = AF_INET;
		address.sin_port = htons(port);

		if (strcmp(ip, "255.255.255.255") == 0) {
			address.sin_addr.s_addr = htonl(INADDR_BROADCAST);

		}
		else {
			//printf("test:%s:%d\n",ip,port);
			if ((address.sin_addr.s_addr = inet_addr(ip)) == INADDR_NONE) {
				address.sin_addr.s_addr = inet_addr("127.0.0.1");
				printf("Can not process address %s.\n", ip);
			}
			else {}
		}
		return address;
	}
	// UDPInterface::UDPInterface(){
	// }
	UDPInterfaceBase::UDPInterfaceBase()
	{
	}
	UDPInterfaceBase::UDPInterfaceBase(const char*ipLocal, int portLocal, const char*ipTarget, int portTarget,
		int sizeBuffSend, int sizeBuffRecv)
	{
		memcpy(this->ipLocal, ipLocal, 32);
		// this->ipLocal = ipLocal;
		this->portLocal = portLocal;
		// this->ipTarget = ipTarget;
		memcpy(this->ipTarget, ipTarget, 32);
		this->portTarget = portTarget;
		this->sizeBuffSend = sizeBuffSend;
		this->sizeBuffRecv = sizeBuffRecv;
	}
	void UDPInterfaceBase::setIpPort(const char*ipLocal, int portLocal, const char*ipTarget, int portTarget,
		int sizeBuffSend, int sizeBuffRecv)
	{
		memcpy(this->ipLocal, ipLocal, 32);
		// this->ipLocal = ipLocal;
		this->portLocal = portLocal;
		// this->ipTarget = ipTarget;
		memcpy(this->ipTarget, ipTarget, 32);
		this->portTarget = portTarget;
		this->sizeBuffSend = sizeBuffSend;
		this->sizeBuffRecv = sizeBuffRecv;
	}

	void UDPInterfaceBase::changeIpPortTarget(const char* ipTarget, int portTarget)
	{
		memcpy(this->ipTarget, ipTarget, 32);
		this->portTarget = portTarget;
		this->sInfo.addTarget = addressInit(ipTarget, portTarget);
	}

	void UDPInterfaceBase::changePortTarget(int portTarget)
	{
		this->sInfo.addTarget = addressInit(this->ipTarget, portTarget);
	}

	bool UDPInterfaceBase::initSocket()
	{
#if defined(__WIN32__)||defined(WIN32)||defined(_WIN32)
		WSADATA wsd;
		if (WSAStartup(MAKEWORD(2, 2), &wsd))
		{
			printf("WSAStartup failed!\n");
			exit(1);
		}
#endif
		this->sInfo.addTarget = addressInit(ipTarget, portTarget);
		
		this->sInfo.socketMy = netServerInit(ipLocal, portLocal, sizeBuffSend, sizeBuffRecv);
		this->sInfo.success = false;
		if (this->sInfo.socketMy != SOCKET_ERROR) {
			nonblockingsocket(this->sInfo.socketMy);
			this->sInfo.success = true;
		}
		
		return this->sInfo.success;
	}

	UDPInterfaceBase::~UDPInterfaceBase()
	{
		closesocket(this->sInfo.socketMy);
		// Calling multiple times in Simulink will result in an error 
//#if defined(__WIN32__)||defined(WIN32)||defined(_WIN32)
//		WSACleanup();
//#endif
	}

	int UDPInterfaceBase::receiveData(char*buffRecv, int lenMaxBuffer, struct sockaddr_in &addressRequest, socklen_t &lenRequest)
	{
		int lenReceive = -1;
		if (this->sInfo.success)
		{
			lenReceive = recvfrom(this->sInfo.socketMy, buffRecv, lenMaxBuffer, 0,
				(struct sockaddr *)&addressRequest, &lenRequest);
		}
		return lenReceive;

	}

	ssize_t UDPInterfaceBase::sendData(const char*buffSend, const int lenBuff)
	{
		ssize_t retval = -1;
		if (this->sInfo.success)
		{
			retval = sendto(this->sInfo.socketMy, buffSend, lenBuff, MSG_NOSIGNAL, (SOCKADDR*)&(this->sInfo.addTarget), sizeof(SOCKADDR));
		}
		return retval;
	}
}