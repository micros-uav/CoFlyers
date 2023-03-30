#ifndef _UDPINTERFACEBASE
#define _UDPINTERFACEBASE
#include <iostream>
#include <string.h>
#if defined(__WIN32__)||defined(WIN32)||defined(_WIN32)
#include <winsock2.h>
#pragma comment(lib,"ws2_32.lib")
#define nonblockingsocket(s) {unsigned long ctl = 1;ioctlsocket( s, FIONBIO, &ctl );}

typedef int socklen_t;
typedef int ssize_t;
#define MSG_NOSIGNAL 0
#define getMillisecond(tsPtr) {*tsPtr = GetTickCount64();} 
//#define _WINSOCK_DEPRECATED_NO_WARNINGS 

#else

#include<netinet/in.h>
#include<unistd.h>
#include<math.h>
#include<arpa/inet.h>
#include<fcntl.h>
#include<sys/socket.h>
#include<sys/time.h>
typedef long SOCKET;
typedef sockaddr_in SOCKADDR_IN;
typedef sockaddr SOCKADDR;
#define INVALID_SOCKET -1
#define SOCKET_ERROR -1
#define nonblockingsocket(s) { int flags = fcntl(s,F_GETFL,0);fcntl(s, F_SETFL, flags | O_NONBLOCK);}
#define closesocket(s) close(s)
#define getMillisecond(tsPtr) {struct timeval tp; gettimeofday(&tp,NULL); *tsPtr=((double)tp.tv_sec+(double)tp.tv_usec/CLOCKS_PER_SEC);} 
#endif

namespace socketTools {

	typedef struct SocketInfomation
	{
		SOCKET socketMy;
		bool success;
		struct sockaddr_in addTarget;
	} SInfo;
	SOCKET netServerInit(const char* ip, unsigned short port, int sizeBuffSend, int sizeBuffRecv);
	SOCKADDR_IN addressInit(const char* ip, unsigned short port);

	class UDPInterfaceBase
	{
	private:
		SInfo sInfo;
		char ipLocal[32] = "127.0.0.1";
		int portLocal = 0;
		char ipTarget[32] = "127.0.0.1";
		int portTarget = 25000;
		int sizeBuffSend = 8192;
		int sizeBuffRecv = 8192;

	public:
		//UDPInterface();
		UDPInterfaceBase();
		UDPInterfaceBase(const char*ipLocal, int portLocal, const char*ipTarget, int portTarget,
			int sizeBuffSend, int sizeBuffRecv);
		void setIpPort(const char*ipLocal, int portLocal, const char*ipTarget, int portTarget,
			int sizeBuffSend, int sizeBuffRecv);
		void changeIpPortTarget(const char* ipTarget, int portTarget);
		void changePortTarget(int portTarget);

		bool initSocket();
		~UDPInterfaceBase();
		int receiveData(char*buffRecv, int lenMaxBuffer, struct sockaddr_in &addressRequest, socklen_t &lenRequest);
		ssize_t sendData(const char*buffSend, const int lenBuff);
	};

}
#endif