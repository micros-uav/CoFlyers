#ifndef _UDPINTERFACEROBOT
#define _UDPINTERFACEROBOT
#include"UDPInterfaceBase.h"
#include"data_description.h"

namespace robot {

	constexpr int LEN_MAX_BUFFER_SEND = 512;
	constexpr int LEN_MAX_BUFFER_RECV = 512;

	class UDPInterfaceRobot:public socketTools::UDPInterfaceBase
	{
	private:
		
	public:
		UDPInterfaceRobot();
		~UDPInterfaceRobot();

		msg::package *packRecvPtr;
		msg::package *packSendPtr;

		char buffRecv[LEN_MAX_BUFFER_RECV];
		char buffSend[LEN_MAX_BUFFER_SEND];
		int length_send = 0;
		int length_recv = LEN_MAX_BUFFER_RECV;

		struct sockaddr_in adressRequest;
		socklen_t lenAdress = sizeof(struct sockaddr_in);

		// Create connection with the udp interface of simulink
		//void connectToUDPInterfaceExternal(udpPack::UDPInterfacePack &);
		void connectToUDPInterfaceExternal(msg::package& packRecv, msg::package& packSend);

		// Process packRecv received from simulink and send to robot
		virtual void processCommand();
		virtual void processCommandSetMode();
		virtual void processCommandArming();
		virtual void processCommandTakeoff();
		virtual void processCommandHover();
		virtual void processCommandLand();
		virtual void processCommandPosition();
		virtual void processCommandVelocity();
		virtual void processCommandAcceleration();
		virtual void processCommandAttitude();
		virtual void processCommandDefault();


		// Process packSend using data received from robot and prepared to send to simulink
		virtual int generateStatePackage();
		virtual void generateKinematicStatePackage();
		virtual void generateBetteryPackage();
	};


}
#endif