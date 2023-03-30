#ifndef _UDPINTERFACETELLO
#define _UDPINTERFACETELLO

#include"UDPInterfaceRobot.h"
namespace tello
{
	constexpr int LEN_MAX_BUFFER_SEND = 64;
	constexpr int LEN_MAX_BUFFER_RECV = 512;

	class UDPInterfaceTello:public robot::UDPInterfaceRobot
	{
	public:
		UDPInterfaceTello();
		~UDPInterfaceTello();

		virtual void processCommand();
		virtual void processCommandArming();
		virtual void processCommandTakeoff();
		virtual void processCommandHover();
		virtual void processCommandLand();
		virtual void processCommandAttitude();
		virtual void processCommandDefault();

		virtual void generateBetteryPackage();

		float state_from_tello[23]{};

		char command_rc[LEN_MAX_BUFFER_SEND]{};
		float gain_roll = 1000;
		float gain_pitch = 1000;
		float gain_throttle = 100;
		float gain_yaw_rate = -100;

		int command_messageID_pre = -1;

		double requst_time_last;

		double requst_time_last_now;
		

		float tof = 0.0f;
	private:

	};
}

#endif