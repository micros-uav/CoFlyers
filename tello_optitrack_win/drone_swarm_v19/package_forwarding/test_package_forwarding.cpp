#include"UDPInterfacePack.h"
#include<thread>

udpPack::UDPInterfacePack udp_pack;

msg::package packRecv, packSend;
int port_target_state = 21000;
int port_target_bat = 20000;

bool flag_run = true;
void init()
{
	udp_pack.setIpPort("127.0.0.1", 21001, "127.0.0.1", port_target_state, msg::LEN_MAX_BUFFER, msg::LEN_MAX_BUFFER);
	udp_pack.initSocket();
	udp_pack.packRecvPtr = &packRecv;
	udp_pack.packSendPtr = &packSend;
}

void run_receive_data()
{
	while (flag_run)
	{
		// Receive command
		int len = udp_pack.recvMsg();
		if (len > 0)
		{

			for (int i = 0; i < packRecv.lenPayload/4; i++)
			{
				printf("%f,", packRecv.msgData[i]);
			}
			printf("\n");
		}
		

		// Send 16 states
		packSend.start = 0xFA;
		packSend.numSequence++;
		packSend.componentId = msg::COMPONENT_TYPE::STATE_TYPE;
		packSend.messageId = msg::MESSAGE_TYPE_STATE_TYPE::KINEMATIC_STATE_TYPE;
		
		int num = msg::getMsgNum(packSend.componentId,packSend.messageId);
		packSend.lenPayload = num * 4;
		for (size_t i = 0; i < num; i++)
		{
			packSend.msgData[i] = i;
		}
		getMillisecond(&packSend.timeStamp);
		packSend.checkNum = 111;

		udp_pack.changeIpPortTarget("127.0.0.1", port_target_state);
		udp_pack.sendMsg();

		// Send bat states
		packSend.start = 0xFA;
		packSend.numSequence++;
		packSend.componentId = msg::COMPONENT_TYPE::STATE_TYPE;
		packSend.messageId = msg::MESSAGE_TYPE_STATE_TYPE::BATTERY_STATE_TYPE;

		num = msg::getMsgNum(packSend.componentId, packSend.messageId);
		packSend.lenPayload = num * 4;
		for (size_t i = 0; i < num; i++)
		{
			packSend.msgData[i] = i+1;
		}
		getMillisecond(&packSend.timeStamp);
		packSend.checkNum = 111;

		udp_pack.changeIpPortTarget("127.0.0.1", port_target_bat);
		udp_pack.sendMsg();
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
	}
}


int main()
{
	init();
	std::thread th1(run_receive_data);
	getchar();
	flag_run = false;
	if (th1.joinable())
	{
		th1.join();
	}
	return 1;
}