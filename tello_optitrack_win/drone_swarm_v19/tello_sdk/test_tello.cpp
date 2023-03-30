//#include"UDPInterfacetello.h"
//
//#include<thread>
//
//tello::UDPInterfaceTello my_tello;
//msg::package packrecv, packsend;
//
//bool flag_run = true;
//
//void receive_tello_state_data()
//{
//	// Receive state from tello
//	while (flag_run)
//	{
//		int len = my_tello.generateStatePackage();
//		if (len > 0)
//		{
//			printf("Timestamp: %3.2lf\n", my_tello.packSendPtr->timeStamp);
//			printf("mid:%f;x:%f;y:%f;z:%f;mpry:%f,%f,%f;pitch:%f;roll:%f;yaw:%f;vgx:%f;vgy:%f;vgz:%f;templ:%f;temph:%f;tof:%f;h:%f;bat:%f;baro:%f;time:%f;agx:%f;agy:%f;agz:%f;\n",
//				my_tello.state_from_tello[0], my_tello.state_from_tello[1], my_tello.state_from_tello[2], my_tello.state_from_tello[3], my_tello.state_from_tello[4], my_tello.state_from_tello[5], my_tello.state_from_tello[6], my_tello.state_from_tello[7], my_tello.state_from_tello[8], my_tello.state_from_tello[9], my_tello.state_from_tello[10], my_tello.state_from_tello[11], my_tello.state_from_tello[12], my_tello.state_from_tello[13], my_tello.state_from_tello[14], my_tello.state_from_tello[15],
//				my_tello.state_from_tello[16], my_tello.state_from_tello[17], my_tello.state_from_tello[18], my_tello.state_from_tello[19], my_tello.state_from_tello[20], my_tello.state_from_tello[21], my_tello.state_from_tello[22]);
//
//		}
//		std::this_thread::sleep_for(std::chrono::microseconds(10));
//	}
//	return;
//}
//
//void print_test()
//{
//	while (flag_run)
//	{
//		printf("111111111\n");
//		std::this_thread::sleep_for(std::chrono::microseconds(100));
//	}
//}
//
//int main()
//{
//	my_tello.setIpPort("192.168.2.3", 8890, "192.168.2.4", 8889, tello::LEN_MAX_BUFFER_SEND, tello::LEN_MAX_BUFFER_RECV);
//	my_tello.initSocket();
//
//	my_tello.packRecvPtr = &packrecv;
//	my_tello.packSendPtr = &packsend;
//	my_tello.packSendPtr->componentId = msg::COMPONENT_TYPE::STATE_TYPE;
//	my_tello.packSendPtr->messageId = msg::MESSAGE_TYPE_STATE_TYPE::BATTERY_STATE_TYPE;
//
//	//HANDLE hThread1 = CreateThread(NULL, 0, receive_tello_state_data, NULL, 0, NULL);
//
//	std::thread t1(receive_tello_state_data);
//
//	std::thread t2(print_test);
//
//	/// Arm
//	while (my_tello.packSendPtr->msgData[0] < 1)
//	{
//		printf("Arming!\n");
//		packrecv.start = 0xFA;
//		packrecv.componentId = msg::COMPONENT_TYPE::COMMAND_TYPE;
//		packrecv.messageId = msg::MESSAGE_TYPE_COMMAND_TYPE::ARMING_TYPE;
//		packrecv.lenPayload = 0;
//		my_tello.processCommand();
//
//		std::this_thread::sleep_for(std::chrono::microseconds(1000));
//	}
//
//	/// Take off
//	float tof_ground = my_tello.tof;
//	while (my_tello.tof - tof_ground < 30)
//	{
//		printf("Take off!\n");
//		packrecv.start = 0xFA;
//		packrecv.componentId = msg::COMPONENT_TYPE::COMMAND_TYPE;
//		packrecv.messageId = msg::MESSAGE_TYPE_COMMAND_TYPE::TAKEOFF_TYPE;
//		packrecv.lenPayload = 0;
//		my_tello.processCommand();
//
//		std::this_thread::sleep_for(std::chrono::microseconds(1000));
//	}
//
//	std::this_thread::sleep_for(std::chrono::microseconds(5000));
//
//	/// Land
//	float tof_hovor = my_tello.tof;
//	while (tof_hovor - my_tello.tof < 10)
//	{
//		printf("Land!\n");
//		packrecv.start = 0xFA;
//		packrecv.componentId = msg::COMPONENT_TYPE::COMMAND_TYPE;
//		packrecv.messageId = msg::MESSAGE_TYPE_COMMAND_TYPE::LAND_TYPE;
//		packrecv.lenPayload = 0;
//		my_tello.processCommand();
//
//		std::this_thread::sleep_for(std::chrono::microseconds(3000));
//	}
//
//	std::this_thread::sleep_for(std::chrono::seconds(3));
//
//
//	flag_run = false;
//	t1.join();
//	t2.join();
//
//	return 0;
//}