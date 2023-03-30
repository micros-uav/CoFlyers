//#include"NatNetClass_my.h"
//#include"drone_commander.h"
//#include<thread>
//#include<ctime>
//
//unsigned int id_drone = 2;
//const char* ip_server = "192.168.1.2";
//const char* ip_client = "192.168.1.3";
//const char* ip_drone = "192.168.1.33";
////const char* ip_server = "127.0.0.1";
////const char* ip_client = "127.0.0.1";
////const char* ip_drone = "127.0.0.1";
//////////////////////////////Optitrack///////////////////////////
//double time_print = 0.0;
//double timestamp_pre = 0.0;
//double dtimestamp = 0.0;
//void get_timestamp_handler(const double timestamp);
//void process_rigid_data_handler(const unsigned int id, const data_description::state_rigidbody* state_ptr);
//void receive_optitrack_data();
///////////////////////////Commander//////////////////////////
//drone_commander::drone_commander drone;
//void init_commander();
////////////////////////////External Commander/////////////////
//const char* ip_target = "192.168.1.3";
////const char* ip_target = "127.0.0.1";
//int port_target_state = 21000, port_target_bat = 20000, port_local_command = 21001;
//
//int main()
//{
//	////////////////////////////Optitrack///////////////////////////
//	receive_optitrack_data();
//	//std::thread hThread1(receive_optitrack_data);
//	std::this_thread::sleep_for(std::chrono::milliseconds(1000));
//
//	// Waiting for optitrack
//	printf("Waiting for external location......\n");
//	while (fabs(drone.state.pos.x) < 0.00001)
//	{
//		std::this_thread::sleep_for(std::chrono::milliseconds(10));
//	}
//	printf("Connected to external location......\n");
//
//
//	
//	//////////////////////////Commander/////////////////////////
//	init_commander();
//	drone.run_receice_state_from_tello();
//	drone.run_process_high_command();
//	drone.run_external_control();
//	//drone.activate_tune_controller();
//	
//
//	bool bExit = false;
//	int c = 0;
//	while (c = _getch())
//	{
//		printf("2222");
//		switch (c)
//		{
//		case 'q':
//
//			bExit = true;
//			break;
//		}
//		if (bExit)
//			break;
//	}
//	drone.stop();
//	natnet::stop();
//	/*if (hThread1.joinable())
//	{
//		hThread1.join();
//	}*/
//	return 0;
//}
//
//
//////////////////////////////Optitrack///////////////////////////
//void get_timestamp_handler(const double timestamp)
//{
//
//	dtimestamp = timestamp - timestamp_pre;
//	timestamp_pre = timestamp;
//	time_print += dtimestamp;
//
//	//printf("Timestamp : %3.2lf\n", timestamp);
//	//printf("dTimestamp : %3.8lf\n", dtimestamp)
//	;
//	drone.timestamp_state = timestamp;
//}
//void process_rigid_data_handler(const unsigned int id, const data_description::state_rigidbody* state_ptr)
//{
//	if (id == id_drone)
//	{
//		drone.update_state_simple_by_mocap(state_ptr, (float)(1.0 / 120.0));
//	}
//	/*printf("Rigid Body [ID=%d]\n", id);
//	printf("\tx\ty\tz\tvx\tvy\tvz\tpitch\troll\tyaw\n");
//	printf("\t%3.2f\t%3.2f\t%3.2f\t%3.2f\t%3.2f\t%3.2f\t%3.2f\t%3.2f\t%3.2f\n",
//		drone.state.pos.x,
//		drone.state.pos.y,
//		drone.state.pos.z,
//		drone.state.vel.x,
//		drone.state.vel.y,
//		drone.state.vel.z,
//		drone.state.angle.pitch,
//		drone.state.angle.roll,
//		drone.state.angle.yaw);*/
//}
//void receive_optitrack_data()
//{
//	natnet::set_process_rigid_data_handler(process_rigid_data_handler);
//	natnet::set_timestamp_handler(get_timestamp_handler);
//	natnet::activate(ip_server, ip_client);
//
//	//bool bExit = false;
//	//while (const int c = _getch())
//	//{
//	//	printf("1111");
//	//	switch (c)
//	//	{
//	//	case 'q':
//
//	//		bExit = true;
//	//		break;
//	//	}
//	//	if (bExit)
//	//		break;
//	//}
////	natnet::stop();
//
//	return;
//}
//
/////////////////////////////Commander///////////////////////////
//void init_commander()
//{
//	//drone.initialize(ip_client, ip_drone, "D:\\StudyMaster\\myCppCode\\vs2017Projects\\drone_swarm\\drone_swarm\\x64\\Debug\\config\\params_controller\\params_controller.txt");
//	//drone.init_ip_local(ip_client);
//	drone.init_controller("D:\\StudyMaster\\myCppCode\\vs2017Projects\\drone_swarm\\drone_swarm\\x64\\Debug\\config\\params_controller\\params_controller.txt");
//	drone.init_drone_udp(ip_client,ip_drone,0);
//	drone.ID = id_drone;
//	drone.init_udp_external(ip_client,ip_target, port_target_state, port_target_bat, port_local_command);
//}