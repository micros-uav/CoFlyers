//#include"NatNetClass_my.h"
////#include"drone_controller.h"
//#include"drone_commander.h"
//#include<thread>
//#include<ctime>
//
//unsigned short int id_drone = 2;
//const char * ip_server = "192.168.1.2";
//const char * ip_client = "192.168.1.3";
//const char * ip_drone = "192.168.1.33";
//////////////////////////////Optitrack///////////////////////////
//double time_print = 0.0;
//double timestamp_pre = 0.0;
//double dtimestamp = 0.0;
//void get_timestamp_handler(const double timestamp);
//void process_rigid_data_handler(const unsigned int id, const data_description::state_rigidbody* state_ptr);
//void receive_optitrack_data();
//
///////////////////////////Commander//////////////////////////
//drone_commander::drone_commander drone;
//void init_commander();
//void circle_motion();
//void box_motion();
//
//
//int main()
//{
//	////////////////////////////Optitrack///////////////////////////
//	std::thread hThread1(receive_optitrack_data);
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
//	//////////////////////////Commander/////////////////////////
//	init_commander();
//	drone.run_receice_state_from_tello();
//	drone.run_process_high_command();
//
//	//Only for display in simulink
//	drone.init_udp_external(ip_client,ip_client, 21000, 20000, 21001);
//	drone.run_external_control();
//
//	// Arm
//	drone.arming();
//	while (!drone.is_armed())
//	{
//		std::this_thread::sleep_for(std::chrono::milliseconds(1000));
//	}
//	// Take off
//	drone.takeoff();
//	std::this_thread::sleep_for(std::chrono::milliseconds(5000));
//	// Circle, position control
//	//circle_motion();
//	// Box motion, velocity control
//	//drone.setpoint_position(1.0, 0.0, 0.5, 0.0);
//	box_motion();
//	//std::this_thread::sleep_for(std::chrono::milliseconds(5000));
//	// Land
//	drone.land();
//	std::this_thread::sleep_for(std::chrono::milliseconds(5000));
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
//
//	drone.stop();
//	hThread1.join();
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
//		;
//	drone.timestamp_state = timestamp;
//}
//
//void process_rigid_data_handler(const unsigned int id, const data_description::state_rigidbody* state_ptr)
//{
//	if (id == id_drone)
//	{
//		drone.update_state_simple_by_mocap(state_ptr, (float)(1.0 / 120.0));
//	}
//
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
//
//void receive_optitrack_data()
//{
//	natnet::set_process_rigid_data_handler(process_rigid_data_handler);
//	natnet::set_timestamp_handler(get_timestamp_handler);
//	natnet::activate(ip_server, ip_client);
//
//	bool bExit = false;
//	while (const int c = _getch())
//	{
//		printf("1111");
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
//
//	natnet::stop();
//
//	return;
//}
//
//
/////////////////////Commander/////////////////////////
//void init_commander()
//{
//	//drone.init_ip_local(ip_client);
//	drone.init_controller("D:\\StudyMaster\\myCppCode\\vs2017Projects\\drone_swarm\\drone_swarm\\x64\\Debug\\config\\params_controller\\params_controller.txt");
//	drone.init_drone_udp(ip_client,ip_drone, 0);
//	drone.ID = id_drone;
//}
//
//void circle_motion()
//{
//	float r = 1.0f;
//	float T = 10.0f;
//	float pi = 3.1415926f;
//	std::chrono::system_clock::time_point ts = std::chrono::system_clock::now();
//	std::chrono::system_clock::time_point te = std::chrono::system_clock::now();
//	long long delta_T = std::chrono::duration_cast<std::chrono::microseconds>(te - ts).count();
//	while (delta_T < 10000000)
//	{
//		float t = (float)((double)delta_T / 1000000.0);
//		float x_d = r * cos(2.0f*pi*(t / T));
//		float y_d = r * sin(2.0f*pi*(t / T));
//		float z_d = 0.5;
//		float yaw_d = 0.0;
//		drone.setpoint_position(x_d, y_d, z_d, yaw_d);
//		printf("%f,%f,%f,%f\n", x_d, y_d, z_d, yaw_d);
//		te = std::chrono::system_clock::now();
//		delta_T = std::chrono::duration_cast<std::chrono::microseconds>(te - ts).count();
//
//		std::this_thread::sleep_for(std::chrono::milliseconds(10));
//	}
//	drone.hover();
//}
//void box_motion()
//{
//	float x_min = -0.5f;
//	float x_max = 0.5f;
//	float y_min = -0.5f;
//	float y_max = 0.5f;
//
//	float speed = 0.4f;
//	float vx_d = speed;
//	float vy_d = 0.0f;
//	float z_d = 0.5f;
//	float yaw_rate_d = 0.0f;
//	int count = 0;
//
//
//	std::chrono::system_clock::time_point ts = std::chrono::system_clock::now();
//	std::chrono::system_clock::time_point te = std::chrono::system_clock::now();
//	long long delta_T = std::chrono::duration_cast<std::chrono::microseconds>(te - ts).count();
//	while (delta_T < 20000000)
//	{
//		if (drone.state.pos.x < x_min)
//		{
//			vx_d = speed; count++;
//		}
//		else if (drone.state.pos.x > x_max)
//		{
//			vx_d = -speed; count++;
//		}
//		if (drone.state.pos.y < y_min)
//		{
//			vy_d = speed; count++;
//		}
//		else if (drone.state.pos.y > y_max)
//		{
//			vy_d = -speed; count++;
//		}
//		if (count == 2)
//		{
//			vx_d *= 0.707f;
//			vy_d *= 0.707f;
//		}
//		count = 0;
//
//		drone.setpoint_velocity_horizontal(vx_d, vy_d, z_d, yaw_rate_d);
//		printf("position: %f,%f,%f,%f\n", drone.state.pos.x, drone.state.pos.x, drone.state.pos.z, drone.state.angle.yaw);
//		printf("command: %f,%f,%f,%f\n", vx_d, vy_d, z_d, yaw_rate_d);
//
//		te = std::chrono::system_clock::now();
//		delta_T = std::chrono::duration_cast<std::chrono::microseconds>(te - ts).count();
//
//		std::this_thread::sleep_for(std::chrono::milliseconds(10));
//	}
//	drone.hover();
//}