//#include"NatNetClass_my.h"
//#include<thread>
//
//double time_print = 0.0;
//
//double timestamp_pre = 0.0;
//double dtimestamp = 0.0;
//void get_timestamp_handler(const double timestamp)
//{
//	
//	dtimestamp = timestamp - timestamp_pre;
//	timestamp_pre = timestamp;
//	time_print += dtimestamp;
//
//	printf("Timestamp : %3.2lf\n", timestamp);
//	printf("dTimestamp : %3.8lf\n", dtimestamp);
//	
//}
//
//void process_rigid_data_handler(const unsigned int id, const data_description::state_rigidbody* state_ptr)
//{
//
//	printf("Rigid Body [ID=%d]\n", id);
//	printf("\tx\ty\tz\tqx\tqy\tqz\tqw\n");
//	printf("\t%3.2f\t%3.2f\t%3.2f\t%3.2f\t%3.2f\t%3.2f\t%3.2f\n",
//		state_ptr->pos.x,
//		state_ptr->pos.y,
//		state_ptr->pos.z,
//		state_ptr->q.qx,
//		state_ptr->q.qy,
//		state_ptr->q.qz,
//		state_ptr->q.qw);
//}
//
//const char * ip_server = "192.168.1.2";
//const char * ip_client = "192.168.1.3";
//
//void receive_optitrack_data()
//{
//	natnet::set_process_rigid_data_handler(process_rigid_data_handler);
//	natnet::set_timestamp_handler(get_timestamp_handler);
//	natnet::activate(ip_server, ip_client);
//
//
//	bool bExit = false;
//	while (const int c = getch())
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
//int main()
//{
//	std::thread hThread1(receive_optitrack_data);
//
//	bool bExit = false;
//	int c = 0;
//	while (c = getch())
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
//	hThread1.join();
//	
//	return 0;
//}