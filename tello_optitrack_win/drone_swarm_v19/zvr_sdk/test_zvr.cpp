//#include<iostream>
//#include"zvr_class_my.h"
//void get_timestamp_handler(const double timestamp);
//void process_rigid_data_handler(const unsigned int id, const data_description::state_rigidbody* state_ptr);
//
//int main()
//{
//	mcs::Motion_capture_system* temp = &zvr::mcs_zvr_handle;
//
//	temp->activate("239.8.192.168", "192.168.200.11");
//	temp->set_timestamp_handler(get_timestamp_handler);
//	temp->set_process_rigid_data_handler(process_rigid_data_handler);
//	getchar();
//	temp->stop();
//	std::cout << "ZVR stoped"<<std::endl;
//	getchar();
//	return 0;
//}
//
//void get_timestamp_handler(const double timestamp)
//{
//	std::cout << "Time: " << timestamp << std::endl;
//}
//
//void process_rigid_data_handler(const unsigned int id, const data_description::state_rigidbody* state_ptr)
//{
//	std::cout << "ID: " << id << ", "
//		<< "x: " << state_ptr->pos.x << ", "
//		<< "y: " << state_ptr->pos.y << ", "
//		<< "z: " << state_ptr->pos.z << ", "
//		<< "qw: " << state_ptr->q.qw << ", "
//		<< "qx: " << state_ptr->q.qx << ", "
//		<< "qy: " << state_ptr->q.qy << ", "
//		<< "qz: " << state_ptr->q.qz << ", "<<std::endl;
//}