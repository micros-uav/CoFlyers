//#include"param_config.h"
//#include"pid_controller.h"
//#include<iostream>
//#include"drone_controller.h"
//
//#include"configuration.h"
//
//int main()
//{
//	/*pid::pid_controller my_pid = pid::pid_controller();
//	my_pid.proportion = 1.0;
//	my_pid.integral = 0.0;
//	my_pid.derivative = 0.0;
//	my_pid.output_max = 0.01;
//
//	float desire = 1.0;
//	float now = 0.0;
//	float error = desire - now;
//	for (int i = 0; i < 1000; i++)
//	{
//		error = desire - now;
//		now = now + my_pid.update(error)*my_pid.dt;
//		std::cout << now << std::endl;
//	}*/
///*
//	config::params_controller p_con;
//	config::get_params_controller_from_txt(p_con,"params_controller/params_controller.txt");*/
//	
//	drone_controller::drone_controller controller;
//	controller.set_params_from_txt("params_controller/params_controller.txt");
//	controller.print_params();
//	getchar();
//	return 0;
//}