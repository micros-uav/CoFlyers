#pragma once
#include"drone_commander.h"
#include"configuration.h"
#include<thread>

namespace drone_swarm
{
	class drone_swarm
	{
	public:
		drone_swarm();
		~drone_swarm();

		void init_params();
		void run_process_high_command_s();
		void run_external_control_s();
		void run_receive_state_from_tellos(); /////////////////////////////No modular//////////////////////////
		void stop();

		config::params_swarm params;
		drone_commander::drone_commander* drones = nullptr;
	private:
		bool flag_run = true;
		std::thread thread_receive_state_from_tellos;
		void callback_receive_state_from_tellos();
	};



	void get_timestamp_handler(const double timestamp);
	void process_rigid_data_handler(const unsigned int id, const data_description::state_rigidbody* state_ptr);
	void run_receive_optitrack_data(drone_swarm &);
	void mocap_stop();
}