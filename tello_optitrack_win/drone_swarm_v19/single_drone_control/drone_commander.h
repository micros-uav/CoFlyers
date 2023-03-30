#pragma once
#include"drone_controller.h"
#include"UDPInterfaceTello.h"
#include"UDPInterfacePack.h"
#include <thread>
namespace drone_commander
{
	class drone_commander
	{
	public:
		drone_commander();
		~drone_commander();

		/*char*ip_local = (char*)"192.168.1.3";
		char*ip_drone = (char *)"192.168.1.33";*/

		unsigned int ID = 0;
		float height_takeoff = 0.7;
		// State
		double timestamp_state;
		data_description::state_12 state;
		data_description::state_12 state_former;
		data_description::state_12 state_save;
		float bat = 0.0f;
		float c_filter = 1.0f;
		
		void update_state_simple_by_mocap(const data_description::state_rigidbody*, float delta_t);
		void updata_state_simple_by_state_simple(const data_description::state_12&);
		
		bool is_armed();

		// 
		void stop();
		void run_receice_state_from_tello(); //Note that: the states from all tellos receive in only one port 8890. 
		void run_process_high_command();
		void run_external_control();

		// High commander
		void arming();
		void takeoff();
		void land_without_ep(); // Without external positioning.
		void hover();
		void setpoint_position(float x_d, float y_d, float z_d, float yaw_d);					// lower control mode: 0
		void setpoint_velocity(float vx_d, float vy_d, float vz_d, float yaw_rate_d);          // lower control mode: 1
		void setpoint_velocity_horizontal(float vx_d, float vy_d, float z_d, float yaw_rate_d);// lower control mode: 2
		//void setpoint_position_velocity(float x, float y, float z, float vx, float vy, float vz, float yaw);// lower control mode: 2, unuse
		void setpoint_attitude(float yaw_rate_d, float roll_d, float pitch_d, float thrust_d); // angle:degree, thrust_d:[-100,100]
		//init
		//void init_ip_local(const char* ip_local);
		bool init_drone_udp(const char*ip_local_drone, const char*ip_target_drone, unsigned short int id);
		bool init_controller(const char* file_name);
		bool init_udp_external(const char*ip_local_external,const char*ip_target_external, int port_target_state, int port_target_bat, int port_local_command);

		// Sample time
		float dt_external = 1.0f / 100.0f;
		float time_out_external = 2.0f; //unuse


		// Tune pid params from txt
		void activate_tune_controller();
		void deactivate_tune_controller();

		//
		tello::UDPInterfaceTello drone_udp; 
		//tello::UDPInterfaceTello*drone_udp_ptr = nullptr; //Public for receive bat from all tellos
		//void set_udp_drone(tello::UDPInterfaceTello& drone_udp);
		//bool set_udp_drone(const char*ip_target_drone); //only support for controlling one tello

		void set_flag_armed(bool flag);
	private:
		//char ip_local[32] = "127.0.0.1";
		char ip_target_drone[32] = "0.0.0.0";
		//char ip_target[32] = "127.0.0.1";


		unsigned short int port_local_drone = 8890;
		unsigned short int port_target_drone = 8889;

		//
		bool flag_run = true;
		bool flag_armed = false;
		bool flag_land = true;
		bool flag_trigger = false;

		// UDP, drone
		msg::package packrecv, packsend;
		std::thread thread_receice_state_from_tello; //Thread
		void callback_receice_state_from_tello();

		// Controller
		drone_controller::drone_controller controller;
		float yaw_rate_d = 0.0f;
		data_description::state_12 desire;
		data_description::attitude_command att_cmd;
		unsigned int high_command_mode = msg::MESSAGE_TYPE_COMMAND_TYPE::ARMING_TYPE; // 1: armming, 2:takeoff, 3:land
		unsigned int high_command_mode_former= msg::MESSAGE_TYPE_COMMAND_TYPE::SET_MODE_TYPE;
		std::thread thread_process_high_command;	//Thread
		void callback_process_high_command();
		
		// External control
		udpPack::UDPInterfacePack udp_pack_external;
		msg::package pack_recv_command, pack_send_state_bat;
		int port_target_state = 21000, port_target_bat = 20000, port_local_command = 21001;
		std::thread thread_udp_external;	//Thread
		void callback_udp_external();
		unsigned int command_type_former = msg::MESSAGE_TYPE_COMMAND_TYPE::SET_MODE_TYPE;

		// process
		void process_arming();
		void process_takeoff();
		void process_land();
		void process_hover();
		void process_setpoint();
		void process_setpoint_position();
		void process_velocity();
		void process_velocity_horizontal();
		void process_setpoint_attitude();
		//void process_setpoint_position_velocity();
	};
}