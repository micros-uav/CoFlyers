#pragma once
namespace  config
{
	struct params_controller
	{
		float delta_t = 0.01f;

		// Position control
		float POS_X_PID_KP = 2.0f;
		float POS_X_PID_KI = 0.0f;
		float POS_X_PID_KD = 0.0f;
		float POS_X_PID_CD = 1.0f;

		float POS_Y_PID_KP = 2.0f;
		float POS_Y_PID_KI = 0.0f;
		float POS_Y_PID_KD = 0.0f;
		float POS_Y_PID_CD = 1.0f;

		float POS_Z_PID_KP = 2.0f;
		float POS_Z_PID_KI = 0.5f;
		float POS_Z_PID_KD = 0.0f;
		float POS_Z_PID_CD = 1.0f;


		// Velocity control
		float VEL_X_PID_KP = 25.0f;
		float VEL_X_PID_KI = 1.0f;
		float VEL_X_PID_KD = 0.0f;
		float VEL_X_PID_CD = 1.0f;

		float VEL_Y_PID_KP = 25.0f;
		float VEL_Y_PID_KI = 1.0f;
		float VEL_Y_PID_KD = 0.0f;
		float VEL_Y_PID_CD = 1.0f;

		float VEL_Z_PID_KP = 25.0f;
		float VEL_Z_PID_KI = 15.0f;
		float VEL_Z_PID_KD = 0.0f;
		float VEL_Z_PID_CD = 1.0f;

		// Yaw control
		float YAW_PID_KP = 2.0f;
		float YAW_PID_KI = 0.3f;
		float YAW_PID_KD = 0.1f;
		float YAW_PID_CD = 1.0f;
		float YAW_PID_I_LIMIT = 360.0f;

		// PID Limit
		float LIMIT_vel_x = 0.5f;
		float LIMIT_vel_y = 0.5f;
		float LIMIT_vel_z = 0.5f;
		float LIMIT_vel_h = 0.5f; // horizontal plane
		float LIMIT_ROLL = 20.0f; // deg
		float LIMIT_PITCH = 20.0f; // deg

		// Thrust
		float THRUST_BASE = 0.0f;
		float THRUST_SCALE = 100.0f;
		float THRUST_MIN = -100.0f;
		float THRUST_MAX = 100.0f;
	};

	struct params_swarm
	{
		// Tune pid controller
		bool flag_tune_controller = false;
		
		// Number of drones
		unsigned int number = 12;
		float height_takeoff = 0.7f;
		// Motion capture system
		char ip_local_mocap[16]{};
		char ip_target_mocap[16]{};
		unsigned int frame_num = 120;
		unsigned int span_filter_v = 10; // For sliding-window filter, velocity
		unsigned int span_filter_a = 10; // For sliding-window filter, acceleration
		bool flag_save_data = true;	   // For saving data

		// External control
		char ip_local_external[16]{};
		char ip_target_external[16]{};
		unsigned short int port_local_start = 21000; // local: port_local_start +1 + 2*i, target_state: port_local_start + 2*i, target_bar: port_local_start - 1000 + 2*i
		//unsigned short int port_target_start_state;
		//unsigned short int port_target_start_bat;
		// Drones
		char ip_local_drone[16]{};
		char **ip_target_drones = nullptr;

		~params_swarm() {
			if (this->ip_target_drones != nullptr)
			{
				delete[] this->ip_target_drones;
			}
		};
	};

	bool get_params_controller_from_txt(params_controller& params,const char*file_name);
	bool get_params_swarm_from_txt(params_swarm& params, const char*file_name);
}