#pragma once
#include"pid_controller.h"
#include"data_description.h"

namespace drone_controller
{
	class drone_controller
	{
	public:
		drone_controller();
		~drone_controller();

		void set_x_pid_params(float proportion, float integral, float derivative, float c_filter_d, float dt, float output_min, float output_max, float integral_min, float integral_max);
		void set_y_pid_params(float proportion, float integral, float derivative, float c_filter_d, float dt, float output_min, float output_max, float integral_min, float integral_max);
		void set_z_pid_params(float proportion, float integral, float derivative, float c_filter_d, float dt, float output_min, float output_max, float integral_min, float integral_max);
		void set_vx_pid_params(float proportion, float integral, float derivative, float c_filter_d, float dt, float output_min, float output_max, float integral_min, float integral_max);
		void set_vy_pid_params(float proportion, float integral, float derivative, float c_filter_d, float dt, float output_min, float output_max, float integral_min, float integral_max);
		void set_vz_pid_params(float proportion, float integral, float derivative, float c_filter_d, float dt, float output_min, float output_max, float integral_min, float integral_max);
		void set_yaw_pid_params(float proportion, float integral, float derivative, float c_filter_d, float dt, float output_min, float output_max, float integral_min, float integral_max);

		bool set_params_from_txt(const char* file_name);
		void print_params();
		void set_mode(int mode);
		void update(data_description::attitude_command& att_cmd, const data_description::state_12 &now, const data_description::state_12 &desire, const float &yaw_rate_d);
		void reset();

		void  set_sample_time(float dt);
		float get_sample_time();

		bool flag_tuning_controller = false;
	private:
		pid::pid_controller x_pid;
		pid::pid_controller y_pid;
		pid::pid_controller z_pid;
		pid::pid_controller vx_pid;
		pid::pid_controller vy_pid;
		pid::pid_controller vz_pid;
		pid::pid_controller yaw_pid;

		float dt = 0.01f;

		float thrust_base = 0.0f;
		float thrust_scale = 100.0f;

		float limit_v_horizontal = 0.5f;

		char file_name_controller[1024]{};

		bool mode_pos_x = true;
		bool mode_pos_y = true;
		bool mode_pos_z = true;
		bool mode_vel_ff_x = false;
		bool mode_vel_ff_y = false;
		bool mode_vel_ff_z = false;
		bool mode_yaw_or_rate = true;
	};
}