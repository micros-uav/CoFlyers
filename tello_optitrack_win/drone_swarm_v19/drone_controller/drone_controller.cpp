#include"drone_controller.h"
#include"configuration.h"
#include<math.h>
#include<iostream>
namespace drone_controller
{
	drone_controller::drone_controller()
	{

	}
	drone_controller::~drone_controller()
	{
	}
	void drone_controller::set_x_pid_params(float proportion, float integral, float derivative, float c_filter_d, float dt, float output_min, float output_max, float integral_min, float integral_max)
	{
		this->x_pid.set_params(proportion, integral, derivative, c_filter_d, dt, output_min, output_max, integral_min, integral_max);
	}
	void drone_controller::set_y_pid_params(float proportion, float integral, float derivative, float c_filter_d, float dt, float output_min, float output_max, float integral_min, float integral_max)
	{
		this->y_pid.set_params(proportion, integral, derivative, c_filter_d, dt, output_min, output_max, integral_min, integral_max);
	}
	void drone_controller::set_z_pid_params(float proportion, float integral, float derivative, float c_filter_d, float dt, float output_min, float output_max, float integral_min, float integral_max)
	{
		this->z_pid.set_params(proportion, integral, derivative, c_filter_d, dt, output_min, output_max, integral_min, integral_max);
	}
	void drone_controller::set_vx_pid_params(float proportion, float integral, float derivative, float c_filter_d, float dt, float output_min, float output_max, float integral_min, float integral_max)
	{
		this->vx_pid.set_params(proportion, integral, derivative, c_filter_d, dt, output_min, output_max, integral_min, integral_max);
	}
	void drone_controller::set_vy_pid_params(float proportion, float integral, float derivative, float c_filter_d, float dt, float output_min, float output_max, float integral_min, float integral_max)
	{
		this->vy_pid.set_params(proportion, integral, derivative, c_filter_d, dt, output_min, output_max, integral_min, integral_max);
	}
	void drone_controller::set_vz_pid_params(float proportion, float integral, float derivative, float c_filter_d, float dt, float output_min, float output_max, float integral_min, float integral_max)
	{
		this->vz_pid.set_params(proportion, integral, derivative, c_filter_d, dt, output_min, output_max, integral_min, integral_max);
	}
	void drone_controller::set_yaw_pid_params(float proportion, float integral, float derivative, float c_filter_d, float dt, float output_min, float output_max, float integral_min, float integral_max)
	{
		this->yaw_pid.set_params(proportion, integral, derivative, c_filter_d, dt, output_min, output_max, integral_min, integral_max);
	}
	bool drone_controller::set_params_from_txt(const char*file_name)
	{
		/*if (this->file_name_controller[0] == '\0')
		{
			strcpy(this->file_name_controller, file_name);
		}*/
		strcpy(this->file_name_controller, file_name);

		config::params_controller params;
		bool out = config::get_params_controller_from_txt(params, file_name);

		if (!out)
		{
			return out;
		}

		this->dt = params.delta_t;
	
		set_x_pid_params(params.POS_X_PID_KP, params.POS_X_PID_KI, params.POS_X_PID_KD, params.POS_X_PID_CD, params.delta_t,-params.LIMIT_vel_x, params.LIMIT_vel_x, -100000.0, 100000.0);
		set_y_pid_params(params.POS_Y_PID_KP, params.POS_Y_PID_KI, params.POS_Y_PID_KD, params.POS_Y_PID_CD, params.delta_t, -params.LIMIT_vel_y, params.LIMIT_vel_y, -100000.0, 100000.0);
		set_z_pid_params(params.POS_Z_PID_KP, params.POS_Z_PID_KI, params.POS_Z_PID_KD, params.POS_Z_PID_CD, params.delta_t, -params.LIMIT_vel_z, params.LIMIT_vel_z, -100000.0, 100000.0);
		set_vx_pid_params(params.VEL_X_PID_KP, params.VEL_X_PID_KI, params.VEL_X_PID_KD, params.VEL_X_PID_CD, params.delta_t, -params.LIMIT_PITCH, params.LIMIT_PITCH, -100000.0, 100000.0);
		set_vy_pid_params(params.VEL_Y_PID_KP, params.VEL_Y_PID_KI, params.VEL_Y_PID_KD, params.VEL_Y_PID_CD, params.delta_t, -params.LIMIT_ROLL, params.LIMIT_ROLL, -100000.0, 100000.0);
		float thrust_min = (params.THRUST_MIN - params.THRUST_BASE) / params.THRUST_SCALE;
		float thrust_max = (params.THRUST_MAX - params.THRUST_BASE) / params.THRUST_SCALE;
		set_vz_pid_params(params.VEL_Z_PID_KP, params.VEL_Z_PID_KI, params.VEL_Z_PID_KD, params.VEL_Z_PID_CD, params.delta_t, thrust_min, thrust_max, -100000.0, 100000.0);
		set_yaw_pid_params(params.YAW_PID_KP, params.YAW_PID_KI, params.YAW_PID_KD, params.YAW_PID_CD, params.delta_t, -360.0, 360.0, -params.YAW_PID_I_LIMIT, params.YAW_PID_I_LIMIT);
		this->thrust_base = params.THRUST_BASE;
		this->thrust_scale = params.THRUST_SCALE;
		this->limit_v_horizontal = params.LIMIT_vel_h;
		
		this->print_params();
		return out;
	}
	void drone_controller::print_params()
	{
		std::cout << "===============================================================" << std::endl;

		std::cout << "Sample time of control: " << this->dt << std::endl;

		std::cout << "POS X PID: \nKP=" << this->x_pid.proportion <<
			"\tKI=" << this->x_pid.integral << "\tKD=" << this->x_pid.derivative <<
			"\tCD=" << this->x_pid.c_filter_d << "\tvmin=" << this->x_pid.output_min <<
			"\tvmax=" << this->x_pid.output_max << "\timin=" << this->x_pid.integral_min <<
			"\timax=" << this->x_pid.integral_max << std::endl;

		std::cout << "POS Y PID: \nKP=" << this->y_pid.proportion <<
			"\tKI=" << this->y_pid.integral << "\tKD=" << this->y_pid.derivative <<
			"\tCD=" << this->y_pid.c_filter_d << "\tvmin=" << this->y_pid.output_min <<
			"\tvmax=" << this->y_pid.output_max << "\timin=" << this->y_pid.integral_min <<
			"\timax=" << this->y_pid.integral_max << std::endl;

		std::cout << "POS Z PID: \nKP=" << this->z_pid.proportion <<
			"\tKI=" << this->z_pid.integral << "\tKD=" << this->z_pid.derivative <<
			"\tCD=" << this->z_pid.c_filter_d << "\tvmin=" << this->z_pid.output_min <<
			"\tvmax=" << this->z_pid.output_max << "\timin=" << this->z_pid.integral_min <<
			"\timax=" << this->z_pid.integral_max << std::endl;

		std::cout << "VEL X PID: \nKP=" << this->vx_pid.proportion <<
			"\tKI=" << this->vx_pid.integral << "\tKD=" << this->vx_pid.derivative <<
			"\tCD=" << this->vx_pid.c_filter_d << "\tvmin=" << this->vx_pid.output_min <<
			"\tvmax=" << this->vx_pid.output_max << "\timin=" << this->vx_pid.integral_min <<
			"\timax=" << this->vx_pid.integral_max << std::endl;

		std::cout << "VEL Y PID: \nKP=" << this->vy_pid.proportion <<
			"\tKI=" << this->vy_pid.integral << "\tKD=" << this->vy_pid.derivative <<
			"\tCD=" << this->vy_pid.c_filter_d << "\tvmin=" << this->vy_pid.output_min <<
			"\tvmax=" << this->vy_pid.output_max << "\timin=" << this->vy_pid.integral_min <<
			"\timax=" << this->vy_pid.integral_max << std::endl;

		std::cout << "VEL Z PID: \nKP=" << this->vz_pid.proportion <<
			"\tKI=" << this->vz_pid.integral << "\tKD=" << this->vz_pid.derivative <<
			"\tCD=" << this->vz_pid.c_filter_d << "\tvmin=" << this->vz_pid.output_min <<
			"\tvmax=" << this->vz_pid.output_max << "\timin=" << this->vz_pid.integral_min <<
			"\timax=" << this->vz_pid.integral_max << std::endl;

		std::cout << "YAW PID: \nKP=" << this->yaw_pid.proportion <<
			"\tKI=" << this->yaw_pid.integral << "\tKD=" << this->yaw_pid.derivative <<
			"\tCD=" << this->yaw_pid.c_filter_d << "\tvmin=" << this->yaw_pid.output_min <<
			"\tvmax=" << this->yaw_pid.output_max << "\timin=" << this->yaw_pid.integral_min <<
			"\timax=" << this->yaw_pid.integral_max << std::endl;

		std::cout << "thrust base = " << this->thrust_base << "\t" << "thrust scale = " <<
			this->thrust_scale << "\t" << "limit v horizontal = " << this->limit_v_horizontal << std::endl;

		std::cout << "===============================================================" << std::endl;
	}
	void drone_controller::set_mode(int mode)
	{
		switch (mode)
		{
		case 0:
			this->mode_pos_x = true;
			this->mode_pos_y = true;
			this->mode_pos_z = true;
			this->mode_vel_ff_x = false;
			this->mode_vel_ff_y = false;
			this->mode_vel_ff_z = false;
			this->mode_yaw_or_rate = true;
			break;
		case 1:
			this->mode_pos_x = false;
			this->mode_pos_y = false;
			this->mode_pos_z = false;
			this->mode_vel_ff_x = true;
			this->mode_vel_ff_y = true;
			this->mode_vel_ff_z = true;
			this->mode_yaw_or_rate = false;
			break;
		case 2:
			this->mode_pos_x = false;
			this->mode_pos_y = false;
			this->mode_pos_z = true;
			this->mode_vel_ff_x = true;
			this->mode_vel_ff_y = true;
			this->mode_vel_ff_z = false;
			this->mode_yaw_or_rate = true;

		default:
			break;
		}
	}
	void drone_controller::update(data_description::attitude_command &att_cmd, const data_description::state_12& now, const data_description::state_12& desire, const float& yaw_rate_d)
	{
		if (this->flag_tuning_controller)
		{
			this->set_params_from_txt(this->file_name_controller);
		}

		float pi = 3.1415926f;

		float cosyaw = (float)cos(now.angle.yaw / 180.0 * pi);
		float sinyaw = (float)sin(now.angle.yaw / 180.0 * pi);

		// Position control
		float vx_u = 0.0f;
		if (this->mode_pos_x)
		{
			float x_body = now.pos.x * cosyaw + now.pos.y * sinyaw;
			float x_d_body = desire.pos.x * cosyaw + desire.pos.y * sinyaw;
			vx_u = this->x_pid.update(x_d_body - x_body);
		}

		float vy_u = 0.0f;
		if (this->mode_pos_y)
		{
			float y_body = -now.pos.x * sinyaw + now.pos.y * cosyaw;
			float y_d_body = -desire.pos.x * sinyaw + desire.pos.y * cosyaw;
			vy_u = this->y_pid.update(y_d_body - y_body);
		}

		float vz_u = 0.0f;
		if (this->mode_pos_z)
		{
			vz_u = this->z_pid.update(desire.pos.z - now.pos.z);
		}

		// Velocity control
		float vx_body = now.vel.x * cosyaw + now.vel.y * sinyaw;
		float vy_body = -now.vel.x * sinyaw + now.vel.y * cosyaw;

		if(this->mode_vel_ff_x)
		{
			float vx_d_body = desire.vel.x * cosyaw + desire.vel.y * sinyaw;
			vx_u += vx_d_body;
		}
		if (this->mode_vel_ff_y)
		{
			float vy_d_body = -desire.vel.x * sinyaw + desire.vel.y * cosyaw;
			vy_u += vy_d_body;
		}
		if (this->mode_vel_ff_z)
		{
			vz_u += desire.vel.z;
		}

		att_cmd.pitch  =  this->vx_pid.update(vx_u - vx_body);
		att_cmd.roll   = -this->vy_pid.update(vy_u - vy_body);
		att_cmd.thrust =  this->vz_pid.update(vz_u - now.vel.z) * this->thrust_scale + this->thrust_base;

		// Yaw control
		if (this->mode_yaw_or_rate)
		{
			float error_yaw = desire.angle.yaw - now.angle.yaw;
			if (error_yaw > 180.0f)
			{
				error_yaw -= 360.0f;
			}
			else if (error_yaw < -180.0f)
			{
				error_yaw += 360.0f;
			}
			att_cmd.yaw_rate = this->yaw_pid.update(error_yaw);
		}
		else
		{
			att_cmd.yaw_rate = yaw_rate_d;
		}

	}
	void drone_controller::reset()
	{
		this->x_pid.reset();
		this->y_pid.reset();
		this->z_pid.reset();
		this->vx_pid.reset();
		this->vy_pid.reset();
		this->vz_pid.reset();
		this->yaw_pid.reset();
	}
	void drone_controller::set_sample_time(float dt)
	{
		this->dt = dt;
		this->x_pid.dt = this->dt;
		this->y_pid.dt = this->dt;
		this->z_pid.dt = this->dt;
		this->vx_pid.dt = this->dt;
		this->vy_pid.dt = this->dt;
		this->vz_pid.dt = this->dt;
		this->yaw_pid.dt = this->dt;
	}
	float drone_controller::get_sample_time()
	{
		return this->dt;
	}
}
