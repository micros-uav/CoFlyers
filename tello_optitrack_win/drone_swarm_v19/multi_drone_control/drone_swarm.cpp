#include"drone_swarm.h"
#include"NatNetClass_my.h"
#include"operations.h"
#include<direct.h>
namespace drone_swarm
{
	drone_swarm::drone_swarm()
	{
	}

	drone_swarm::~drone_swarm()
	{
		
		if (this->drones != nullptr)
		{
			delete[] this->drones;
		}
	}

	void drone_swarm::init_params()
	{
		//config::get_params_swarm_from_txt(this->params, "D:\\StudyMaster\\myCppCode\\vs2017Projects\\drone_swarm\\drone_swarm\\x64\\Debug\\config\\configuration_swarm.txt");
		if (!config::get_params_swarm_from_txt(this->params, "config\\configuration_swarm.txt"))
		{
			exit(1);
			return;
		}
		;
		if (this->drones == nullptr)
		{
			this->drones = new drone_commander::drone_commander[this->params.number];
		}
		for (unsigned int i = 0; i < this->params.number; i++)
		{
			bool flag = this->drones[i].init_drone_udp(this->params.ip_local_drone,this->params.ip_target_drones[i],i);
			if (!flag)
			{
				printf("ID %d init failed!\n", i);
			}
			this->drones[i].init_udp_external(this->params.ip_local_external, this->params.ip_target_external,
				this->params.port_local_start + 2 * i, this->params.port_local_start + 2 * i - 1000, this->params.port_local_start + 2 * i + 1);
			//this->drones[i].init_controller("D:\\StudyMaster\\myCppCode\\vs2017Projects\\drone_swarm\\drone_swarm\\x64\\Debug\\config\\params_controller\\params_controller.txt");
			this->drones[i].height_takeoff = this->params.height_takeoff;
			if (!this->drones[i].init_controller("config\\params_controller\\params_controller.txt"))
			{
				exit(1);
				return;
			}
		}
		if (this->params.flag_tune_controller)
		{
			for (unsigned int i = 0; i < this->params.number; i++)
			{
				this->drones[i].activate_tune_controller();
			}
		}
	}

	void drone_swarm::run_process_high_command_s()
	{
		if (this->drones !=nullptr)
		{
			for (unsigned int i = 0; i < this->params.number; i++)
			{
				this->drones[i].run_process_high_command();
			}
		}
	}

	void drone_swarm::run_external_control_s()
	{
		if (this->drones != nullptr)
		{
			for (unsigned int i = 0; i < this->params.number; i++)
			{
				this->drones[i].run_external_control();
			}
		}
	}
	void drone_swarm::run_receive_state_from_tellos()
	{
		this->thread_receive_state_from_tellos = std::thread(&drone_swarm::drone_swarm::callback_receive_state_from_tellos,this);
	}
	void drone_swarm::stop()
	{
		this->flag_run = false;
		if (this->thread_receive_state_from_tellos.joinable())
		{
			this->thread_receive_state_from_tellos.join();
		}
		if (this->drones != nullptr)
		{
			for (unsigned int i = 0; i < this->params.number; i++)
			{
				this->drones[i].stop();
			}
		}
	}

	void drone_swarm::callback_receive_state_from_tellos()
	{
		long long time_out_bat = 10 * 1000; //milliseconds
		std::chrono::system_clock::time_point *ts = new std::chrono::system_clock::time_point[this->params.number];
		std::chrono::system_clock::time_point *te = new std::chrono::system_clock::time_point[this->params.number];
		long long *delta_T = new long long[this->params.number];

		for (unsigned int i = 0; i < this->params.number; i++)
		{
			ts[i] = std::chrono::system_clock::now();
			te[i] = std::chrono::system_clock::now();
			delta_T[i] = std::chrono::duration_cast<std::chrono::milliseconds>(te[i] - ts[i]).count();
		}

		while (this->flag_run)
		{
			if (this->drones != nullptr)
			{
				int len = this->drones[0].drone_udp.generateStatePackage();

				if (len > 0)
				{
					for (unsigned int i = 0; i < this->params.number; i++)
					{
						if (this->drones[0].drone_udp.adressRequest.sin_addr.s_addr == inet_addr(this->params.ip_target_drones[i]))
						{
							this->drones[i].bat = this->drones[0].drone_udp.packSendPtr->msgData[0];
							//printf("ID:%d, bat:%f\n", i, this->drones[i].bat);
							ts[i] = std::chrono::system_clock::now();
						}
					}
				}
				for (unsigned int i = 0; i < this->params.number; i++)
				{
					te[i] = std::chrono::system_clock::now();
					delta_T[i] = std::chrono::duration_cast<std::chrono::milliseconds>(te[i] - ts[i]).count();
					if (delta_T[i] > time_out_bat && this->drones[i].bat > 0)
					{
						printf("ID %d timed out! (Battery)\n",this->drones[i].ID);
						this->drones[i].bat = 0;
						this->drones[i].set_flag_armed(false);
					}
				}
				std::this_thread::sleep_for(std::chrono::milliseconds(10));
			}
		}

		if (ts != nullptr) { delete[] ts; ts = nullptr; }
		if (te != nullptr) { delete[] te; te = nullptr; }
		if (ts != nullptr) { delete[] delta_T; delta_T = nullptr; }

	}
}


namespace drone_swarm
{
	drone_swarm* swarm_ptr = nullptr;
	
	// For discovering time-out event
	long long time_out_state = 1 * 1000; //milliseconds
	std::chrono::system_clock::time_point *ts_state = nullptr;
	std::chrono::system_clock::time_point *te_state = nullptr;
	long long *delta_T_state = nullptr;
	bool *flag_captured = nullptr;

	// For filter
	data_description::position **vel_spin_ptr = nullptr, ** acc_spin_ptr = nullptr;
	data_description::position *pos_pre = nullptr, *vel_pre = nullptr;
	std::chrono::system_clock::time_point time_stamp_pre = std::chrono::system_clock::now();
	unsigned int* ind_ptr_v = nullptr, * ind_ptr_a = nullptr;
	unsigned int span_v = 10, span_a = 9;


	// For saving data
	FILE * fp = nullptr;
	char file_name[64]{};
	long long time_out_file = 1 * 1000; //millseconds. Write the buffer data to the file every 1s.
	std::chrono::system_clock::time_point ts_file = std::chrono::system_clock::now();
	std::chrono::system_clock::time_point te_file = std::chrono::system_clock::now();
	long long delta_T_file = std::chrono::duration_cast<std::chrono::milliseconds>(te_file - ts_file).count();


	// For initializing mocap
	void run_receive_optitrack_data(drone_swarm & swarm)
	{
		if (swarm_ptr == nullptr)
		{
			swarm_ptr = &swarm;
		}

		// For discovering time-out event
		ts_state = new std::chrono::system_clock::time_point[swarm_ptr->params.number];
		te_state = new std::chrono::system_clock::time_point[swarm_ptr->params.number];
		delta_T_state = new long long[swarm_ptr->params.number];
		flag_captured = new bool[swarm_ptr->params.number];

		// For sliding-window filter
		span_v = swarm_ptr->params.span_filter_v;
		vel_spin_ptr = new data_description::position*[swarm_ptr->params.number];
		for (unsigned int i = 0; i < swarm_ptr->params.number; i++)
		{
			vel_spin_ptr[i] = new data_description::position[span_v+1];
		}
		ind_ptr_v = new unsigned int[swarm_ptr->params.number];

		span_a = swarm_ptr->params.span_filter_a;
		acc_spin_ptr = new data_description::position * [swarm_ptr->params.number];
		for (unsigned int i = 0; i < swarm_ptr->params.number; i++)
		{
			acc_spin_ptr[i] = new data_description::position[span_a + 1];
		}
		ind_ptr_a = new unsigned int[swarm_ptr->params.number];

		pos_pre = new data_description::position[swarm_ptr->params.number];
		vel_pre = new data_description::position[swarm_ptr->params.number];

		//
		for (unsigned int i = 0; i < swarm_ptr->params.number; i++)
		{
			ts_state[i] = std::chrono::system_clock::now();
			te_state[i] = std::chrono::system_clock::now();
			ts_state[i] -= std::chrono::milliseconds(2 * time_out_state);

			delta_T_state[i] = std::chrono::duration_cast<std::chrono::milliseconds>(te_state[i] - ts_state[i]).count();
			flag_captured[i] = false;

			ind_ptr_v[i] = 0;
			ind_ptr_a[i] = 0;
		}
		
		// For saving data
		const char * dir_name = "data_save";
		if (_mkdir(dir_name))
		{
			printf("Create the folder %s failed. The folder may already exist.\n\n",dir_name);
		}
		else
		{
			printf("Create the folder %s succeeded.\n\n", dir_name);
		}
		if (swarm_ptr->params.flag_save_data)
		{
			const time_t tt = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
			tm local_tm = *localtime(&tt);
			sprintf(file_name, "%s\\data_%d_%d_%d_%d_%d_%d.txt", dir_name, local_tm.tm_year + 1900,
				local_tm.tm_mon + 1,
				local_tm.tm_mday,
				local_tm.tm_hour,
				local_tm.tm_min,
				local_tm.tm_sec);
			printf("The data got by mocap is saved into %s.\n", file_name);
			fp = fopen(file_name, "a");
			fprintf(fp, "\tt\tID\tx\ty\tz\tvx\tvy\tvz\tyaw\troll\tpitch\t\n");
		}
		
		//
		natnet::set_process_rigid_data_handler(process_rigid_data_handler);
		natnet::set_timestamp_handler(get_timestamp_handler);
		natnet::activate(swarm_ptr->params.ip_target_mocap, swarm_ptr->params.ip_local_mocap);
	}
	// Stop mocap and clear up.
	void mocap_stop()
	{
		natnet::stop();

		if (ts_state != nullptr) { delete[] ts_state;	ts_state = nullptr; }
		if (te_state != nullptr) { delete[] te_state; te_state = nullptr; }
		if (ts_state != nullptr) { delete[] delta_T_state; delta_T_state = nullptr; }
		if (flag_captured != nullptr) { delete[] flag_captured; flag_captured = nullptr; }
		if (ind_ptr_v != nullptr) { delete[] ind_ptr_v; ind_ptr_v = nullptr; }
		if (ind_ptr_a != nullptr) { delete[] ind_ptr_a; ind_ptr_a = nullptr; }
		if (pos_pre != nullptr) { delete[] pos_pre; pos_pre = nullptr; }
		if (vel_pre != nullptr) { delete[] vel_pre; vel_pre = nullptr; }

		if (vel_spin_ptr != nullptr)
		{
			for (unsigned int i = 0; i < swarm_ptr->params.number; i++)
			{
				delete[] vel_spin_ptr[i];
			}
			delete[] vel_spin_ptr;
			vel_spin_ptr = nullptr;
		}
		if (acc_spin_ptr != nullptr)
		{
			for (unsigned int i = 0; i < swarm_ptr->params.number; i++)
			{
				delete[] acc_spin_ptr[i];
			}
			delete[] acc_spin_ptr;
			acc_spin_ptr = nullptr;
		}
		if (fp != nullptr)
		{
			fclose(fp);
		}
	}

	// For getting timestamp
	void get_timestamp_handler(const double timestamp)
	{
		if (swarm_ptr != nullptr)
		{
			for (unsigned int i = 0; i < swarm_ptr->params.number; i++)
			{
				swarm_ptr->drones[i].timestamp_state = timestamp;
			}

			// For discovering time-out event
			for (unsigned int i = 0; i < swarm_ptr->params.number; i++)
			{
				te_state[i] = std::chrono::system_clock::now();
				delta_T_state[i] = std::chrono::duration_cast<std::chrono::milliseconds>(te_state[i] - ts_state[i]).count();
				if (delta_T_state[i] > time_out_state)
				{
					if (flag_captured[i])
					{
						flag_captured[i] = false;
						printf("ID %d has been lost in Mocap (time out: %lld ms)!\n", swarm_ptr->drones[i].ID, time_out_state);
						
						swarm_ptr->drones[i].land_without_ep();
						swarm_ptr->drones[i].state.pos.x = 0.0f;
						swarm_ptr->drones[i].state.pos.y = 0.0f;
						swarm_ptr->drones[i].state.pos.z = 0.0f;
						swarm_ptr->drones[i].state.vel.x = 0.0f;
						swarm_ptr->drones[i].state.vel.y = 0.0f;
						swarm_ptr->drones[i].state.vel.z = 0.0f;
						swarm_ptr->drones[i].state.angle.yaw = 0.0f;
						swarm_ptr->drones[i].state.angle.roll = 0.0f;
						swarm_ptr->drones[i].state.angle.pitch = 0.0f;
					}
				}
				else
				{
					if (!flag_captured[i])
					{
						flag_captured[i] = true;
						printf("ID %d has captured by Mocap!\n", swarm_ptr->drones[i].ID);
					}
				}
			}

			// For writing the buffer data to the file
			if (fp != nullptr)
			{
				te_file = std::chrono::system_clock::now();
				delta_T_file = std::chrono::duration_cast<std::chrono::milliseconds>(te_file - ts_file).count();
				if (delta_T_file > time_out_file)
				{
					ts_file = std::chrono::system_clock::now();
					fclose(fp);
					fp = fopen(file_name,"a");
				}
			}
		}
	}

	// For getting states
	void process_rigid_data_handler(const unsigned int id, const data_description::state_rigidbody* state_ptr)
	{
		if (swarm_ptr != nullptr)
		{
			if (id < swarm_ptr->params.number)
			{
				ts_state[id] = std::chrono::system_clock::now();

				//swarm_ptr->drones[id].update_state_simple_by_mocap(state_ptr,1.0f/(float)swarm_ptr->params.frame_num);

				//printf("ID ID:%d, x%f, y%f, z%f, vx %f, vy %f, vz %f yaw%f, roll%f, pitch%f\n",
				//	id, swarm_ptr->drones[id].state.pos.x, swarm_ptr->drones[id].state.pos.y, swarm_ptr->drones[id].state.pos.z,
				//	swarm_ptr->drones[id].state.vel.x, swarm_ptr->drones[id].state.vel.y, swarm_ptr->drones[id].state.vel.z,
				//	swarm_ptr->drones[id].state.angle.yaw, swarm_ptr->drones[id].state.angle.roll, swarm_ptr->drones[id].state.angle.pitch);
				
				//flag_captured[id] = true;
				double dt = ((double)std::chrono::duration_cast<std::chrono::milliseconds>(ts_state[id] - time_stamp_pre).count())/1000.0;
				printf("dt: %lf.\n",dt);

				time_stamp_pre = ts_state[id];
				unsigned int ind_v = ind_ptr_v[id];
				unsigned int ind_a = ind_ptr_a[id];
				
				data_description::state_12 state_origin_now;
				data_description::position* p_ptr_pre = &(pos_pre[id]);
				data_description::position* v_ptr_pre = &(vel_pre[id]);
				// Get now pose from motion capture
				state_origin_now.pos = state_ptr->pos;
				operations::quaternion2euler(state_ptr->q, state_origin_now.angle);
				// Get origin velocity
				state_origin_now.vel.x = (state_origin_now.pos.x - p_ptr_pre->x) * (float)(swarm_ptr->params.frame_num);
				state_origin_now.vel.y = (state_origin_now.pos.y - p_ptr_pre->y) * (float)(swarm_ptr->params.frame_num);
				state_origin_now.vel.z = (state_origin_now.pos.z - p_ptr_pre->z) * (float)(swarm_ptr->params.frame_num);


				/*for (unsigned int i = 0; i < span+1; i++)
				{
					printf("%f,%f,%f,%f,%f,%f,%f,%f,%f\n", state_save_ptr[id][i].pos.x, state_save_ptr[id][i].pos.y, state_save_ptr[id][i].pos.z,
						state_save_ptr[id][i].vel.x, state_save_ptr[id][i].vel.y, state_save_ptr[id][i].vel.z,
						state_save_ptr[id][i].angle.yaw, state_save_ptr[id][i].angle.roll, state_save_ptr[id][i].angle.pitch);
				}*/
				unsigned int ind_span_first_v = (ind_v + 1) % span_v; // indice of the oldest origin position
				unsigned int ind_span_first_a = (ind_a + 1) % span_a; // indice of the oldest origin velocity
				data_description::position* v_ptr_oldest = &(vel_spin_ptr[id][ind_span_first_v]); //Oldest origin velocity
				data_description::position* a_ptr_oldest = &(acc_spin_ptr[id][ind_span_first_a]); //Oldest origin acceleration
				data_description::state_12* s_filter_ptr = &(swarm_ptr->drones[id].state);
				s_filter_ptr->pos.x = state_origin_now.pos.x;
				s_filter_ptr->pos.y = state_origin_now.pos.y;
				s_filter_ptr->pos.z = state_origin_now.pos.z;
				s_filter_ptr->angle.yaw = state_origin_now.angle.yaw;
				s_filter_ptr->angle.roll = state_origin_now.angle.roll;
				s_filter_ptr->angle.pitch = state_origin_now.angle.pitch;
				// Smooth Velocity
				s_filter_ptr->vel.x += (state_origin_now.vel.x - v_ptr_oldest->x) / (float)span_v;
				s_filter_ptr->vel.y += (state_origin_now.vel.y - v_ptr_oldest->y) / (float)span_v;
				s_filter_ptr->vel.z += (state_origin_now.vel.z - v_ptr_oldest->z) / (float)span_v;

				// Get origin acceleration
				state_origin_now.acc.x = (s_filter_ptr->vel.x - v_ptr_pre->x) * (float)(swarm_ptr->params.frame_num);
				state_origin_now.acc.y = (s_filter_ptr->vel.y - v_ptr_pre->y) * (float)(swarm_ptr->params.frame_num);
				state_origin_now.acc.z = (s_filter_ptr->vel.z - v_ptr_pre->z) * (float)(swarm_ptr->params.frame_num);

				//Smooth acceleration
				s_filter_ptr->acc.x += (state_origin_now.acc.x - a_ptr_oldest->x) / (float)span_a;
				s_filter_ptr->acc.y += (state_origin_now.acc.y - a_ptr_oldest->y) / (float)span_a;
				s_filter_ptr->acc.z += (state_origin_now.acc.z - a_ptr_oldest->z) / (float)span_a;

				printf("ID:%d, x%f, y%f, z%f, vx %f, vy %f, vz %f, ax %f, ay %f, az %f, yaw%f, roll%f, pitch%f\n",
					id,swarm_ptr->drones[id].state.pos.x, swarm_ptr->drones[id].state.pos.y, swarm_ptr->drones[id].state.pos.z,
					swarm_ptr->drones[id].state.vel.x, swarm_ptr->drones[id].state.vel.y, swarm_ptr->drones[id].state.vel.z,
					swarm_ptr->drones[id].state.acc.x, swarm_ptr->drones[id].state.acc.y, swarm_ptr->drones[id].state.acc.z,
					swarm_ptr->drones[id].state.angle.yaw, swarm_ptr->drones[id].state.angle.roll, swarm_ptr->drones[id].state.angle.pitch);
				
				//
				ind_ptr_v[id] = ind_span_first_v;
				ind_ptr_a[id] = ind_span_first_a;
				*p_ptr_pre = state_origin_now.pos;
				*v_ptr_pre = state_origin_now.vel;
				*v_ptr_oldest = state_origin_now.vel;
				*a_ptr_oldest = state_origin_now.acc;
				// Data save
				//fprintf(fp, "t\tID\tx\ty\tz\tvx\tvy\tvz\tyaw\troll\tpitch\t\n");
				if (fp != nullptr)
				{
					fprintf(fp, "%lf\t%d\t%3.4f\t%3.4f\t%3.4f\t%3.4f\t%3.4f\t%3.4f\t%3.4f\t%3.4f\t%3.4f\t\n",
						swarm_ptr->drones[id].timestamp_state,
						id,
						s_filter_ptr->pos.x,
						s_filter_ptr->pos.y,
						s_filter_ptr->pos.z,
						s_filter_ptr->vel.x,
						s_filter_ptr->vel.y,
						s_filter_ptr->vel.z,
						s_filter_ptr->angle.yaw,
						s_filter_ptr->angle.roll,
						s_filter_ptr->angle.pitch);
				}
			}
		}
	}


	
}