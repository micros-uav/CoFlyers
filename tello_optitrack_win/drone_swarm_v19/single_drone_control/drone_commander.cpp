#include"drone_commander.h"
#include"operations.h"
#include<mutex>
namespace drone_commander
{
	drone_commander::drone_commander()
	{

		this->drone_udp.packRecvPtr = &(this->packrecv);
		this->drone_udp.packSendPtr = &(this->packsend);
		this->drone_udp.packSendPtr->componentId = msg::COMPONENT_TYPE::STATE_TYPE;
		this->drone_udp.packSendPtr->messageId = msg::MESSAGE_TYPE_STATE_TYPE::BATTERY_STATE_TYPE;
		this->udp_pack_external.packSendPtr = &(this->pack_send_state_bat);
		this->udp_pack_external.packRecvPtr = &(this->pack_recv_command);
	}

	drone_commander::~drone_commander()
	{

	}
	void drone_commander::update_state_simple_by_mocap(const data_description::state_rigidbody * state_ptr, float delta_t)
	{
		this->state.pos.x = state_ptr->pos.x;
		this->state.pos.y = state_ptr->pos.y;
		this->state.pos.z = state_ptr->pos.z;
		this->state.vel.x = (this->state.pos.x - this->state_former.pos.x) / delta_t;
		this->state.vel.y = (this->state.pos.y - this->state_former.pos.y) / delta_t;
		this->state.vel.z = (this->state.pos.z - this->state_former.pos.z) / delta_t;
		operations::quaternion2euler(state_ptr->q, this->state.angle);
		this->state_former = this->state;
	}

	void drone_commander::updata_state_simple_by_state_simple(const data_description::state_12 & input)
	{
		this->state = input;
	}

	bool drone_commander::init_drone_udp(const char*ip_local_drone,const char*ip_target_drone,unsigned short int idd)
	{
		this->ID = idd;
		strcpy(this->ip_target_drone, ip_target_drone);
		unsigned short int port = this->port_local_drone + idd;
		this->drone_udp.setIpPort(ip_local_drone, port, ip_target_drone, this->port_target_drone, tello::LEN_MAX_BUFFER_SEND, tello::LEN_MAX_BUFFER_RECV);
		return this->drone_udp.initSocket();
	}


	bool drone_commander::init_udp_external(const char*ip_local_external, const char*ip_target_external, int port_target_state, int port_target_bat, int port_local_command)
	{
		this->port_local_command = port_local_command;
		this->port_target_state = port_target_state;
		this->port_target_bat = port_target_bat;
		this->udp_pack_external.setIpPort(ip_local_external, this->port_local_command, ip_target_external, this->port_target_state, msg::LEN_MAX_BUFFER, msg::LEN_MAX_BUFFER);
		return this->udp_pack_external.initSocket();
	}
	bool drone_commander::init_controller(const char* file_name)
	{
		return this->controller.set_params_from_txt(file_name);
	}
	void drone_commander::activate_tune_controller() { this->controller.flag_tuning_controller = true; }
	void drone_commander::deactivate_tune_controller() { this->controller.flag_tuning_controller = false; }

	void drone_commander::callback_receice_state_from_tello()
	{
		// Receive state from tello
		while (this->flag_run)
		{
			int len = this->drone_udp.generateStatePackage();
			
			if (len > 0)
			{
				/*if (this->drone_udp.adressRequest.sin_addr.s_addr == inet_addr("192.168.1.34"))
				{
					printf("11111");
				}*/
				this->bat = this->drone_udp.packSendPtr->msgData[0];

				/*printf("Timestamp: %3.2lf\n", drone_udp.packSendPtr->timeStamp);
				printf("mid:%f;x:%f;y:%f;z:%f;mpry:%f,%f,%f;pitch:%f;roll:%f;yaw:%f;vgx:%f;vgy:%f;vgz:%f;templ:%f;temph:%f;tof:%f;h:%f;bat:%f;baro:%f;time:%f;agx:%f;agy:%f;agz:%f;\n",
					drone_udp.state_from_tello[0], drone_udp.state_from_tello[1], drone_udp.state_from_tello[2], drone_udp.state_from_tello[3], drone_udp.state_from_tello[4], drone_udp.state_from_tello[5], drone_udp.state_from_tello[6], drone_udp.state_from_tello[7], drone_udp.state_from_tello[8], drone_udp.state_from_tello[9], drone_udp.state_from_tello[10], drone_udp.state_from_tello[11], drone_udp.state_from_tello[12], drone_udp.state_from_tello[13], drone_udp.state_from_tello[14], drone_udp.state_from_tello[15],
					drone_udp.state_from_tello[16], drone_udp.state_from_tello[17], drone_udp.state_from_tello[18], drone_udp.state_from_tello[19], drone_udp.state_from_tello[20], drone_udp.state_from_tello[21], drone_udp.state_from_tello[22]);
*/
			}
			std::this_thread::sleep_for(std::chrono::milliseconds(200));
		}
		return;
	}
	void drone_commander::callback_process_high_command()
	{
		while (this->flag_run)
		{
			if (this->flag_trigger)
			{
				this->flag_trigger = false;
				this->high_command_mode_former = this->high_command_mode;
				switch (this->high_command_mode)
				{
				case msg::MESSAGE_TYPE_COMMAND_TYPE::ARMING_TYPE:
					process_arming();
					break;
				case msg::MESSAGE_TYPE_COMMAND_TYPE::TAKEOFF_TYPE:
					process_takeoff();
					break;
				case msg::MESSAGE_TYPE_COMMAND_TYPE::LAND_TYPE:
					process_land();
					break;
				case msg::MESSAGE_TYPE_COMMAND_TYPE::HOVER_TYPE:
					process_hover();
					break;
				case msg::MESSAGE_TYPE_COMMAND_TYPE::POSITION_CONTROL_TYPE:
					process_setpoint_position();
					break;
				case msg::MESSAGE_TYPE_COMMAND_TYPE::VELOCITY_CONTROL_TYPE:
					process_velocity();
				case msg::MESSAGE_TYPE_COMMAND_TYPE::VELOCITY_HORIZONTAL_CONTROL_TYPE:
					process_velocity_horizontal();
				case msg::MESSAGE_TYPE_COMMAND_TYPE::ATTITUDE_CONTROL_TYPE:
					process_setpoint_attitude();
					break;
				case msg::MESSAGE_TYPE_COMMAND_TYPE::FULL_STATE_CONTROL_TYPE:
					//process_setpoint_position_velocity();
					break;
				default:
					break;
				}
			}
			std::this_thread::sleep_for(std::chrono::milliseconds(10));
		}
	}
	void drone_commander::callback_udp_external()
	{
		std::chrono::system_clock::time_point ts = std::chrono::system_clock::now();
		std::chrono::system_clock::time_point te = std::chrono::system_clock::now();
		long long delta_T = std::chrono::duration_cast<std::chrono::microseconds>(te - ts).count();


		std::chrono::system_clock::time_point ts_to = std::chrono::system_clock::now();
		std::chrono::system_clock::time_point te_to = std::chrono::system_clock::now();
		long long delta_T_to = std::chrono::duration_cast<std::chrono::microseconds>(te - ts).count();
		while (this->flag_run)
		{
			ts = std::chrono::system_clock::now();
			//Receive command
			int len = udp_pack_external.recvMsg();
			if (len > 0)
			{
				ts_to = std::chrono::system_clock::now();
				if (this->pack_recv_command.componentId == msg::COMPONENT_TYPE::COMMAND_TYPE)
				{
					switch (this->pack_recv_command.messageId)
					{
					case msg::MESSAGE_TYPE_COMMAND_TYPE::ARMING_TYPE:
						if (this->pack_recv_command.messageId != this->command_type_former)
						{
							printf("ID %d arming!\n", this->ID);
							this->arming();
						}
						break;
					case msg::MESSAGE_TYPE_COMMAND_TYPE::TAKEOFF_TYPE:
						if (this->pack_recv_command.messageId != this->command_type_former)
						{
							this->state_save = this->state;
							printf("ID %d takeoff!\n", this->ID);
							this->takeoff();
						}
						break;
					case msg::MESSAGE_TYPE_COMMAND_TYPE::LAND_TYPE:
						if (this->pack_recv_command.messageId != this->command_type_former)
						{
							this->state_save = this->state;
							printf("ID %d land!\n", this->ID);
							this->land_without_ep();
						}
						break;
					case msg::MESSAGE_TYPE_COMMAND_TYPE::HOVER_TYPE:
						if (this->pack_recv_command.messageId != this->command_type_former)
						{
							this->state_save = this->state;
							printf("ID %d hover!\n", this->ID);
							this->hover();
						}
						break;
					case msg::MESSAGE_TYPE_COMMAND_TYPE::POSITION_CONTROL_TYPE:
						if (this->pack_recv_command.messageId != this->command_type_former)
						{
							printf("ID %d position control!\n", this->ID);
						}
						this->setpoint_position(pack_recv_command.msgData[0], pack_recv_command.msgData[1], pack_recv_command.msgData[2], pack_recv_command.msgData[3]);
						break;
					case msg::MESSAGE_TYPE_COMMAND_TYPE::VELOCITY_CONTROL_TYPE:
						if (this->pack_recv_command.messageId != this->command_type_former)
						{
							printf("ID %d velocity control!\n", this->ID);
						}
						this->setpoint_velocity(pack_recv_command.msgData[0], pack_recv_command.msgData[1], pack_recv_command.msgData[2], pack_recv_command.msgData[3]);
						break;
					case msg::MESSAGE_TYPE_COMMAND_TYPE::VELOCITY_HORIZONTAL_CONTROL_TYPE:
						if (this->pack_recv_command.messageId != this->command_type_former)
						{
							printf("ID %d velocity horizontal control!\n", this->ID);
						}
						this->setpoint_velocity_horizontal(pack_recv_command.msgData[0], pack_recv_command.msgData[1], pack_recv_command.msgData[2], pack_recv_command.msgData[3]);
						break;
					case msg::MESSAGE_TYPE_COMMAND_TYPE::ATTITUDE_CONTROL_TYPE:
						if (this->pack_recv_command.messageId != this->command_type_former)
						{
							printf("ID %d attitude control!\n", this->ID);
						}
						this->setpoint_attitude(pack_recv_command.msgData[0], pack_recv_command.msgData[1], pack_recv_command.msgData[2], pack_recv_command.msgData[3]);
						break;
					case msg::MESSAGE_TYPE_COMMAND_TYPE::FULL_STATE_CONTROL_TYPE:
						if (this->pack_recv_command.messageId != this->command_type_former)
						{
							printf("ID %d full state control!\n", this->ID);
						}
						//this->setpoint_position_velocity(pack_recv_command.msgData[0], pack_recv_command.msgData[1], pack_recv_command.msgData[2], pack_recv_command.msgData[3],
						//	pack_recv_command.msgData[4], pack_recv_command.msgData[5], pack_recv_command.msgData[6]);
						break;
					default:
						if (this->pack_recv_command.messageId != this->command_type_former)
						{
							printf("Undefined command type!\n");
							this->hover();
						}
						break;
					}
					this->command_type_former = this->pack_recv_command.messageId;
				}
				//printf("systemID:%d,componentID:%d,messageID:%d,", this->pack_recv_command.systemId, this->pack_recv_command.componentId, this->pack_recv_command.messageId);
				//for (int i = 0; i < this->pack_recv_command.lenPayload/4; i++)
				//{
				//	printf("%f,", this->pack_recv_command.msgData[i]);
				//}
				//printf("\n");
			}

			// Send 16 states
			this->pack_send_state_bat.start = 0xFA;
			this->pack_send_state_bat.numSequence++;
			this->pack_send_state_bat.componentId = msg::COMPONENT_TYPE::STATE_TYPE;
			//this->pack_send_state_bat.messageId = msg::MESSAGE_TYPE_STATE_TYPE::KINEMATIC_STATE_TYPE;
			this->pack_send_state_bat.messageId = msg::MESSAGE_TYPE_STATE_TYPE::STATE_12_TYPE;
			
			int num = msg::getMsgNum(this->pack_send_state_bat.componentId, this->pack_send_state_bat.messageId);
			this->pack_send_state_bat.systemId = this->ID;
			this->pack_send_state_bat.lenPayload = num * 4;
			this->pack_send_state_bat.msgData[0] = this->state.pos.x;
			this->pack_send_state_bat.msgData[1] = this->state.pos.y;
			this->pack_send_state_bat.msgData[2] = this->state.pos.z;
			this->pack_send_state_bat.msgData[3] = this->state.vel.x;
			this->pack_send_state_bat.msgData[4] = this->state.vel.y;
			this->pack_send_state_bat.msgData[5] = this->state.vel.z;
			this->pack_send_state_bat.msgData[6] = this->state.acc.x;
			this->pack_send_state_bat.msgData[7] = this->state.acc.y;
			this->pack_send_state_bat.msgData[8] = this->state.acc.z;
			this->pack_send_state_bat.msgData[9] = this->state.angle.yaw;
			this->pack_send_state_bat.msgData[10] = this->state.angle.roll;
			this->pack_send_state_bat.msgData[11] = this->state.angle.pitch;
			getMillisecond(&this->pack_send_state_bat.timeStamp);
			this->pack_send_state_bat.checkNum = msg::get_check_num(this->pack_send_state_bat);

			udp_pack_external.changePortTarget(this->port_target_state);
			udp_pack_external.sendMsg();

			// Send bat states
			this->pack_send_state_bat.start = 0xFA;
			this->pack_send_state_bat.numSequence++;
			this->pack_send_state_bat.componentId = msg::COMPONENT_TYPE::STATE_TYPE;
			this->pack_send_state_bat.messageId = msg::MESSAGE_TYPE_STATE_TYPE::BATTERY_STATE_TYPE;

			num = msg::getMsgNum(this->pack_send_state_bat.componentId, this->pack_send_state_bat.messageId);
			this->pack_send_state_bat.systemId = this->ID;
			this->pack_send_state_bat.lenPayload = num * 4;
			this->pack_send_state_bat.msgData[0] = this->bat;
			getMillisecond(&this->pack_send_state_bat.timeStamp);
			this->pack_send_state_bat.checkNum = msg::get_check_num(this->pack_send_state_bat);

			udp_pack_external.changePortTarget(this->port_target_bat);
			udp_pack_external.sendMsg();

			// Time out
			/*te_to = std::chrono::system_clock::now();
			delta_T_to = std::chrono::duration_cast<std::chrono::microseconds>(te_to - ts_to).count();
			if (delta_T > this->time_out_external * 1000 * 1000)
			{
				this->land();
			}*/

			// Sleep
			te = std::chrono::system_clock::now();
			delta_T = std::chrono::duration_cast<std::chrono::microseconds>(te - ts).count();

			if (delta_T < this->dt_external * 1000 * 1000)
			{
				std::this_thread::sleep_for(std::chrono::microseconds(int(this->dt_external * 1000 * 1000 - delta_T)));
			}
			else
			{
				printf("warning, real delta t:%lld, external control delta_t:%d\n", delta_T, int(this->dt_external * 1000 * 1000));
			}
		}
	}

	void drone_commander::process_arming()
	{
		/*if (this->drone_udp.packSendPtr->msgData[0] < 1)
		{*/
			this->packrecv.start = 0xFA;
			this->packrecv.componentId = msg::COMPONENT_TYPE::COMMAND_TYPE;
			this->packrecv.messageId = msg::MESSAGE_TYPE_COMMAND_TYPE::ARMING_TYPE;
			this->packrecv.lenPayload = 0;
			// battery: this->drone_udp.packSendPtr->msgData[0]
			//printf("ID %d Arming!\n", this->ID);
			this->drone_udp.processCommand();
			std::this_thread::sleep_for(std::chrono::milliseconds(10));
			while (this->bat < 1 && this->flag_run)
			{
				this->flag_armed = false;
				this->drone_udp.processCommand();
				std::this_thread::sleep_for(std::chrono::milliseconds(10));
			}
			if (this->bat > 1)
			{
				printf("ID %d Armed!\n", this->ID);
				this->flag_armed = true;
			}
		//}
	}
	void drone_commander::process_takeoff()
	{
		// Confirm takeoff
		/*while (true)
		{
		}*/
		this->controller.reset();
		// It requires to send "takeoff" command before controlling tello by attitude control 
		//printf("ID %d take off!\n",this->ID);
		packrecv.start = 0xFA;
		packrecv.componentId = msg::COMPONENT_TYPE::COMMAND_TYPE;
		packrecv.messageId = msg::MESSAGE_TYPE_COMMAND_TYPE::TAKEOFF_TYPE;
		packrecv.lenPayload = 0;
		this->drone_udp.processCommand();
		this->flag_land = false;
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
		// Hover, skip to position control
		this->setpoint_position(this->state_save.pos.x, this->state_save.pos.y, this->height_takeoff, this->state_save.angle.yaw);
	}
	void drone_commander::process_land()
	{
		/*if (!this->flag_land)
		{*/
			this->controller.reset();
			//float z_hovor = this->state.pos.z;
			//while (z_hovor - this->state.pos.z < 0.1)
			//{
				//printf("ID %d Land!\n",this->ID);
				packrecv.start = 0xFA;
				packrecv.componentId = msg::COMPONENT_TYPE::COMMAND_TYPE;
				packrecv.messageId = msg::MESSAGE_TYPE_COMMAND_TYPE::LAND_TYPE;
				packrecv.lenPayload = 0;
				this->drone_udp.processCommand();
				//std::this_thread::sleep_for(std::chrono::seconds(3));
			//}
			this->flag_land = true;
		//}
	}
	void drone_commander::process_hover()
	{
		this->setpoint_position(this->state_save.pos.x, this->state_save.pos.y, this->state_save.pos.z, this->state_save.angle.yaw);
	}
	void drone_commander::process_setpoint()
	{
		float gain_roll = 1000;
		float gain_pitch = 1000;
		float gain_throttle = 100;
		float gain_yaw_rate = -100;
		std::chrono::system_clock::time_point ts = std::chrono::system_clock::now();
		std::chrono::system_clock::time_point te = std::chrono::system_clock::now();
		long long delta_T = std::chrono::duration_cast<std::chrono::microseconds>(te - ts).count();
		while (this->flag_run && (this->high_command_mode == this->high_command_mode_former))
		{
			ts = std::chrono::system_clock::now();

			this->controller.update(this->att_cmd, this->state, this->desire, this->yaw_rate_d);

			packrecv.start = 0xFA;
			packrecv.componentId = msg::COMPONENT_TYPE::COMMAND_TYPE;
			packrecv.messageId = msg::MESSAGE_TYPE_COMMAND_TYPE::ATTITUDE_CONTROL_TYPE;
			packrecv.lenPayload = 4;
			/*packrecv.msgData[0] = this->att_cmd.yaw_rate / 180.0f*3.1415926f * gain_yaw_rate;
			packrecv.msgData[1] = this->att_cmd.roll / 180.0f*3.1415926f * gain_roll;
			packrecv.msgData[2] = this->att_cmd.pitch / 180.0f*3.1415926f * gain_pitch;*/
			packrecv.msgData[0] = -this->att_cmd.yaw_rate;
			packrecv.msgData[1] = this->att_cmd.roll;
			packrecv.msgData[2] = this->att_cmd.pitch;
			packrecv.msgData[3] = this->att_cmd.thrust;
			this->drone_udp.processCommand();

			//printf("command: %f, %f, %f, %f\n", packrecv.msgData[0], packrecv.msgData[1], packrecv.msgData[2], packrecv.msgData[3]);

			te = std::chrono::system_clock::now();
			delta_T = std::chrono::duration_cast<std::chrono::microseconds>(te - ts).count();

			if (delta_T < this->controller.get_sample_time() * 1000 * 1000)
			{
				std::this_thread::sleep_for(std::chrono::microseconds(int(this->controller.get_sample_time() * 1000 * 1000 - delta_T)));
			}
			else
			{
				printf("warning, real delta t:%lld, control delta_t:%d\n", delta_T, int(this->controller.get_sample_time() * 1000 * 1000));
			}
		}
	}
	void drone_commander::process_setpoint_position(){	this->process_setpoint();}
	void drone_commander::process_velocity() { this->process_setpoint(); }
	void drone_commander::process_velocity_horizontal(){	this->process_setpoint();}
	void drone_commander::process_setpoint_attitude()
	{
		float gain_roll = 1000;
		float gain_pitch = 1000;
		float gain_throttle = 100;
		float gain_yaw_rate = -100;
		std::chrono::system_clock::time_point ts = std::chrono::system_clock::now();
		std::chrono::system_clock::time_point te = std::chrono::system_clock::now();
		long long delta_T = std::chrono::duration_cast<std::chrono::microseconds>(te - ts).count();
		while (this->flag_run && (this->high_command_mode == this->high_command_mode_former))
		{
			ts = std::chrono::system_clock::now();

			packrecv.start = 0xFA;
			packrecv.componentId = msg::COMPONENT_TYPE::COMMAND_TYPE;
			packrecv.messageId = msg::MESSAGE_TYPE_COMMAND_TYPE::ATTITUDE_CONTROL_TYPE;
			packrecv.lenPayload = 4;
			//packrecv.msgData[0] = this->att_cmd.yaw_rate / 180.0f*3.1415926f * gain_yaw_rate;
			//packrecv.msgData[1] = this->att_cmd.roll / 180.0f*3.1415926f * gain_roll;
			//packrecv.msgData[2] = this->att_cmd.pitch / 180.0f*3.1415926f * gain_pitch;
			packrecv.msgData[0] = -this->att_cmd.yaw_rate;
			packrecv.msgData[1] = this->att_cmd.roll;
			packrecv.msgData[2] = this->att_cmd.pitch;
			packrecv.msgData[3] = this->att_cmd.thrust;
			this->drone_udp.processCommand();

			//printf("command: %f, %f, %f, %f\n", packrecv.msgData[0], packrecv.msgData[1], packrecv.msgData[2], packrecv.msgData[3]);

			te = std::chrono::system_clock::now();
			delta_T = std::chrono::duration_cast<std::chrono::microseconds>(te - ts).count();

			if (delta_T < this->controller.get_sample_time() * 1000 * 1000)
			{
				std::this_thread::sleep_for(std::chrono::microseconds(int(this->controller.get_sample_time() * 1000 * 1000 - delta_T)));
			}
			else
			{
				printf("warning, real delta t:%lld, control delta_t:%d\n", delta_T, int(this->controller.get_sample_time() * 1000 * 1000));
			}
		}
	}
//	void drone_commander::process_setpoint_position_velocity()
//	{
//		this->process_setpoint();
//	}


	void drone_commander::stop()
	{
		this->flag_run = false;
		if (this->thread_receice_state_from_tello.joinable()){this->thread_receice_state_from_tello.join();}
		if (this->thread_process_high_command.joinable()){this->thread_process_high_command.join();}
		if (this->thread_udp_external.joinable()){this->thread_udp_external.join();}
	}
	void drone_commander::run_receice_state_from_tello()
	{
		this->thread_receice_state_from_tello = std::thread(&drone_commander::drone_commander::callback_receice_state_from_tello, this);
	}
	void drone_commander::run_process_high_command()
	{
		this->thread_process_high_command = std::thread(&drone_commander::drone_commander::callback_process_high_command, this);
	}
	void drone_commander::run_external_control()
	{
		this->thread_udp_external = std::thread(&drone_commander::drone_commander::callback_udp_external, this);
	}

	void drone_commander::arming()
	{
		this->high_command_mode = msg::MESSAGE_TYPE_COMMAND_TYPE::ARMING_TYPE;
		this->flag_trigger = true;
	}
	void drone_commander::takeoff()
	{
		this->high_command_mode = msg::MESSAGE_TYPE_COMMAND_TYPE::TAKEOFF_TYPE;

		this->flag_trigger = true;
	}
	void drone_commander::land_without_ep()
	{
		this->high_command_mode = msg::MESSAGE_TYPE_COMMAND_TYPE::LAND_TYPE;
		this->flag_trigger = true;
	}
	void drone_commander::hover()
	{
		this->high_command_mode = msg::MESSAGE_TYPE_COMMAND_TYPE::HOVER_TYPE;
		this->flag_trigger = true;
	}
	void drone_commander::setpoint_position(float x_d, float y_d, float z_d, float yaw_d)
	{
		this->high_command_mode = msg::MESSAGE_TYPE_COMMAND_TYPE::POSITION_CONTROL_TYPE;
		this->controller.set_mode(0);
		this->desire.pos.x = x_d;
		this->desire.pos.y = y_d;
		this->desire.pos.z = z_d;
		this->desire.angle.yaw = yaw_d;
		/*this->desire.vel.x = 0.0f;
		this->desire.vel.y = 0.0f;
		this->desire.vel.z = 0.0f;*/
		this->flag_trigger = true;
	}
	void drone_commander::setpoint_velocity(float vx_d, float vy_d, float vz_d, float yaw_rate_d)
	{
		this->high_command_mode = msg::MESSAGE_TYPE_COMMAND_TYPE::VELOCITY_CONTROL_TYPE;
		this->controller.set_mode(1);
		this->desire.vel.x = vx_d;
		this->desire.vel.y = vy_d;
		this->desire.vel.z = vz_d;
		this->yaw_rate_d = yaw_rate_d;

		this->flag_trigger = true;
	}
	void drone_commander::setpoint_velocity_horizontal(float vx_d, float vy_d, float z_d, float yaw_rate_d)
	{
		this->high_command_mode = msg::MESSAGE_TYPE_COMMAND_TYPE::VELOCITY_CONTROL_TYPE;
		this->controller.set_mode(2);
		this->desire.vel.x = vx_d;
		this->desire.vel.y = vy_d;
		this->desire.pos.z = z_d;
		this->yaw_rate_d = yaw_rate_d;

		this->flag_trigger = true;
	}
	//void drone_commander::setpoint_position_velocity(float x_d, float y_d, float z_d, float vx_d, float vy_d, float vz_d, float yaw_d)
	//{/////////////////////////////////////////////////////////////////////////////////////////
	//	this->high_command_mode = msg::MESSAGE_TYPE_COMMAND_TYPE::POSITION_VELOCITY_CONTROL_TYPE;
	//	this->controller.set_mode(2);
	//	this->desire.pos.x = x_d;
	//	this->desire.pos.y = y_d;
	//	this->desire.pos.z = z_d;
	//	this->desire.angle.yaw = yaw_d;
	//	this->desire.vel.x = vx_d;
	//	this->desire.vel.y = vy_d;
	//	this->desire.pos.z = z_d;

	//	this->flag_trigger = true;
	//}
	void drone_commander::setpoint_attitude(float yaw_rate_d, float roll_d, float pitch_d, float thrust_d)
	{
		this->high_command_mode = msg::MESSAGE_TYPE_COMMAND_TYPE::ATTITUDE_CONTROL_TYPE;
		this->att_cmd.yaw_rate = yaw_rate_d;
		this->att_cmd.roll = roll_d;
		this->att_cmd.pitch = pitch_d;
		this->att_cmd.thrust = thrust_d;

		this->flag_trigger = true;
	}

	bool drone_commander::is_armed()
	{
		return this->flag_armed;
	}

	void drone_commander::set_flag_armed(bool flag)
	{
		this->flag_armed = flag;
	}
}