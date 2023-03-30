#include"UDPInterfaceTello.h"

namespace tello
{
	UDPInterfaceTello::UDPInterfaceTello()
	{
		getMillisecond(&(this->requst_time_last));
		getMillisecond(&(this->requst_time_last_now));
	}

	UDPInterfaceTello::~UDPInterfaceTello()
	{
	}
	void UDPInterfaceTello::processCommand()
	{
		switch (this->packRecvPtr->componentId)
		{
			//case msg::COMPONENT_TYPE::STATE_TYPE:
			//	break;
		case msg::COMPONENT_TYPE::COMMAND_TYPE:
			switch (this->packRecvPtr->messageId)
			{
			case msg::MESSAGE_TYPE_COMMAND_TYPE::SET_MODE_TYPE:
				if (this->packRecvPtr->messageId!=this->command_messageID_pre)
				{
					this->processCommandHover();
					this->sendData(this->buffSend, this->length_send);
				}
				getMillisecond(&(this->requst_time_last_now));
				if ((this->packRecvPtr->messageId != this->command_messageID_pre)
					| (this->requst_time_last_now - this->requst_time_last > 2000))
				{
					this->processCommandSetMode();
					this->requst_time_last = this->requst_time_last_now;
				}
				break;

			case msg::MESSAGE_TYPE_COMMAND_TYPE::ARMING_TYPE:
				if (this->packRecvPtr->messageId != this->command_messageID_pre)
				{
					this->processCommandHover();
					this->sendData(this->buffSend, this->length_send);
				}
				getMillisecond(&(this->requst_time_last_now));
				if ((this->packRecvPtr->messageId != this->command_messageID_pre)
					| (this->requst_time_last_now - this->requst_time_last > 2000))
				{
					this->processCommandArming();
					this->requst_time_last = this->requst_time_last_now;
				}
				break;

			case msg::MESSAGE_TYPE_COMMAND_TYPE::TAKEOFF_TYPE:
				if (this->packRecvPtr->messageId != this->command_messageID_pre)
				{
					this->processCommandHover();
					this->sendData(this->buffSend, this->length_send);
				}

				getMillisecond(&(this->requst_time_last_now));
				//if ((this->packRecvPtr->messageId != this->command_messageID_pre)
				//	| (this->requst_time_last_now - this->requst_time_last > 2000))
				if (this->requst_time_last_now - this->requst_time_last > 100)
				{
					this->processCommandTakeoff();
					this->requst_time_last = this->requst_time_last_now;
				}
				break;

			case msg::MESSAGE_TYPE_COMMAND_TYPE::HOVER_TYPE:
				this->processCommandHover();
				break;

			case msg::MESSAGE_TYPE_COMMAND_TYPE::LAND_TYPE:
				if (this->packRecvPtr->messageId != this->command_messageID_pre)
				{
					this->processCommandHover();
					this->sendData(this->buffSend, this->length_send);
				}

				getMillisecond(&(this->requst_time_last_now));
				//if ((this->packRecvPtr->messageId != this->command_messageID_pre)
				//	| (this->requst_time_last_now - this->requst_time_last > 2000))
				//if (this->packRecvPtr->messageId != this->command_messageID_pre)
				if (this->requst_time_last_now - this->requst_time_last > 100)
				{
					/*this->processCommandHover();
					this->sendData(this->buffSend, this->length_send);*/

					this->processCommandLand();
					this->requst_time_last = this->requst_time_last_now;
				}
				break;

			case msg::MESSAGE_TYPE_COMMAND_TYPE::POSITION_CONTROL_TYPE:
				this->processCommandPosition();
				break;

			case msg::MESSAGE_TYPE_COMMAND_TYPE::VELOCITY_CONTROL_TYPE:
				this->processCommandVelocity();
				break;
			case msg::MESSAGE_TYPE_COMMAND_TYPE::ACCELERATION_CONTROL_TYPE:
				this->processCommandAcceleration();
				break;

			case msg::MESSAGE_TYPE_COMMAND_TYPE::ATTITUDE_CONTROL_TYPE:
				this->processCommandAttitude();
				break;
			default:
				this->processCommandDefault();
				break;
			}
		default:
			break;
		}
		
		this->sendData(this->buffSend, this->length_send);
		this->command_messageID_pre = this->packRecvPtr->messageId;
	}
	void UDPInterfaceTello::processCommandArming()
	{
		this->length_send = 8;
		memcpy(this->buffSend, "command", 8);
	}
	void UDPInterfaceTello::processCommandTakeoff()
	{
		this->length_send = 8;
		memcpy(this->buffSend, "takeoff", this->length_send);
	}
	void UDPInterfaceTello::processCommandHover()
	{
		this->length_send = 10;
		memcpy(this->buffSend, "rc 0 0 0 0", this->length_send);
	}
	void UDPInterfaceTello::processCommandLand()
	{
		this->length_send = 4;
		memcpy(this->buffSend, "land", this->length_send);
	}
	void UDPInterfaceTello::processCommandAttitude()
	{
		this->length_send = tello::LEN_MAX_BUFFER_SEND;
		memset(this->buffSend, 0, tello::LEN_MAX_BUFFER_SEND);
		//sprintf(this->buffSend, "rc %f %f %f %f", this->packRecvPtr->msgData[1]*this->gain_roll,
		//	this->packRecvPtr->msgData[2] * this->gain_pitch, this->packRecvPtr->msgData[3] * this->gain_throttle,
		//	this->packRecvPtr->msgData[0] * this->gain_yaw_rate);
		/*sprintf(this->buffSend, "rc %f %f %f %f", this->packRecvPtr->msgData[1]*this->gain_roll,
			this->packRecvPtr->msgData[2] * this->gain_pitch, this->packRecvPtr->msgData[3],
			this->packRecvPtr->msgData[0] * this->gain_yaw_rate);*/
		for (int i = 0; i < 4; i++)
		{
			if (this->packRecvPtr->msgData[i] < -100)
			{
				this->packRecvPtr->msgData[i] = -100.0;
			}
			else if (this->packRecvPtr->msgData[i] > 100)
			{
				this->packRecvPtr->msgData[i] = 100.0;
			}
		}
		sprintf(this->buffSend, "rc %f %f %f %f", this->packRecvPtr->msgData[1],
			this->packRecvPtr->msgData[2], this->packRecvPtr->msgData[3],
			this->packRecvPtr->msgData[0]);

	}
	void UDPInterfaceTello::processCommandDefault()
	{
		this->length_send = 10;
		memcpy(this->buffSend, "rc 0 0 0 0", this->length_send);
	}

	void UDPInterfaceTello::generateBetteryPackage()
	{
		sscanf_s(this->buffRecv, "mid:%f;x:%f;y:%f;z:%f;mpry:%f,%f,%f;pitch:%f;roll:%f;yaw:%f;vgx:%f;vgy:%f;vgz:%f;templ:%f;temph:%f;tof:%f;h:%f;bat:%f;baro:%f;time:%f;agx:%f;agy:%f;agz:%f;",
			&state_from_tello[0], &state_from_tello[1], &state_from_tello[2], &state_from_tello[3], &state_from_tello[4], &state_from_tello[5], &state_from_tello[6], &state_from_tello[7], &state_from_tello[8], &state_from_tello[9], &state_from_tello[10], &state_from_tello[11], &state_from_tello[12], &state_from_tello[13], &state_from_tello[14], &state_from_tello[15],
			&state_from_tello[16], &state_from_tello[17], &state_from_tello[18], &state_from_tello[19], &state_from_tello[20], &state_from_tello[21], &state_from_tello[22]);
		this->packSendPtr->msgData[0] = state_from_tello[17];
		this->tof = state_from_tello[15];
		getMillisecond(&this->packSendPtr->timeStamp);
	}
}