#include"UDPInterfaceRobot.h"
namespace robot {

	UDPInterfaceRobot::UDPInterfaceRobot()
	{
		this->packRecvPtr = nullptr;
		this->packSendPtr = nullptr;
	}
	UDPInterfaceRobot::~UDPInterfaceRobot()
	{
		this->packRecvPtr = nullptr;
		this->packSendPtr = nullptr;
	}

	/*void UDPInterfaceRobot::connectToUDPInterfaceExternal(udpPack::UDPInterfacePack & udp_sim)
	{
		this->packRecvPtr = &udp_sim.packRecv;
		this->packSendPtr = &udp_sim.packSend;
	}*/
	void UDPInterfaceRobot::connectToUDPInterfaceExternal(msg::package& packRecv, msg::package& packSend)
	{
		this->packRecvPtr = &packRecv;
		this->packSendPtr = &packSend;
	}
	void UDPInterfaceRobot::processCommand()
	{
		if (this->packRecvPtr == nullptr)
		{
			printf("Warning: not package of receiving!");
			return;
		}
		switch (this->packRecvPtr->componentId)
		{
		//case msg::COMPONENT_TYPE::STATE_TYPE:
		//	break;
		case msg::COMPONENT_TYPE::COMMAND_TYPE:
			switch (this->packRecvPtr->messageId)
			{
			case msg::MESSAGE_TYPE_COMMAND_TYPE::SET_MODE_TYPE:
				this->processCommandSetMode();
				break;

			case msg::MESSAGE_TYPE_COMMAND_TYPE::ARMING_TYPE:
				this->processCommandArming();
				break;

			case msg::MESSAGE_TYPE_COMMAND_TYPE::TAKEOFF_TYPE:
				this->processCommandTakeoff();
				break;

			case msg::MESSAGE_TYPE_COMMAND_TYPE::HOVER_TYPE:
				this->processCommandHover();
				break;

			case msg::MESSAGE_TYPE_COMMAND_TYPE::LAND_TYPE:
				this->processCommandLand();
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
	}
	int UDPInterfaceRobot::generateStatePackage()
	{
		int reclen = this->receiveData(this->buffRecv,this->length_recv,this->adressRequest,this->lenAdress);
		
		if (reclen > 0)
		{
			if (this->packSendPtr == nullptr)
			{
				printf("Warning: not package of sending!");
				return reclen;
			}
			switch (this->packSendPtr->componentId)
			{
			case msg::COMPONENT_TYPE::STATE_TYPE:
				switch (this->packSendPtr->messageId)
				{
				case msg::MESSAGE_TYPE_STATE_TYPE::KINEMATIC_STATE_TYPE:
					this->generateKinematicStatePackage();
					break;
				case msg::MESSAGE_TYPE_STATE_TYPE::BATTERY_STATE_TYPE:
					this->generateBetteryPackage();
					break;
				default:
					break;
				}
				break;
			}
		}
		return reclen;
	}

	void UDPInterfaceRobot::processCommandSetMode(){}
	void UDPInterfaceRobot::processCommandArming() {}
	void UDPInterfaceRobot::processCommandTakeoff() {}
	void UDPInterfaceRobot::processCommandHover() {}
	void UDPInterfaceRobot::processCommandLand() {}
	void UDPInterfaceRobot::processCommandPosition() {}
	void UDPInterfaceRobot::processCommandVelocity() {}
	void UDPInterfaceRobot::processCommandAcceleration() {}
	void UDPInterfaceRobot::processCommandAttitude() {}
	void UDPInterfaceRobot::processCommandDefault() {}


	void UDPInterfaceRobot::generateKinematicStatePackage(){}
	void UDPInterfaceRobot::generateBetteryPackage(){}
}


