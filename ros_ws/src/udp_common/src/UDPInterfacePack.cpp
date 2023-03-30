#include"udp_common/UDPInterfacePack.h"

namespace udpPack{
	UDPInterfacePack::UDPInterfacePack()
	{
		//this->initialzePackageBuffer();	
	}
	UDPInterfacePack::UDPInterfacePack(uint8_t systemId, const char*ipLocal,int portLocal, const char*ipTarget,int portTarget,
        int sizeBuffSend,int sizeBuffRecv)
    {
        //this->initialzePackageBuffer();
		this->initialzeSocket(ipLocal,portLocal,ipTarget,portTarget,
		sizeBuffSend,sizeBuffRecv);
		//this->packSend.systemId = systemId;  
    }
    
    UDPInterfacePack::~UDPInterfacePack()
    {
    }
	
	void UDPInterfacePack::initialzePackageBuffer()
	{
		/*this->packRecv.start = 0x00;
        this->packRecv.lenPayload = 0;
        this->packRecv.numSequence = 0;
        this->packRecv.systemId = 0;
        this->packRecv.componentId = 0;
        this->packRecv.messageId = 0;
        for (int i = 0; i < msg::LEN_MAX_MSG; i++)
        {
            this->packRecv.msgData[i] = 0;
        }
        this->packRecv.timeStamp = 0;
        this->packRecv.checkNum = 0;

        this->packSend.start = 0x00;
        this->packSend.lenPayload = 0;
        this->packSend.numSequence = 0;
        this->packSend.systemId = 0;
        this->packSend.componentId = 0;
        this->packSend.messageId = 0;
        for (int i = 0; i < msg::LEN_MAX_MSG; i++)
        {
            this->packSend.msgData[i] = 0;
        }
        this->packSend.timeStamp = 0;
        this->packSend.checkNum = 0;

        for (int i = 0; i < msg::LEN_MAX_BUFFER; i++)
        {
            this->buffRecv[i] = 0;
            this->buffSend[i] = 0;
        }*/
	}
	void UDPInterfacePack::initialzeSocket(const char*ipLocal,int portLocal, const char*ipTarget,int portTarget,
	int sizeBuffSend,int sizeBuffRecv)
	{
		// this->udpInterface.setIpPort(ipLocal,portLocal,ipTarget,portTarget,
		// sizeBuffSend,sizeBuffRecv);
		// this->udpInterface.initSocket();
		this->setIpPort(ipLocal,portLocal,ipTarget,portTarget,
		sizeBuffSend,sizeBuffRecv);
		//this->initSocket();
	}
    
    int UDPInterfacePack::packMsg(){
        int lenBuff = 0;
        if (this->packSendPtr->start == 0xFA)
        {
            int num = msg::getMsgNum(this->packSendPtr->componentId,this->packSendPtr->messageId);   
            if (num < 0)
                return 0;
            int lenPayload = num*4;
            memcpy(this->buffSend,this->packSendPtr,6);
            memcpy(&this->buffSend[6],this->packSendPtr->msgData,lenPayload);
            memcpy(&this->buffSend[6+lenPayload],&this->packSendPtr->timeStamp,10);
            lenBuff = lenPayload + 16;
        }
        return lenBuff;
    }
    int UDPInterfacePack::unpackMsg(){
        int lenBuff = 0;
        if ((uint8_t)this->buffRecv[0] == 0xFA)
        {
            memcpy(this->packRecvPtr,this->buffRecv,6);
            int lenPayload = (int)this->packRecvPtr->lenPayload;
            memcpy(this->packRecvPtr->msgData,&this->buffRecv[6],lenPayload);
            memcpy(&this->packRecvPtr->timeStamp,&this->buffRecv[6+lenPayload],10);
            lenBuff = lenPayload + 16;
        }
        return lenBuff;
    }

	
	void UDPInterfacePack::set_package_send_common(const uint8_t componentId,const uint8_t messageId)
	{
		int msgNum = msg::getMsgNum(componentId, messageId);

		this->packSendPtr->start = 0xFA;					//#0
		this->packSendPtr->lenPayload = msgNum * 4;			//#1, ==n
		this->packSendPtr->numSequence++;					//#2
		//this->packSendPtr->systemId;						//#3
		this->packSendPtr->componentId = componentId;		//#4
		this->packSendPtr->messageId = messageId;			//#5
		//memcpy(this->packSendPtr->msgData, msgData, this->packSendPtr->lenPayload);//#6-n+5
		getMillisecond(&(this->packSendPtr->timeStamp));    //#n+6 - n+13
		//this->packSendPtr->checkNum = this->get_check_num(this->packSend);//#n+14 - n+15
		this->set_package_send_check_num();
	}

	void UDPInterfacePack::set_package_send_check_num()
	{
		this->packSendPtr->checkNum = msg::get_check_num(*(this->packSendPtr));
	}

	void UDPInterfacePack::set_package_data(const float*msgData,const uint8_t componentId,const uint8_t messageId)
	{
		this->set_package_send_common(componentId,messageId);
		memcpy(this->packSendPtr->msgData, msgData, this->packSendPtr->lenPayload);//#6-n+5
		this->packSendPtr->checkNum = msg::get_check_num(*(this->packSendPtr));//#n+14 - n+15
	}


    int UDPInterfacePack::recvMsg(){
        int lenRecv = 0;
        struct sockaddr_in adressRequest;
        socklen_t lenAdress = sizeof(struct sockaddr_in);
        lenRecv = this->receiveData(this->buffRecv, msg::LEN_MAX_BUFFER, adressRequest, lenAdress);
        if (lenRecv > 0)
        {
            unpackMsg();
        }
        return lenRecv;
    }
    ssize_t UDPInterfacePack::sendMsg(){
        int lenBuff = packMsg();
        ssize_t retval = -1;
        retval = this->sendData(this->buffSend,lenBuff);
        return retval;
    }
    
	//uint16_t UDPInterfacePack::get_check_num(const  msg::package& myPackage)
	//{
	//	uint16_t checkNum = 111;
	//	// myPackage
	//	return checkNum;
	//}

 //   int UDPInterfacePack::getMsgNum(uint8_t componentId,uint8_t messageId){
	//	int num = 0;
	//	switch (componentId)
	//	{
	//	case  msg::COMMAND_TYPE:
	//		/* Control */
	//		switch (messageId)
	//		{
	//		case  msg::SET_MODE_TYPE:
	//			/* Set mode */
	//			num = msg::SET_MODE_TYPE_NUM;
	//			break;
	//		case  msg::ARMING_TYPE:
	//			/* Arming */
	//			num = msg::ARMING_TYPE_NUM;
	//			break;
	//		case  msg::TAKEOFF_TYPE:
	//			/* Takeoff */
	//			num = 1;
	//			break;
	//		case  msg::HOVER_TYPE:
	//			/* Hover */
	//			num = 1;
	//			break;
	//		case  msg::LAND_TYPE:
	//			/* Land */
	//			num = 1;
	//			break;
	//		case  msg::POSITION_CONTROL_TYPE:
	//			/* Position control */
	//			num = 4;
	//			break;
	//		case  msg::VELOCITY_CONTROL_TYPE:
	//			/* Velocity control */
	//			num = 4;
	//			break;
	//		case  msg::ACCELERATION_CONTROL_TYPE:
	//			/* Acceleration control */
	//			num = 4;
	//			break;
	//		case  msg::ATTITUDE_CONTROL_TYPE:
	//			/* Attitude control */
	//			num = 4;
	//			break;
	//		default:
	//			num = ERROR_MSG_NUM;
	//			break;
	//		}
	//		break;
	//	case  msg::STATE_TYPE:
	//		/*  */
	//		switch (messageId)
	//		{
	//		case  msg::KINEMATIC_STATE_TYPE:
	//			/* */
	//			num = 16;
	//			break;
	//		case  msg::BATTERY_STATE_TYPE:
	//			/* */
	//			num = 1;
	//			break;
	//		default:
	//			num = ERROR_MSG_NUM;
	//			break;
	//		}
	//		break;
	//	default:
	//		num = ERROR_MSG_NUM;
	//		break;
	//	}
	//	return num;
 //   }
}
