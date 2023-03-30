#include"ros/ros.h"

#include"udp_common/UDPInterfacePack.h"

int main(int argc, char*argv[])
{
    ros::init(argc,argv,"test_pack2");
    ros::NodeHandle n;
    
    ROS_INFO("UDP node2.");

    int id = 0;
    udpPack::UDPInterfacePack udp_node(id,"192.168.2.3",21000,"192.168.2.4",21001,
        msg::LEN_MAX_BUFFER,msg::LEN_MAX_BUFFER);
    udp_node.initSocket();
    

    msg::package packSend;
    msg::package packRecv;
    udp_node.packRecvPtr = &packRecv;
    udp_node.packSendPtr = &packSend;

    packSend.componentId = msg::COMPONENT_TYPE::COMMAND_TYPE;  
    packSend.messageId = msg::MESSAGE_TYPE_COMMAND_TYPE::VELOCITY_HORIZONTAL_CONTROL_TYPE;      
    packSend.checkNum = 111;
    packSend.lenPayload = 4*msg::VELOCITY_HORIZONTAL_CONTROL_TYPE_NUM;
    packSend.msgData[0] = 1;
    packSend.msgData[1] = 2;
    packSend.msgData[2] = 3;
    packSend.msgData[3] = 4;

    ros::Rate(30);

    while (ros::ok())
    {
        packSend.timeStamp = (double)ros::Time::now().toNSec();
        
        udp_node.sendMsg();
        
        
        int len = udp_node.recvMsg();
        if (len > 0)
        {
            printf("Received timestamp: %lf,cid: %d, mid: %d, msg:", packRecv.timeStamp,
                packRecv.componentId, packRecv.messageId);
            for (int i = 0; i < udp_node.packRecvPtr->lenPayload/4; i++)
            {
                printf(" %f,",udp_node.packRecvPtr->msgData[i]);
            }
            printf("\n");
        }

        ros::spinOnce();
    }
    
    return 0;
}