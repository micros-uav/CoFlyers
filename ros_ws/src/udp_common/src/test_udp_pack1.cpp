#include"ros/ros.h"

#include"udp_common/UDPInterfacePack.h"

int main(int argc, char*argv[])
{
    ros::init(argc,argv,"test_pack1");
    ros::NodeHandle n;
    
    ROS_INFO("UDP node1.");

    int id = 0;
    udpPack::UDPInterfacePack udp_node(id,"192.168.2.3",21001,"192.168.2.4",22000,
        msg::LEN_MAX_BUFFER,msg::LEN_MAX_BUFFER);
    udp_node.initSocket();
    

    msg::package packSend;
    msg::package packRecv;
    udp_node.packRecvPtr = &packRecv;
    udp_node.packSendPtr = &packSend;

    packSend.componentId = msg::COMPONENT_TYPE::STATE_TYPE;  
    packSend.messageId = msg::MESSAGE_TYPE_STATE_TYPE::BATTERY_STATE_TYPE;      
    packSend.checkNum = 111;
    packSend.lenPayload = 4*msg::BATTERY_STATE_TYPE_NUM;
    packSend.msgData[0] = 10;


    ros::Rate(30);

    while (ros::ok())
    {
        packSend.timeStamp = (double)ros::Time::now().toNSec();
        packSend.msgData[0] = packSend.msgData[0] + 1.0/30.0;

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