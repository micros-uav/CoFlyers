#include "ros/ros.h"   
#include "udp_common/package_msg.h"
#include "udp_common/UDPInterfacePack.h"


char ip_local[16]{},ip_target[16]{};
int port_local = 21001, port_target = 20000;
bool flag_recv = true, flag_send = true;
udpPack::UDPInterfacePack udp_node;
msg::package packSend;
msg::package packRecv;
bool getAndCheckParams(int argc,char **argv);
void package_sender_callback(const udp_common::package_msg& pack_msg);


int main(int argc, char **argv)
{
    // Get ip addresses and ports from input
    if (!getAndCheckParams(argc,argv))
    {
        return -1;
    }
    else
    {
        ROS_INFO("Local ip:%s@%d, target ip:%s@%d",
        ip_local,port_local,ip_target,port_target);
    }

    // Create udp socket
    udp_node.initialzeSocket(ip_local, port_local, ip_target, port_target,
        msg::LEN_MAX_BUFFER,msg::LEN_MAX_BUFFER);
    udp_node.initSocket();
    udp_node.packRecvPtr = &packRecv;
    udp_node.packSendPtr = &packSend;


    ros::init(argc,argv,"udp_node");
    ros::NodeHandle n;
    // Create ros topic
    ros::Publisher listener;
    ros::Subscriber sender;
    if (flag_recv)
    {
        listener = n.advertise<udp_common::package_msg>("package_listener",1);
    }
    if (flag_send)
    {
        sender = n.subscribe("package_sender",1, package_sender_callback);
    }
    udp_common::package_msg pack_msg_rec;

    //
    ros::Rate rate(30);
    while (ros::ok())
    {
        if (flag_recv)
        {
            int len = udp_node.recvMsg();
            if (len > 0)
            {
                pack_msg_rec.start = packRecv.start;
                pack_msg_rec.len_payload = packRecv.lenPayload;
                pack_msg_rec.num_sequence = packRecv.numSequence;
                pack_msg_rec.system_id = packRecv.systemId;
                pack_msg_rec.component_id = packRecv.componentId;
                pack_msg_rec.message_id = packRecv.messageId;
                if (pack_msg_rec.msg_data.size() != packRecv.lenPayload/4)
                {
                    pack_msg_rec.msg_data.resize(packRecv.lenPayload/4);
                }
                for (int i = 0; i < pack_msg_rec.len_payload/4; i++)
                {
                    pack_msg_rec.msg_data[i] = packRecv.msgData[i];
                }
                pack_msg_rec.time_stamp = packRecv.timeStamp;
                pack_msg_rec.check_num = packRecv.checkNum;
                listener.publish(pack_msg_rec);
            }
        }
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}


void package_sender_callback(const udp_common::package_msg& pack_msg)
{
    packSend.start       = 0xFA;
    packSend.numSequence = pack_msg.num_sequence;
    packSend.systemId    = pack_msg.system_id;
    packSend.componentId = pack_msg.component_id;
    packSend.messageId   = pack_msg.message_id;
    int num = msg::getMsgNum(packSend.componentId,packSend.messageId);
    for (int i = 0; i < num; i++)
    {
        packSend.msgData[i] = pack_msg.msg_data[i];
    }
    packSend.timeStamp = pack_msg.time_stamp;
    packSend.lenPayload  = num*4;
    packSend.checkNum = msg::get_check_num(packSend);
    udp_node.sendMsg();
}

bool getAndCheckParams(int argc,char **argv)
{
    memcpy(ip_local,argv[1],256);
    memcpy(ip_target,argv[3],256);
    port_local = atoi(argv[2]);
    port_target = atoi(argv[4]);
    flag_recv = (bool)atoi(argv[5]);
    flag_send = (bool)atoi(argv[6]);

    if ( ((port_local < 1025) || (port_local > 65535)) && (port_local != 0))
    {ROS_ERROR("The UDP port of local must be from 1025 to 65535!");return false;}
    if ( ((port_target < 1025) || (port_target > 65535)) && (port_target != 0))
    {ROS_ERROR("The UDP port of local must be from 1025 to 65535!");return false;}
    if(strcmp(ip_local,"255.255.255.255")!=0){
        if(inet_addr(ip_local)==INADDR_NONE){
            ROS_ERROR("The format of the local IP address is wrong!"); 
            return false; 			
        }
    }
    if(strcmp(ip_target,"255.255.255.255")!=0){
        if(inet_addr(ip_target)==INADDR_NONE){
            ROS_ERROR("The format of the local IP address is wrong!"); 
            return false; 			
        }
    }
    return true;
}