#include"ros/ros.h"

#include"udp_common/UDPInterfaceBase.h"

int main(int argc, char*argv[])
{
    ros::init(argc,argv,"test_base 2");
    ros::NodeHandle n;
    
    ROS_INFO("UDP node2.");

    socketTools::UDPInterfaceBase udp_sender("127.0.0.1",25001,"127.0.0.1",25000,128,128);

    udp_sender.initSocket();
    
    char buffSend[3]{};
    char buffRecv[128]{};

    sockaddr_in add;
    socklen_t add_len = sizeof(add);

    
    
    ros::Rate(30);

    while (ros::ok())
    {
        udp_sender.sendData(buffSend,3);
        buffSend[0] = buffSend[0] + 1;
        
        int len = udp_sender.receiveData(buffRecv,128, add, add_len);
        if (len > 0)
        {
            printf("Receive from %s: %s.\n", inet_ntoa(add.sin_addr), buffRecv);
        }

        ros::spinOnce();
    }
    
    return 0;
}