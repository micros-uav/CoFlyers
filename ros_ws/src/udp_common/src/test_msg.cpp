#include "ros/ros.h"   
#include "udp_common/package_msg.h"

int main(int argc, char*argv[]) 
{  
 
  ros::init(argc, argv, "talker");  
  
  ros::NodeHandle n;  

  ros::Publisher chatter_pub = n.advertise<udp_common::package_msg>("chatter", 1000);  

  ros::Rate loop_rate(10);  
   
  uint8_t count = 0;
  while (ros::ok())  
  {  
    udp_common::package_msg pack;

    pack.start = 0xFA;
    pack.len_payload = 1;
    pack.num_sequence = count;
    pack.system_id = 0;
    pack.component_id = 0;
    pack.message_id = 0;
    pack.msg_data = {1,2,3};
    pack.time_stamp = double(ros::Time::now().toNSec());
    pack.check_num = 111;
    count++;

    chatter_pub.publish(pack); 
    ros::spinOnce();  
    loop_rate.sleep();    
   }  
    return 0;  
} 
