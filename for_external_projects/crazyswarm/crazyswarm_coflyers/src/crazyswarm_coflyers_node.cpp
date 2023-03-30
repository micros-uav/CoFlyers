#include "ros/ros.h" 
#include "crazyswarm_coflyers/package_msg.h"
#include "crazyswarm_coflyers/data_description.h"
#include"crazyswarm/Position.h"
#include"crazyswarm/VelocityWorld.h"
#include"geometry_msgs/Twist.h"
#include"crazyswarm/FullState.h"
#include"tf/tfMessage.h"
#include"geometry_msgs/Point.h"
#include"geometry_msgs/PoseStamped.h"


//==================State command==================//
std::vector<ros::Subscriber> subs_pose;
std::vector<ros::Subscriber> subs_command;
std::vector<ros::Publisher>  pubs_state;
std::vector<ros::Publisher>  pubs_bat;
std::vector<crazyswarm_coflyers::package_msg> packs_state;
std::vector<crazyswarm_coflyers::package_msg> packs_bat;
// std::vector<crazyswarm_coflyers::package_msg> packs_cmd;
std::vector<uint8_t> messages_id_pre;

////
void callback_pose(const geometry_msgs::PoseStamped::ConstPtr&msg, crazyswarm_coflyers::package_msg*pack,int ind);
void callback_package_cmd(const crazyswarm_coflyers::package_msg::ConstPtr&msg, uint8_t& message_id_pre,int ind);
void callback_package_bat(crazyswarm_coflyers::package_msg&pack, int ind);
void callback_time_out();

void process_takeoff(int ind,const std::vector<float>&command,bool trigger);
void process_hover(int ind,const std::vector<float>&command,bool trigger);
void process_land(int ind,const std::vector<float>&command,bool trigger);
void process_postion(int ind,const std::vector<float>&command,bool trigger);
void process_velocity(int ind,const std::vector<float>&command,bool trigger);
void process_velocity_horizontal(int ind,const std::vector<float>&command,bool trigger);
void process_attitude(int ind,const std::vector<float>&command,bool trigger);
void process_full_state(int ind,const std::vector<float>&command,bool trigger);
//==================Control==================//
std::vector<ros::Publisher>  pubs_c_pos;
std::vector<ros::Publisher>  pubs_c_vel;
std::vector<ros::Publisher>  pubs_c_att;
std::vector<ros::Publisher>  pubs_c_ful;
float Z_takeoff = 1.0;
float Z_land = 0.0;
float p_h_velocity_horizontal = 1.25;
std::vector<crazyswarm::Position> ps_hover;
std::vector<double> timestamps_pre_cmd;
std::vector<bool> flag_timeout;
double time_timeout = 0.8;
//==================Get velocities and accelerations==================//
unsigned int span_filter_v = 10;
unsigned int span_filter_a = 10;
std::vector<double> time_stamp_t_1_s;          // time stamps at time t-1 for getting delta_t
std::vector<geometry_msgs::Point> pos_t_1_s;   // positions at time t-1 for getting velocities by difference
std::vector<geometry_msgs::Point> vel_f_t_1_s; // filtered velocities at time t-1 for getting accelerations by difference and getting filtered velocities at time t
std::vector<geometry_msgs::Point> acc_f_t_1_s; // filtered velocities at time t-1 for getting accelerations by difference and getting filtered accelerations at time t
std::vector<std::vector<geometry_msgs::Point>> vel_s_series; // historical velocities between time t-1 and time t-s_fv for filtering velocityies
std::vector<std::vector<geometry_msgs::Point>> acc_s_series; // historical accelerations between time t-1 and time t-s_fa for filtering accelerations
std::vector<unsigned int> count_v_s, count_a_s;

//
int main(int argc, char **argv)
{
    ros::init(argc,argv,"crazyswarm_coflyers_node");
    ros::NodeHandle n;
    
    span_filter_v = atoi(argv[1]);
    span_filter_a = atoi(argv[2]);
    ROS_INFO("Span for filtering velocities is %d",span_filter_v);
    ROS_INFO("Span for filtering accelerations is %d",span_filter_a);
    //Get configuration
    std::vector<int> cfIds;
    XmlRpc::XmlRpcValue crazyflies;;
    n.getParam("crazyflies", crazyflies);
    ROS_ASSERT(crazyflies.getType() == XmlRpc::XmlRpcValue::TypeArray);

    for (int32_t i = 0; i < crazyflies.size(); ++i)
    {
        ROS_ASSERT(crazyflies[i].getType() == XmlRpc::XmlRpcValue::TypeStruct);
        XmlRpc::XmlRpcValue crazyflie = crazyflies[i];
        int id = crazyflie["id"];
        int channel = crazyflie["channel"];
        std::vector<int>::iterator result = find(cfIds.begin(),cfIds.end(),id);
        if (result != cfIds.end()) {
        ROS_FATAL("CF with the same id twice in configuration!");
        return 1;
        }
        cfIds.push_back(id);
    }
    //==================Init ros topic==================//
    int num = cfIds.size();
    subs_pose.resize(num);
    subs_command.resize(num);
    pubs_state.resize(num);
    pubs_bat.resize(num);
    packs_state.resize(num);
    packs_bat.resize(num);
    // packs_cmd.resize(num);
    messages_id_pre.resize(num);
    ros::Timer *timer_bat = new ros::Timer[num];
    pubs_c_pos.resize(num);
    pubs_c_vel.resize(num);
    pubs_c_att.resize(num);
    pubs_c_ful.resize(num);
    ps_hover.resize(num);
    time_stamp_t_1_s.resize(num);          
    pos_t_1_s.resize(num);   
    vel_f_t_1_s.resize(num); 
    acc_f_t_1_s.resize(num); 
    vel_s_series.resize(num);
    acc_s_series.resize(num);
    count_v_s.resize(num); 
    count_a_s.resize(num);
    for (int i = 0; i < num; i++)
    {
        vel_s_series[i].resize(span_filter_v);
        acc_s_series[i].resize(span_filter_a);
    }

    for (int i = 0; i < num; i++)
    {
        char str[100]{};
        sprintf(str,"/cf%d/pose",cfIds[i]);
        subs_pose[i] = n.subscribe<geometry_msgs::PoseStamped>(str,1,boost::bind(&callback_pose,_1, &(packs_state[i]),i));
        sprintf(str,"/uav%d/coflyers/command_state/package_listener",i);
        subs_command[i] = n.subscribe<crazyswarm_coflyers::package_msg>(str,1,boost::bind(&callback_package_cmd,_1,messages_id_pre[i],i));
        sprintf(str,"/uav%d/coflyers/command_state/package_sender",i);
        pubs_state[i] = n.advertise<crazyswarm_coflyers::package_msg>(str,1);
        sprintf(str,"/uav%d/coflyers/battery/package_sender",i);
        pubs_bat[i] = n.advertise<crazyswarm_coflyers::package_msg>(str,1);
        timer_bat[i] = n.createTimer(ros::Duration(1.0),boost::bind(&callback_package_bat,packs_bat[i],i));
        ////
        sprintf(str,"/cf%d/cmd_position",cfIds[i]);
        pubs_c_pos[i] = n.advertise<crazyswarm::Position>(str,1);
        sprintf(str,"/cf%d/cmd_velocity_world",cfIds[i]);
        pubs_c_vel[i] = n.advertise<crazyswarm::VelocityWorld>(str,1);
        sprintf(str,"/cf%d/cmd_vel",cfIds[i]);
        pubs_c_att[i] = n.advertise<geometry_msgs::Twist>(str,1);
        sprintf(str,"/cf%d/cmd_full_state",cfIds[i]);
        pubs_c_ful[i] = n.advertise<crazyswarm::FullState>(str,1);
    }
    
    timestamps_pre_cmd.resize(num);
    flag_timeout.resize(num);
    for (int i = 0; i < num; i++)
    {
        flag_timeout[i] = false;
    }
    
    ros::Timer timer_timeout = n.createTimer(ros::Duration(1.0/30.0),boost::bind(&callback_time_out));

    ros::spin();

    return 0;
}

void callback_pose(const geometry_msgs::PoseStamped::ConstPtr&msg, crazyswarm_coflyers::package_msg*pack,int ind)
{
    // ROS_INFO("pose: %d,%f,%f,%f,%f,%f,%f,%f,%f",
    //     ind,msg->pose.position.x,msg->pose.position.y,msg->pose.position.z,
    //     msg->pose.orientation.w,msg->pose.orientation.x,msg->pose.orientation.y,msg->pose.orientation.z);
    pack->start = 0xFA;
    pack->num_sequence++;
    pack->system_id = ind;
    pack->component_id = msg::STATE_TYPE;
    pack->message_id = msg::STATE_12_TYPE;
    int num = msg::getMsgNum(pack->component_id,pack->message_id);
    pack->msg_data.resize(num);
    pack->len_payload  = num*4;
    pack->msg_data[0]  = msg->pose.position.x;
    pack->msg_data[1]  = msg->pose.position.y;
    pack->msg_data[2]  = msg->pose.position.z;
    // Get Velocities and accelerations
    double t = ros::Time::now().toSec();
    double dt = t - time_stamp_t_1_s[ind];
    geometry_msgs::Point pos, vel, vel_f, acc, acc_f;
    geometry_msgs::Point* p_t_1_ptr = &pos_t_1_s[ind],
    *v_f_t_1_ptr = &vel_f_t_1_s[ind],
    *a_f_t_1_ptr = &acc_f_t_1_s[ind];
    //
    pos = msg->pose.position;
    vel.x = (pos.x - p_t_1_ptr->x)/dt;
    vel.y = (pos.y - p_t_1_ptr->y)/dt;
    vel.z = (pos.z - p_t_1_ptr->z)/dt;
    unsigned int*count_v_ptr = &count_v_s[ind];
    geometry_msgs::Point*v_oldest_ptr= &(vel_s_series[ind][*count_v_ptr]);
    vel_f.x = v_f_t_1_ptr->x - (v_oldest_ptr->x - vel.x)/span_filter_v;
    vel_f.y = v_f_t_1_ptr->y - (v_oldest_ptr->y - vel.y)/span_filter_v;
    vel_f.z = v_f_t_1_ptr->z - (v_oldest_ptr->z - vel.z)/span_filter_v;
    //
    acc.x = (vel_f.x - v_f_t_1_ptr->x)/dt;
    acc.y = (vel_f.y - v_f_t_1_ptr->y)/dt;
    acc.z = (vel_f.z - v_f_t_1_ptr->z)/dt;
    unsigned int*count_a_ptr = &count_a_s[ind];
    geometry_msgs::Point*a_oldest_ptr= &(acc_s_series[ind][*count_a_ptr]);
    acc_f.x = a_f_t_1_ptr->x - (a_oldest_ptr->x - acc.x)/span_filter_a;
    acc_f.y = a_f_t_1_ptr->y - (a_oldest_ptr->y - acc.y)/span_filter_a;
    acc_f.z = a_f_t_1_ptr->z - (a_oldest_ptr->z - acc.z)/span_filter_a;
    //
    time_stamp_t_1_s[ind] = t;
    *p_t_1_ptr = pos;
    *v_f_t_1_ptr = vel_f;
    *a_f_t_1_ptr = acc_f;
    *v_oldest_ptr = vel;
    *a_oldest_ptr = acc;
    *count_v_ptr = ((*count_v_ptr) + 1)%span_filter_v;
    *count_a_ptr = ((*count_a_ptr) + 1)%span_filter_a;
    //
    pack->msg_data[3]  = vel_f.x;
    pack->msg_data[4]  = vel_f.y;
    pack->msg_data[5]  = vel_f.z;
    pack->msg_data[6]  = acc_f.x;
    pack->msg_data[7]  = acc_f.y;
    pack->msg_data[8]  = acc_f.z;

    // 
    float qw = msg->pose.orientation.w,
     qx = msg->pose.orientation.x,
     qy = msg->pose.orientation.y,
     qz = msg->pose.orientation.z;
    pack->msg_data[9]  = atan2(-2.0*qx*qy+2.0*qw*qz, -2.0*qx*qx-2*qz*qz+1.0); //Yaw
    pack->msg_data[10] = asin(2.0*qw*qx + 2.0*qy*qz);//Roll
    pack->msg_data[11] = atan2(-2.0*qx*qz+2.0*qw*qy, -2.0*qx*qx-2*qy*qy+1.0);//Pitch
    pack->time_stamp = ros::Time::now().toNSec();
    pack->check_num = 111;
    pubs_state[ind].publish(*pack);
    // ROS_INFO("packs_state: %d,%d",packs_state[ind].component_id,packs_state[ind].message_id);   
}
void callback_package_bat(crazyswarm_coflyers::package_msg&pack, int ind)
{
    pack.start = 0xFA;
    pack.num_sequence++;
    pack.system_id = ind;
    pack.component_id = msg::STATE_TYPE;
    pack.message_id = msg::BATTERY_STATE_TYPE;
    int num = msg::getMsgNum(pack.component_id,pack.message_id);
    pack.msg_data.resize(num);
    pack.len_payload = num*4;
    pack.msg_data[0] = 1.0;
    pack.time_stamp = ros::Time::now().toNSec();
    pack.check_num = 111;
    pubs_bat[ind].publish(pack);
}

void callback_time_out()
{
    for (int i = 0; i < timestamps_pre_cmd.size(); i++)
    {
        if (ros::Time::now().toSec() - timestamps_pre_cmd[i] > time_timeout)
        {
            if (packs_state[i].msg_data.size()>0)
            {
                if (!flag_timeout[i])
                {
                    ps_hover[i].x   = packs_state[i].msg_data[0];
                    ps_hover[i].y   = packs_state[i].msg_data[1];
                    ps_hover[i].z   = packs_state[i].msg_data[2];
                    ps_hover[i].yaw = packs_state[i].msg_data[9]*180.0/3.1415926;
                }
                flag_timeout[i] = true;
                crazyswarm::Position p;
                p.x = ps_hover[i].x;
                p.y = ps_hover[i].y;
                // p.z = Z_land;
                p.z = 0.15;
                p.yaw = ps_hover[i].yaw;
                if (packs_state[i].msg_data[2] - p.z>0.05)
                {
                    pubs_c_pos[i].publish(p);
                }
            }
        }
        else
        {
            flag_timeout[i] = false;
        }
    }
}


void callback_package_cmd(const crazyswarm_coflyers::package_msg::ConstPtr&msg, uint8_t& message_id_pre, int ind)
{
    if (msg->start == 0xFA)
    {
        // ROS_INFO("UAV%d,%d",ind,msg->msg_data.size());
        uint8_t component_id = msg->component_id;
        if (component_id == msg::COMMAND_TYPE)
        {
            timestamps_pre_cmd[ind] = ros::Time::now().toSec();
            uint8_t message_id = msg->message_id;
            bool trigger_cmd = message_id_pre != message_id;
            message_id_pre = message_id;
            switch (message_id)
            {
            case msg::TAKEOFF_TYPE:
                process_takeoff(ind, msg->msg_data, trigger_cmd);
                break;
            case msg::HOVER_TYPE:
                process_hover(ind, msg->msg_data, trigger_cmd);
                break;
            case msg::LAND_TYPE:
                process_land(ind, msg->msg_data, trigger_cmd);
                break;
            case msg::POSITION_CONTROL_TYPE:
                process_postion(ind, msg->msg_data, trigger_cmd);
                break;
            case msg::VELOCITY_CONTROL_TYPE:
                process_velocity(ind, msg->msg_data, trigger_cmd);
                break;
            case msg::VELOCITY_HORIZONTAL_CONTROL_TYPE:
                process_velocity_horizontal(ind, msg->msg_data, trigger_cmd);
                break;
            case msg::ATTITUDE_CONTROL_TYPE:
                process_attitude(ind, msg->msg_data, trigger_cmd);
                break;
            case msg::FULL_STATE_CONTROL_TYPE:
                process_full_state(ind, msg->msg_data, trigger_cmd);
                break;
            default:
                break;
            }
        }
    }
}

void process_takeoff(int ind,const std::vector<float>&command,bool trigger)
{
    // printf("takeoff %d,%d,%d\n",packs_state[ind].msg_data.size(),packs_state[ind].component_id,packs_state[ind].message_id);
    if (packs_state[ind].msg_data.size()>0)
    {
        if (trigger)
        {
            ps_hover[ind].x   = packs_state[ind].msg_data[0];
            ps_hover[ind].y   = packs_state[ind].msg_data[1];
            ps_hover[ind].z   = packs_state[ind].msg_data[2];
            ps_hover[ind].yaw = packs_state[ind].msg_data[9]*180.0/3.1415926;
            ROS_INFO("UAV%d takeoff.",ind);
        }
        crazyswarm::Position p;
        p.x = ps_hover[ind].x;
        p.y = ps_hover[ind].y;
        // p.z = Z_takeoff;
        p.z = command[0];
        p.yaw = ps_hover[ind].yaw;
        // ROS_INFO("%f,%f,%f,%f",p.x,p.y,p.z,p.yaw);
        pubs_c_pos[ind].publish(p);
    }
}
void process_hover(int ind,const std::vector<float>&command,bool trigger)
{
    if (packs_state[ind].msg_data.size()>0)
    {
        if (trigger)
        {
            ps_hover[ind].x   = packs_state[ind].msg_data[0];
            ps_hover[ind].y   = packs_state[ind].msg_data[1];
            ps_hover[ind].z   = packs_state[ind].msg_data[2];
            ps_hover[ind].yaw = packs_state[ind].msg_data[9]*180.0/3.1415926;
            ROS_INFO("UAV%d hover.",ind);
        }
        pubs_c_pos[ind].publish(ps_hover[ind]);
    }
}
void process_land(int ind,const std::vector<float>&command,bool trigger)
{
    if (packs_state[ind].msg_data.size()>0)
    {
        if (trigger)
        {
            ps_hover[ind].x   = packs_state[ind].msg_data[0];
            ps_hover[ind].y   = packs_state[ind].msg_data[1];
            ps_hover[ind].z   = packs_state[ind].msg_data[2];
            ps_hover[ind].yaw = packs_state[ind].msg_data[9]*180.0/3.1415926;
            ROS_INFO("UAV%d land.",ind);
        }
        crazyswarm::Position p;
        p.x = ps_hover[ind].x;
        p.y = ps_hover[ind].y;
        // p.z = Z_land;
        p.z = command[0]+0.15;
        p.yaw = ps_hover[ind].yaw;
        if (packs_state[ind].msg_data[2] - p.z>0.05)
        {
            pubs_c_pos[ind].publish(p);
        }
    }
}
void process_postion(int ind,const std::vector<float>&command,bool trigger)
{
    if (trigger){ROS_INFO("UAV%d position control.",ind);}
    crazyswarm::Position p;
    p.x = command[0];
    p.y = command[1];
    p.z = command[2];
    p.yaw = command[3]*180/3.1415926;
    pubs_c_pos[ind].publish(p);
}
void process_velocity(int ind,const std::vector<float>&command,bool trigger)
{
    if (trigger){ROS_INFO("UAV%d velocity control.",ind);}
    crazyswarm::VelocityWorld v;
    v.vel.x = command[0];
    v.vel.y = command[1];
    v.vel.z = command[2];
    v.yawRate = command[3];
    pubs_c_vel[ind].publish(v);
}

void process_velocity_horizontal(int ind,const std::vector<float>&command,bool trigger)
{
    if (packs_state[ind].msg_data.size()>0)
    {
        if (trigger){ROS_INFO("UAV%d velocity horizontal control.",ind);}
        // p_h_velocity_horizontal
        crazyswarm::VelocityWorld v;
        v.vel.x = command[0];
        v.vel.y = command[1];
        v.vel.z = p_h_velocity_horizontal * (command[2] - packs_state[ind].msg_data[2]);
        v.yawRate = command[3];
        pubs_c_vel[ind].publish(v);
        // ROS_INFO("UAV %d v_h:%f,%f,%f,%f",ind,v.vel.x,v.vel.y,v.vel.z,v.yawRate);
    }
}

void process_attitude(int ind,const std::vector<float>&command,bool trigger)
{
    if (trigger){ROS_INFO("UAV%d attitude control.",ind);}
    geometry_msgs::Twist att;
    att.angular.z = command[0]; // yaw_rate
    att.linear.y = command[1]; // Roll
    att.linear.x = command[2]; // Pitch
    att.linear.z  = command[3]; // Thrust
    pubs_c_att[ind].publish(att);
}
void process_full_state(int ind,const std::vector<float>&command,bool trigger)
{
    if (trigger){ROS_INFO("UAV%d full state control.",ind);}
    crazyswarm::FullState fs;
    fs.pose.position.x = command[0];
    fs.pose.position.y = command[1];
    fs.pose.position.z = command[2];
    fs.twist.linear.x  = command[3];
    fs.twist.linear.y  = command[4];
    fs.twist.linear.z  = command[5];
    fs.acc.x           = command[6];
    fs.acc.y           = command[7];
    fs.acc.z           = command[8];
    fs.pose.orientation.w = command[9];
    fs.pose.orientation.x = command[10];
    fs.pose.orientation.y = command[11];
    fs.pose.orientation.z = command[12];
    fs.twist.angular.x    = command[13];
    fs.twist.angular.y    = command[14];
    fs.twist.angular.z    = command[15];
    pubs_c_ful[ind].publish(fs);
}