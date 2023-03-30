#ifndef _PX4_NODE
#define _PX4_NODE


#include"udp_common/UDPInterfaceRobot.h"
#include"udp_common/package_msg.h"
#include"ros/ros.h"
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/PositionTarget.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/BatteryState.h>
#include <tf/tf.h>



namespace drone
{
    constexpr int LEN_MAX_BUFFER_SEND = 1;
    constexpr int LEN_MAX_BUFFER_RECV = 1;
    class px4_node
    {
    private:  
        // ros::NodeHandle n_;
        // System state
        ros::Subscriber state_sub ; 
        // Flight state
        ros::Subscriber pose_sub;
        ros::Subscriber vel_sub;
        ros::Subscriber imu_sub;
        ros::Subscriber bat_sub;
        
        // Control
        ros::ServiceClient arming_client;   // Arming
        ros::ServiceClient set_mode_client; // Set mode
        ros::Publisher local_pub;           // Position control, velocity control, acceleration control  
        ros::Publisher attitude_pub;        // Attitude control

        // Listen package of commands and send package of states
        ros::Subscriber pack_cmd_sub;
        ros::Publisher  pack_sta_pub;
        ros::Publisher  pack_bat_pub;
        
        uint8_t systemId = 0;
        float x0 =0;
        float y0 =0;
        float z0 =0;
        float roll0 =0;
        float pitch0 =0;
        float yaw0 =0;

        //Command
        mavros_msgs::PositionTarget local_raw_pos_cmd;
        mavros_msgs::PositionTarget local_raw_vel_cmd;
        mavros_msgs::PositionTarget local_raw_acc_cmd;
        mavros_msgs::AttitudeTarget attitude_raw_cmd;
        mavros_msgs::SetMode offb_set_mode;
        mavros_msgs::CommandBool arm_cmd;
        
        //bool flag_trigger;                    //  Flag triggering takeoff, land, hover, etc. 

        float throttle_hover = 0.6106;
        float thrust_to_throttle_gain = 0.1;

        int command_pre;

        ros::Time last_request; 

        mavros_msgs::State _current_state;
        geometry_msgs::PoseStamped _current_pose;
        geometry_msgs::TwistStamped _current_vel;
        sensor_msgs::Imu _current_imu;
        sensor_msgs::BatteryState _current_bat;
        
        void state_cb(const mavros_msgs::State::ConstPtr& msg){_current_state = *msg;}
        void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){_current_pose = *msg;}
        void vel_cb(const geometry_msgs::TwistStamped::ConstPtr& msg){_current_vel = *msg;}
        void imu_cb(const sensor_msgs::Imu::ConstPtr& msg){_current_imu = *msg;}
        void bat_cb(const sensor_msgs::BatteryState::ConstPtr& msg){_current_bat = *msg;}

        void pack_cmd_cb(const udp_common::package_msg::ConstPtr& msg);
        
		msg::package pack_recv_command;
        // udp_common::package_msg pack_recv_command;
		udp_common::package_msg pack_send_bat;
        udp_common::package_msg pack_send_sstate;
    public:
        px4_node();
        ~px4_node();


        // Process packRecv received from simulink and send to robot
		void processCommand();
		void processCommandSetMode();
		void processCommandArming();
		void processCommandTakeoff();
		void processCommandHover();
		void processCommandLand();
		void processCommandPosition();
		void processCommandVelocityHori();
		void processCommandAcceleration();
		void processCommandAttitude();
		void processCommandDefault();


		// Process packSend using data received from robot and prepared to send to simulink
		int generateStatePackage(uint8_t componentId,uint8_t messageId);
		void generateBetteryPackage();
		void generateKinematicStatePackage();
        void generateSimpleStatePackage();
        void generateState12Package();
        // void connectToUDPInterfaceSimulink_command_ksate(msg::UDPInterfaceSimulink & udp_sim);

        // void connectToUDPInterfaceSimulink_bat(msg::UDPInterfaceSimulink & udp_sim);

        void init_px4_node(int ID,float x,float y,float z,float roll,float pitch,float yaw,
            float throttle_hover);

        // void pack_ssta_cb();
        void pack_sta12_cb();
        void pack_bat_cb();
    };
    
}


#endif