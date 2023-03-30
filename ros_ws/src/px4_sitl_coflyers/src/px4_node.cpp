#include"px4_sitl_coflyers/px4_node.h"


namespace drone
{
    px4_node::px4_node()
    {
    }
    
    px4_node::~px4_node()
    {
    }
    
    void px4_node::init_px4_node(int ID,float x,float y,float z,float roll,float pitch,float yaw,
            float throttle_hover)
    {
        this->systemId = ID;
        this->x0 = x;
        this->y0 = y;
        this->z0 = z;
        this->roll0 = roll;
        this->pitch0 = pitch;
        this->yaw0 = yaw;
        this->throttle_hover = throttle_hover;

        ros::NodeHandle nh;


        char str[100]{};
         // System state
        sprintf(str,"/uav%d/mavros/state",this->systemId);
        this->state_sub = nh.subscribe<mavros_msgs::State>(str, 1, &px4_node::state_cb,this);

        // Flight state
        sprintf(str,"/uav%d/mavros/local_position/pose",this->systemId);
        this->pose_sub = nh.subscribe<geometry_msgs::PoseStamped>(str, 1, &px4_node::pose_cb,this);
        sprintf(str,"/uav%d/mavros/local_position/velocity_local",this->systemId);
        this->vel_sub = nh.subscribe<geometry_msgs::TwistStamped>(str, 1, &px4_node::vel_cb,this);
        sprintf(str,"/uav%d/mavros/imu/data",this->systemId);
        this->imu_sub = nh.subscribe<sensor_msgs::Imu>(str,1,&px4_node::imu_cb,this);
        sprintf(str,"/uav%d/mavros/battery",this->systemId);
        this->bat_sub = nh.subscribe<sensor_msgs::BatteryState>(str,1,&px4_node::bat_cb,this);
        
        //Control
        sprintf(str,"/uav%d/mavros/cmd/arming",this->systemId);
        this->arming_client = nh.serviceClient<mavros_msgs::CommandBool>(str);// Arming
        sprintf(str,"/uav%d/mavros/set_mode",this->systemId);
        this->set_mode_client = nh.serviceClient<mavros_msgs::SetMode>(str);// Set mode
        sprintf(str,"/uav%d/mavros/setpoint_raw/local",this->systemId);
        this->local_pub = nh.advertise<mavros_msgs::PositionTarget>(str,1);               // Position control, velocity control, acceleration control  
        sprintf(str,"/uav%d/mavros/setpoint_raw/attitude",this->systemId);
        this->attitude_pub = nh.advertise<mavros_msgs::AttitudeTarget>(str,1);        // Attitude control
        
        // Communicate with common udp interface
        this->pack_cmd_sub = nh.subscribe<udp_common::package_msg>("command_state/package_listener", 1, &px4_node::pack_cmd_cb,this);
        this->pack_sta_pub = nh.advertise<udp_common::package_msg>("command_state/package_sender",1);
        this->pack_bat_pub = nh.advertise<udp_common::package_msg>("battery/package_sender",1);

        // Control variables published to mavros/setpoint_raw/local
        this->local_raw_pos_cmd.type_mask = 0b100111111000;
        this->local_raw_pos_cmd.coordinate_frame = 1; //1:LOCAL_NED
        this->local_raw_pos_cmd.position.x = 0;
        this->local_raw_pos_cmd.position.y = 0;
        this->local_raw_pos_cmd.position.z = 2;
        this->local_raw_pos_cmd.yaw = 0;

        this->local_raw_vel_cmd.type_mask = 0b010111100011;
        this->local_raw_vel_cmd.coordinate_frame = 1; //1:LOCAL_NED 8:BODY_NED
        this->local_raw_vel_cmd.velocity.x = 0;
        this->local_raw_vel_cmd.velocity.y = 0;
        this->local_raw_vel_cmd.velocity.z = 0;
        this->local_raw_vel_cmd.yaw_rate = 0;

        this->local_raw_acc_cmd.type_mask = 0b010000111111;
        this->local_raw_acc_cmd.coordinate_frame = 1; //1:LOCAL_NED 8:BODY_NED
        this->local_raw_acc_cmd.acceleration_or_force.x = 0;
        this->local_raw_acc_cmd.acceleration_or_force.y = 0;
        this->local_raw_acc_cmd.acceleration_or_force.z = 0;
        this->local_raw_acc_cmd.yaw_rate = 0;
        
        this->attitude_raw_cmd.type_mask = 0b00000011;
        this->attitude_raw_cmd.orientation.w = 1;
        this->attitude_raw_cmd.orientation.x = 0;
        this->attitude_raw_cmd.orientation.y = 0;
        this->attitude_raw_cmd.orientation.z = 0;
        this->attitude_raw_cmd.thrust = this->throttle_hover;
        // this->flag_trigger = false;

        this->offb_set_mode.request.custom_mode = "OFFBOARD";

        this->arm_cmd.request.value = true;

        this->last_request = ros::Time::now().fromSec(0);

        this->command_pre = -1;
    }



    // Process packRecv received from simulink and send to robot
    void px4_node:: processCommand()
    {
        switch (this->pack_recv_command.componentId)
		{
		//case msg::COMPONENT_TYPE::STATE_TYPE:
		//	break;
		case msg::COMPONENT_TYPE::COMMAND_TYPE:
			switch (this->pack_recv_command.messageId)
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

			case msg::MESSAGE_TYPE_COMMAND_TYPE::VELOCITY_HORIZONTAL_CONTROL_TYPE:
				this->processCommandVelocityHori();
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
        this->command_pre = this->pack_recv_command.messageId;
    }
    void px4_node:: processCommandSetMode()
    {
        // ROS_INFO("Set_mode.");
        if( _current_state.mode != "OFFBOARD" &&
                    (ros::Time::now() - this->last_request > ros::Duration(5.0)))
        {
            local_raw_pos_cmd.position.z = 2.0;
            for(int i = 100; ros::ok() && i > 0; --i){
                this->local_pub.publish(this->local_raw_pos_cmd);
                ros::spinOnce();
                // sleep(0.01);
            }
            if( this->set_mode_client.call(this->offb_set_mode) &&
                this->offb_set_mode.response.mode_sent)
            {
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        }
    }
    void px4_node:: processCommandArming()
    {
        ROS_INFO("Arming.");
        if( _current_state.mode != "OFFBOARD" &&
                    (ros::Time::now() - this->last_request > ros::Duration(5.0)))
        {
            local_raw_pos_cmd.position.z = 2.0;
            for(int i = 100; ros::ok() && i > 0; --i){
                this->local_pub.publish(this->local_raw_pos_cmd);
                ros::spinOnce();
                // sleep(0.01);
            }
            if( this->set_mode_client.call(this->offb_set_mode) &&
                this->offb_set_mode.response.mode_sent)
            {
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        }
        else
        {
            if( !_current_state.armed &&
                        (ros::Time::now() - this->last_request > ros::Duration(5.0)))
            {
                if( this->arming_client.call(this->arm_cmd) &&
                    this->arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }

        }
}
    void px4_node:: processCommandTakeoff()
    {
        // ROS_INFO("Arming.");
        if( _current_state.mode != "OFFBOARD" &&
                    (ros::Time::now() - this->last_request > ros::Duration(5.0)))
        {
            printf("1");
            local_raw_pos_cmd.position.z = 2.0;
            for(int i = 100; ros::ok() && i > 0; --i){
                this->local_pub.publish(this->local_raw_pos_cmd);
                ros::spinOnce();
                // sleep(0.01);
            }
            if( this->set_mode_client.call(this->offb_set_mode) &&
                this->offb_set_mode.response.mode_sent)
            {
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        }
        else
        {
            if( !_current_state.armed &&
                        (ros::Time::now() - this->last_request > ros::Duration(5.0)))
            {
                if( this->arming_client.call(this->arm_cmd) &&
                    this->arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }

        }
        if (this->command_pre != this->pack_recv_command.messageId){
            
            this->local_raw_pos_cmd.position.x = _current_pose.pose.position.x;
            this->local_raw_pos_cmd.position.y = _current_pose.pose.position.y;
            this->local_raw_pos_cmd.position.z = 2.0;
            ROS_INFO("Takeoff in the position of %lf,%lf,%lf",this->local_raw_pos_cmd.position.x,
                this->local_raw_pos_cmd.position.y,
                this->local_raw_pos_cmd.position.z);
            // this->flag_trigger = true;
        }
        this->local_pub.publish(this->local_raw_pos_cmd);
        
    }
    void px4_node:: processCommandHover()
    {
        if (this->command_pre != this->pack_recv_command.messageId){
            ROS_INFO("Hover!");
            this->local_raw_pos_cmd.position.x = _current_pose.pose.position.x;
            this->local_raw_pos_cmd.position.y = _current_pose.pose.position.y;
            this->local_raw_pos_cmd.position.z = _current_pose.pose.position.z;
            // this->flag_trigger = true;
        }
        this->local_pub.publish(this->local_raw_pos_cmd);
    }
    void px4_node:: processCommandLand()
    {
        if (this->command_pre != this->pack_recv_command.messageId){
            ROS_INFO("Land!");
            this->local_raw_pos_cmd.position.x = _current_pose.pose.position.x;
            this->local_raw_pos_cmd.position.y = _current_pose.pose.position.y;
            this->local_raw_pos_cmd.position.z = 0;
            // this->flag_trigger = true;
        }
        this->local_pub.publish(this->local_raw_pos_cmd);
        
    }
    void px4_node:: processCommandPosition()
    {
        this->local_raw_pos_cmd.position.x = this->pack_recv_command.msgData[0];
        this->local_raw_pos_cmd.position.y = this->pack_recv_command.msgData[1];
        this->local_raw_pos_cmd.position.z = this->pack_recv_command.msgData[2];
        this->local_raw_pos_cmd.yaw = this->pack_recv_command.msgData[3];
        this->local_pub.publish(this->local_raw_pos_cmd);
                
    }
    void px4_node:: processCommandVelocityHori()
    {
         this->local_raw_vel_cmd.velocity.x = this->pack_recv_command.msgData[0];
        this->local_raw_vel_cmd.velocity.y = this->pack_recv_command.msgData[1];
        // this->local_raw_vel_cmd.velocity.z = this->pack_recv_command.msgData[2];
        this->local_raw_vel_cmd.position.z = this->pack_recv_command.msgData[2];
        this->local_raw_vel_cmd.yaw_rate = this->pack_recv_command.msgData[3];
        this->local_pub.publish(this->local_raw_vel_cmd);
        ROS_INFO("%f,%f,%f,%f",this->pack_recv_command.msgData[0],this->pack_recv_command.msgData[1],this->pack_recv_command.msgData[2],this->pack_recv_command.msgData[3]);
                
    }
    void px4_node:: processCommandAcceleration()
    {
        this->local_raw_acc_cmd.acceleration_or_force.x = this->pack_recv_command.msgData[0];
        this->local_raw_acc_cmd.acceleration_or_force.y = this->pack_recv_command.msgData[1];
        this->local_raw_acc_cmd.acceleration_or_force.z = this->pack_recv_command.msgData[2];
        this->local_raw_acc_cmd.yaw_rate = this->pack_recv_command.msgData[3];
        this->local_pub.publish(this->local_raw_acc_cmd);
                
    }
    void px4_node:: processCommandAttitude()
    {
        geometry_msgs::Quaternion q;
        float yaw_rate = this->pack_recv_command.msgData[0];
        float roll = this->pack_recv_command.msgData[1];
        float pitch = this->pack_recv_command.msgData[2];
        float thrust = this->pack_recv_command.msgData[3];
        q = tf::createQuaternionMsgFromRollPitchYaw(roll,pitch,0);
        // this->attitude_raw_cmd.
        this->attitude_raw_cmd.thrust = this->throttle_hover + thrust*thrust_to_throttle_gain;
        this->attitude_raw_cmd.body_rate.z = yaw_rate;
        this->attitude_raw_cmd.orientation = q;
        this->attitude_pub.publish(this->attitude_raw_cmd);
        
    }
    void px4_node:: processCommandDefault()
    {
        this->processCommandLand();
    }


    // Process packSend using data received from robot and prepared to send to simulink
    int px4_node:: generateStatePackage(uint8_t componentId,uint8_t messageId)
    {
        switch (componentId)
        {
        case msg::COMPONENT_TYPE::STATE_TYPE:
            switch (messageId)
            {
            case msg::MESSAGE_TYPE_STATE_TYPE::KINEMATIC_STATE_TYPE:
                this->generateKinematicStatePackage();
                break;
            case msg::MESSAGE_TYPE_STATE_TYPE::BATTERY_STATE_TYPE:
                this->generateBetteryPackage();
                break;
            case msg::MESSAGE_TYPE_STATE_TYPE::SIMPLE_STATE_TYPE:
                this->generateSimpleStatePackage();
            case msg::MESSAGE_TYPE_STATE_TYPE::STATE_12_TYPE:
                generateState12Package();
            default:
                break;
            }
            break;
        }
		
		return 1;
    }
    void px4_node:: generateBetteryPackage()
    {
        this->pack_send_bat.start       = 0xFA;
        this->pack_send_bat.system_id = this->systemId;
        this->pack_send_bat.component_id = msg::COMPONENT_TYPE::STATE_TYPE;
        this->pack_send_bat.message_id = msg::MESSAGE_TYPE_STATE_TYPE::BATTERY_STATE_TYPE;
        this->pack_send_bat.num_sequence++;
        int num = msg::getMsgNum(this->pack_send_bat.component_id,this->pack_send_bat.message_id);
        if (this->pack_send_bat.msg_data.size() != num)
        {
            this->pack_send_bat.msg_data.resize(num);
        }

        this->pack_send_bat.msg_data[0] = _current_bat.voltage;
    }

	void px4_node::generateKinematicStatePackage()
    {
        // this->pack_send_sstate.start       = 0xFA;
        // this->pack_send_sstate.system_id = this->systemId;
        // this->pack_send_sstate.component_id = msg::COMPONENT_TYPE::STATE_TYPE;
        // this->pack_send_sstate.message_id = msg::MESSAGE_TYPE_STATE_TYPE::KINEMATIC_STATE_TYPE;
        // this->pack_send_sstate.num_sequence++;
        // int num = msg::getMsgNum(this->pack_send_sstate.component_id,this->pack_send_sstate.message_id);
        // if (this->pack_send_sstate.msg_data.size() != num)
        // {
        //     this->pack_send_sstate.msg_data.resize(num);
        // }
        // this->pack_send_sstate.msg_data[0] = _current_pose.pose.position.x + this->x0;
        // this->pack_send_sstate.msg_data[1] = _current_pose.pose.position.y + this->y0;
        // this->pack_send_sstate.msg_data[2] = _current_pose.pose.position.z + this->z0;

        // this->pack_send_sstate.msg_data[3] = _current_vel.twist.linear.x;
        // this->pack_send_sstate.msg_data[4] = _current_vel.twist.linear.y;
        // this->pack_send_sstate.msg_data[5] = _current_vel.twist.linear.z;

        // this->pack_send_sstate.msg_data[6] = _current_imu.linear_acceleration.x;
        // this->pack_send_sstate.msg_data[7] = _current_imu.linear_acceleration.y;
        // this->pack_send_sstate.msg_data[8] = _current_imu.linear_acceleration.z;

        // this->pack_send_sstate.msg_data[9] = _current_pose.pose.orientation.w;
        // this->pack_send_sstate.msg_data[10] = _current_pose.pose.orientation.x;
        // this->pack_send_sstate.msg_data[11] = _current_pose.pose.orientation.y;
        // this->pack_send_sstate.msg_data[12] = _current_pose.pose.orientation.z;

        // this->pack_send_sstate.msg_data[13] = _current_vel.twist.angular.x;
        // this->pack_send_sstate.msg_data[14] = _current_vel.twist.angular.y;
        // this->pack_send_sstate.msg_data[15] = _current_vel.twist.angular.z;

    }

	void px4_node::generateSimpleStatePackage()
    {
        this->pack_send_sstate.start       = 0xFA;
        this->pack_send_sstate.system_id = this->systemId;
        this->pack_send_sstate.component_id = msg::COMPONENT_TYPE::STATE_TYPE;
        this->pack_send_sstate.message_id = msg::MESSAGE_TYPE_STATE_TYPE::SIMPLE_STATE_TYPE;
        this->pack_send_sstate.num_sequence++;
        int num = msg::getMsgNum(this->pack_send_sstate.component_id,this->pack_send_sstate.message_id);
        if (this->pack_send_sstate.msg_data.size() != num)
        {
            this->pack_send_sstate.msg_data.resize(num);
        }
        this->pack_send_sstate.msg_data[0] = _current_pose.pose.position.x + this->x0;
        this->pack_send_sstate.msg_data[1] = _current_pose.pose.position.y + this->y0;
        this->pack_send_sstate.msg_data[2] = _current_pose.pose.position.z + this->z0;
        this->pack_send_sstate.msg_data[3] = _current_vel.twist.linear.x;
        this->pack_send_sstate.msg_data[4] = _current_vel.twist.linear.y;
        this->pack_send_sstate.msg_data[5] = _current_vel.twist.linear.z;

        float qw=0,qx=0,qy=0,qz=0;
        qw = _current_pose.pose.orientation.w;
        qx = _current_pose.pose.orientation.x;
        qy = _current_pose.pose.orientation.y;
        qz = _current_pose.pose.orientation.z;
        this->pack_send_sstate.msg_data[6] = atan2(-2.0*qx*qy + 2.0*qw*qz, -2.0*qx*qx - 2.0*qz*qz + 1);
        this->pack_send_sstate.msg_data[7] = asin(2.0*qw*qx + 2.0*qy*qz);
        this->pack_send_sstate.msg_data[8] = atan2(-2.0*qx*qz + 2.0*qw*qy, -2.0*qx*qx - 2.0*qy*qy + 1);
        getMillisecond(&this->pack_send_sstate.time_stamp);
        this->pack_send_sstate.len_payload  = num*4;
        this->pack_send_sstate.check_num = 111;
    }

    void px4_node::generateState12Package()
    {
        this->pack_send_sstate.start       = 0xFA;
        this->pack_send_sstate.system_id = this->systemId;
        this->pack_send_sstate.component_id = msg::COMPONENT_TYPE::STATE_TYPE;
        this->pack_send_sstate.message_id = msg::MESSAGE_TYPE_STATE_TYPE::STATE_12_TYPE;
        this->pack_send_sstate.num_sequence++;
        int num = msg::getMsgNum(this->pack_send_sstate.component_id,this->pack_send_sstate.message_id);
        if (this->pack_send_sstate.msg_data.size() != num)
        {
            this->pack_send_sstate.msg_data.resize(num);
        }
        this->pack_send_sstate.msg_data[0] = _current_pose.pose.position.x + this->x0;
        this->pack_send_sstate.msg_data[1] = _current_pose.pose.position.y + this->y0;
        this->pack_send_sstate.msg_data[2] = _current_pose.pose.position.z + this->z0;
        this->pack_send_sstate.msg_data[3] = _current_vel.twist.linear.x;
        this->pack_send_sstate.msg_data[4] = _current_vel.twist.linear.y;
        this->pack_send_sstate.msg_data[5] = _current_vel.twist.linear.z;
        this->pack_send_sstate.msg_data[6] = _current_imu.linear_acceleration.x;
        this->pack_send_sstate.msg_data[7] = _current_imu.linear_acceleration.y;
        this->pack_send_sstate.msg_data[8] = _current_imu.linear_acceleration.z;

        float qw=0,qx=0,qy=0,qz=0;
        qw = _current_pose.pose.orientation.w;
        qx = _current_pose.pose.orientation.x;
        qy = _current_pose.pose.orientation.y;
        qz = _current_pose.pose.orientation.z;
        this->pack_send_sstate.msg_data[9] = atan2(-2.0*qx*qy + 2.0*qw*qz, -2.0*qx*qx - 2.0*qz*qz + 1);
        this->pack_send_sstate.msg_data[10] = asin(2.0*qw*qx + 2.0*qy*qz);
        this->pack_send_sstate.msg_data[11] = atan2(-2.0*qx*qz + 2.0*qw*qy, -2.0*qx*qx - 2.0*qy*qy + 1);
        getMillisecond(&this->pack_send_sstate.time_stamp);
        this->pack_send_sstate.len_payload  = num*4;
        this->pack_send_sstate.check_num = 111;
    }


    void px4_node::pack_cmd_cb(const udp_common::package_msg::ConstPtr& msg)
    {
        pack_recv_command.start       = msg->start;
        pack_recv_command.numSequence = msg->num_sequence;
        pack_recv_command.systemId    = msg->system_id;
        pack_recv_command.componentId = msg->component_id;
        pack_recv_command.messageId   = msg->message_id;
        int num = msg::getMsgNum(pack_recv_command.componentId,pack_recv_command.messageId);
        for (int i = 0; i < num; i++)
        {
            pack_recv_command.msgData[i] = msg->msg_data[i];
        }
        pack_recv_command.timeStamp = msg->time_stamp;
        pack_recv_command.lenPayload  = num*4;
        pack_recv_command.checkNum = msg->check_num;

        this->processCommand();
    }

    // void px4_node::pack_ssta_cb()
    // {
    //     this->generateStatePackage(msg::COMPONENT_TYPE::STATE_TYPE, msg::MESSAGE_TYPE_STATE_TYPE::SIMPLE_STATE_TYPE);
    //     this->pack_sta_pub.publish(this->pack_send_sstate);
    // }
    void px4_node::pack_sta12_cb()
    {
        this->generateStatePackage(msg::COMPONENT_TYPE::STATE_TYPE, msg::MESSAGE_TYPE_STATE_TYPE::STATE_12_TYPE);
        this->pack_sta_pub.publish(this->pack_send_sstate);
    }
    void px4_node::pack_bat_cb()
    {
        this->generateStatePackage(msg::COMPONENT_TYPE::STATE_TYPE, msg::MESSAGE_TYPE_STATE_TYPE::BATTERY_STATE_TYPE);
        this->pack_bat_pub.publish(this->pack_send_bat);
    }
    
}

bool getAndCheckParams(int argc,char **argv,int &id,
float &x0,float &y0,float &z0,float &roll0,float &pitch0,float &yaw0,float &throttle_hover);

int main(int argc, char **argv)
{
    ros::init(argc,argv,"px4_node");
    ros::NodeHandle n;
    //
    drone::px4_node my_px4_node;
    int id = 0;
    float x0 = 0.0f,y0 = 0.0f,z0 = 0.0f,roll0 = 0.0f,pitch0 = 0.0f,yaw0 = 0.0f, throttle_hover = 0.6106f;
    if (!getAndCheckParams(argc,argv,id,x0,y0,z0,roll0,pitch0,yaw0,throttle_hover))
    {
        return -1;
    }
    else
    {
        ROS_INFO("VehicleID:%d,\tinit pos:%f,%f,%f,%f,%f,%f.\thover throttle:%f.",
        id,x0,y0,z0,roll0,pitch0,yaw0,throttle_hover);
        my_px4_node.init_px4_node(id,x0,y0,z0,roll0,pitch0,yaw0,throttle_hover);
    }
    

    // ros::Timer timer_1 = n.createTimer(ros::Duration(0.01),[&](const ros::TimerEvent&){
    //     my_px4_node.pack_ssta_cb();
    // });
    // ros::Timer timer_2 = n.createTimer(ros::Duration(1),[&](const ros::TimerEvent&){
    //     my_px4_node.pack_bat_cb();
    // });
    
    // ros::spin();
    int count = 1;
    ros::Rate rate(100);

    while (ros::ok())
    {
        // my_px4_node.pack_ssta_cb();
        my_px4_node.pack_sta12_cb();
        if (count%100 == 0)
        {
            my_px4_node.pack_bat_cb();        
        }

        ros::spinOnce();
        rate.sleep();
        count++;
    }
    
    return 0;
}

bool getAndCheckParams(int argc,char **argv,int &id,
float &x0,float &y0,float &z0,float &roll0,float &pitch0,float &yaw0,float &throttle_hover)
{
    // if (argc != 13)
    // {
    //     ROS_ERROR("The number of input arguments must be 12!(now %d)",argc);
    //     return false;
    // }
    id = atoi(argv[1]);
    x0 = atof(argv[2]);
    y0 = atof(argv[3]);
    z0 = atof(argv[4]);
    roll0 = atof(argv[5]);
    pitch0 = atof(argv[6]);
    yaw0 = atof(argv[7]);
    throttle_hover = atof(argv[8]);
    if ( (id < 0) || (id > 255) )
    {ROS_ERROR("The id must be from 0 to 255!");return false;}
    
    return true;
}