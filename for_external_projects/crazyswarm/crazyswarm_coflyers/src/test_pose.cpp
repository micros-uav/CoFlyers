#include "ros/ros.h" 
#include"geometry_msgs/PoseStamped.h"
#include"geometry_msgs/Point.h"
//
int main(int argc, char **argv)
{
    ros::init(argc,argv,"test_node");
    ros::NodeHandle n;

    ros::Publisher pub = n.advertise<geometry_msgs::PoseStamped>("/cf4/pose",1);
    geometry_msgs::PoseStamped pos, pos_noise;
    geometry_msgs::Point vel, acc;
    vel.x = 2.0;
    vel.y = 2.0;
    vel.z = 2.0;
    acc.x = 0.0;
    acc.y = 0.0;
    acc.z = 0.0;
    ros::Rate rate(100);
    double dt = 0.01f;
    while (ros::ok())
    {
        pos.pose.position.x = pos.pose.position.x + vel.x*dt;
        pos.pose.position.y = pos.pose.position.y + vel.y*dt;
        pos.pose.position.z = pos.pose.position.z + vel.z*dt;

        vel.x = vel.x + acc.x*dt;
        vel.y = vel.y + acc.y*dt;
        vel.z = vel.z + acc.z*dt;
        pos.header.stamp = ros::Time::now();
        pos.pose.orientation.x = vel.x;
        pos.pose.orientation.y = vel.y;
        pos.pose.orientation.z = vel.z;
        pos_noise = pos;
        double x_rand,y_rand,z_rand;
        x_rand = (((double)rand()/(double)RAND_MAX - 0.5) * 0.01);
        y_rand = (((double)rand()/(double)RAND_MAX - 0.5) * 0.01);
        z_rand = (((double)rand()/(double)RAND_MAX - 0.5) * 0.01);
        // x_rand = 0;
        // y_rand = 0;
        // z_rand = 0;
        // ROS_INFO("%lf,%lf,%lf",x_rand,y_rand,z_rand);
        pos_noise.pose.position.x += x_rand;
        pos_noise.pose.position.y += y_rand;
        pos_noise.pose.position.z += z_rand;
        pub.publish(pos_noise);
        ros::spinOnce();
        rate.sleep();
    }
    

    return 0;
}