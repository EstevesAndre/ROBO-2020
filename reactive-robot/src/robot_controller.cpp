#include <ros/ros.h>

#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Int32.h>

#include <geometry_msgs/Twist.h>

ros::Publisher vel_pub;

void clbk_laser(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    geometry_msgs::Twist res_msg;
    res_msg.linear.x = 1;
    vel_pub.publish(res_msg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "robot_controller");

    ros::NodeHandle n;

    vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 100);
    ros::Subscriber sub_laser = n.subscribe("/scan", 1, clbk_laser);

    ros::spin();

    return 0;
}