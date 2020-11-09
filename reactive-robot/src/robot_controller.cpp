#include <ros/ros.h>

#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>

#include <limits>
#include <cmath>

#define INF std::numeric_limits<float>::max()
#define PI 2*acos(0.0)

ros::Publisher vel_pub;

void clbk_laser(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    float min_range = INF;
    float min_angle = INF;


    for(int i = 0; i < msg->ranges.size(); ++i)
    {
        float angle = msg->angle_min + i * msg->angle_increment;

        if(msg->ranges[i] < min_range)
        {
            min_range = msg->ranges[i];
            min_angle = angle;
        }
    }

    ROS_INFO("min_range: %f, min_angle: %f", min_range, min_angle);

    geometry_msgs::Twist res_msg;
    res_msg.angular.z = min_angle - (PI / 2);
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