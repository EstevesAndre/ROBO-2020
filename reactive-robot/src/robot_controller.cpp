#include <ros/ros.h>

#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>

#include <limits>
#include <cmath>
#include <vector>
#include <utility>


#define INF std::numeric_limits<float>::max()
#define PI (2*acos(0.0))

#define LINEAR_VELOCITY 0.3
#define ANGULAR_VELOCITY 0.5

#define MIN_DIST_WALL 0.3

#define D_WALL 0.3
#define WALL_LEAD 0.3

ros::Publisher vel_pub;

float radian_to_degree(float radians) 
{
    return radians * 180.0 / PI;
}

float degree_to_radian(float degrees) 
{
    return degrees * PI / 180.0;
}

void clbk_laser(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    geometry_msgs::Twist res_msg;
    res_msg.linear.x = LINEAR_VELOCITY;

    float min_range = INF;
    float min_angle = INF;
    float sum_angle_away = 0;
    float angles_away = 0;

    for(int i = 0; i < msg->ranges.size(); ++i)
    {
        float angle_rad = msg->angle_min + (i * msg->angle_increment);
        float angle = radian_to_degree(angle_rad);

        if(msg->ranges[i] > 1)
        {
            sum_angle_away += angle;
            angles_away++;
        }

        if((angle < 180 && msg->ranges[i] < min_range) || (angle >= 180 && msg->ranges[i] <= min_range))
        {
            min_range = msg->ranges[i];
            min_angle = angle;
        }
    }

    ROS_INFO("MIN_DIST: %f", min_range);

    // if wall close in front -> stop and rotate to angle with no walls / walls far away
    if(msg->ranges[0] <= MIN_DIST_WALL)
    {
        res_msg.linear.x = 0;

        float angle_away = sum_angle_away / angles_away;

        if(angle_away < 180)
            res_msg.angular.z = 1;

        else
            res_msg.angular.z = -1;
        
        vel_pub.publish(res_msg);
        return;
    }

    // if robot gets too far from the wall -> move more agressively towards wall
    if(min_range >= 0.3)
    {
        if(min_angle > 180)
        {
            float turn_angle = degree_to_radian(abs(min_angle - 360));
            res_msg.angular.z = -turn_angle * ANGULAR_VELOCITY;
        }

        else
        {
            float turn_angle = degree_to_radian(min_angle);
            res_msg.angular.z = turn_angle * ANGULAR_VELOCITY;
        }

        vel_pub.publish(res_msg);
        return;
    }

    // normal movement - following wall on the left
    if(abs(min_angle - 90) <= abs(min_angle - 270))
    {
        float y0 = msg->ranges[90];
        float x1 = msg->ranges[45] * cos(45);
        float y1 = msg->ranges[45] * sin(45);

        res_msg.angular.z = atan2(y1 - D_WALL, x1 + WALL_LEAD - y0);
    }

    // normal movement - following wall on the right
    else
    {
        float y0 = msg->ranges[270];
        float x1 = msg->ranges[315] * cos(45);
        float y1 = msg->ranges[315] * sin(45);

        res_msg.angular.z = -atan2(y1 - D_WALL, x1 + WALL_LEAD - y0);
    }

    vel_pub.publish(res_msg);
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "robot_controller");

    ros::NodeHandle n;

    vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    ros::Subscriber sub_laser = n.subscribe("/scan", 1, clbk_laser);

    ros::spin();

    return 0;
}