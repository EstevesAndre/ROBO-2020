#include <ros/ros.h>

#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>

#include <limits>
#include <cmath>

#define INF std::numeric_limits<float>::max()
#define PI (2*acos(0.0))

#define FORWARD_VELOCITY 0.2
#define ANGULAR_VELOCITY 3

ros::Publisher vel_pub;

float radian_to_degree(float radians) {
    return radians * 180.0 / PI;
}

float degree_to_radian(float degrees) {
    return degrees * PI / 180.0;
}

void clbk_laser(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    float min_range = INF;
    float min_angle = INF;

    for(int i = 0; i < msg->ranges.size(); ++i)
    {
        float angle = radian_to_degree(msg->angle_min + (i * msg->angle_increment));

        if(msg->ranges[i] < min_range)
        {
            min_range = msg->ranges[i];
            min_angle = angle;
        }
    }

    ROS_INFO("min_range: %f m, min_angle: %f o", min_range, min_angle);

    geometry_msgs::Twist res_msg;

    if(min_range >= 0.2)
    {
        res_msg.linear.x = FORWARD_VELOCITY;

        if(min_angle > 180)
        {
            float turn_angle = degree_to_radian(abs(min_angle - 360));
            res_msg.angular.z = -turn_angle;
        }

        else
        {
            float turn_angle = degree_to_radian(min_angle);
            res_msg.angular.z = turn_angle;
        }
    }

    else if(abs(min_angle - 90) <= abs(min_angle - 270))
    {
        float turn_angle = degree_to_radian(abs(min_angle - 90));
        res_msg.linear.x = FORWARD_VELOCITY * cos(turn_angle);

        ROS_INFO("Parede Direita: Angle %f", turn_angle);

        if (min_angle > 90)
            res_msg.angular.z = ANGULAR_VELOCITY * turn_angle;

        else res_msg.angular.z = ANGULAR_VELOCITY * -turn_angle;
    }

    else
    {
        float turn_angle = degree_to_radian(abs(min_angle - 270));
        res_msg.linear.x = FORWARD_VELOCITY * cos(turn_angle);

        ROS_INFO("Parede Esquerda: Angle %f", turn_angle);

        if (min_angle > 270)
            res_msg.angular.z = ANGULAR_VELOCITY * turn_angle;

        else res_msg.angular.z = ANGULAR_VELOCITY * -turn_angle;
    }

    if (min_range <= 0.15 && (min_angle < 10 || min_angle > 350))
    {
        res_msg.linear.x = -FORWARD_VELOCITY;
    }

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