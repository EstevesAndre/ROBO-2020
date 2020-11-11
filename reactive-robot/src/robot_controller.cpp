#include <ros/ros.h>

#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>

#include <limits>
#include <cmath>
#include <vector>
#include <utility>


#define INF std::numeric_limits<float>::max()
#define PI (2*acos(0.0))

#define FORWARD_VELOCITY 0.3
#define ANGULAR_VELOCITY 3
#define ANG_VEL_FAR_AWAY 0.4
#define ANG_VEL_TOO_CLOSE 0.25
#define NUMBER_IMP_LASERS 100

ros::Publisher vel_pub;

float radian_to_degree(float radians) {
    return radians * 180.0 / PI;
}

float degree_to_radian(float degrees) {
    return degrees * PI / 180.0;
}

void clbk_laser(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    geometry_msgs::Twist res_msg;
    float min_range = INF;
    float min_angle = INF;
    std::vector<std::pair<float, float>> dist_ang;

    for(int i = 0; i < msg->ranges.size(); ++i)
    {
        float angle_rad = msg->angle_min + (i * msg->angle_increment);
        float angle = radian_to_degree(angle_rad);

        dist_ang.push_back(std::make_pair(msg->ranges[i],angle_rad));

        if((angle < 180 && msg->ranges[i] < min_range) || (angle >= 180 && msg->ranges[i] <= min_range))
        {
            min_range = msg->ranges[i];
            min_angle = angle;
        }
    }

    sort(dist_ang.begin(), dist_ang.end());

    float x = 0;
    float y = 0;

    for(int i = 0; i < NUMBER_IMP_LASERS; ++i)
    {
        float weight = 1 / pow(dist_ang[i].first,5);
        x += cos(dist_ang[i].second) * weight;
        y += sin(dist_ang[i].second) * weight;
    }   

    float mean = radian_to_degree(atan2(y,x));

    if(mean < 0)
        mean += 360;

    min_angle = mean;

    ROS_INFO("min_range: %f m, normal_angle: %f o", min_range, min_angle);
    res_msg.linear.x = FORWARD_VELOCITY;

    if(min_range <= 0.1205)
    {
        ROS_INFO("Perto");

        if(min_angle > 180)
        {
            float turn_angle = 1.5 * degree_to_radian(abs(min_angle - 360));
            res_msg.angular.z = turn_angle * ANG_VEL_TOO_CLOSE;
        }

        else
        {
            float turn_angle = 1.5 * degree_to_radian(min_angle);
            res_msg.angular.z = -turn_angle * ANG_VEL_TOO_CLOSE;
        }
    }

    else if(min_range >= 0.225)
    {
        ROS_INFO("Longe");

        if(min_angle > 180)
        {
            float turn_angle = 1.5 * degree_to_radian(abs(min_angle - 360));
            res_msg.angular.z = -turn_angle * ANG_VEL_FAR_AWAY;
        }

        else
        {
            float turn_angle = 1.5 * degree_to_radian(min_angle);
            res_msg.angular.z = turn_angle * ANG_VEL_FAR_AWAY;
        }
    }

    else if(abs(min_angle - 90) <= abs(min_angle - 270))
    {
        float turn_angle = 1.5 * degree_to_radian(abs(min_angle - 90));

        ROS_INFO("Parede Esquerda: Angle %f", turn_angle);

        if (min_angle > 90)
            res_msg.angular.z = ANGULAR_VELOCITY * turn_angle;

        else res_msg.angular.z = ANGULAR_VELOCITY * -turn_angle;
    }

    else
    {
        float turn_angle = 1.5 * degree_to_radian(abs(min_angle - 270));

        ROS_INFO("Parede Direita: Angle %f", turn_angle);

        if (min_angle > 270)
            res_msg.angular.z = ANGULAR_VELOCITY * turn_angle;

        else res_msg.angular.z = ANGULAR_VELOCITY * -turn_angle;
    }

    ROS_INFO("Vel lin: %f ang: %f", res_msg.linear.x, res_msg.angular.z);

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