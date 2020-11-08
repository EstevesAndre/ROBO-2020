#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "controller");

    ros::NodeHandle n;

    ros::Publisher vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 100);

    ros::Rate loop_rate(10);

    while (ros::ok())
    {
        geometry_msgs::Twist msg;

        msg.linear.x = 1;

        vel_pub.publish(msg);

        ros::spinOnce();

        loop_rate.sleep();
    }

    return 0;
}