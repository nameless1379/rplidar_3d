
#include "ros/ros.h"
#include "geometry_msgs/QuaternionStamped.h"
#include "geometry_msgs/Vector3Stamped.h"

#define RAD2DEG(x) ((x)*180./M_PI)

void imuCallback(const geometry_msgs::QuaternionStamped::ConstPtr& imu_msg)
{
/*
    int count = scan->scan_time / scan->time_increment;
    ROS_INFO("I heard a laser scan %s[%d]:", scan->header.frame_id.c_str(), count);
    ROS_INFO("angle_range, %f, %f", RAD2DEG(scan->angle_min), RAD2DEG(scan->angle_max));

    for(int i = 0; i < count; i++) {
        float degree = RAD2DEG(scan->angle_min + scan->angle_increment * i);
        ROS_INFO(": [%f, %f]", degree, scan->ranges[i]);
    }
    */
    geometry_msgs::Vector3Stamped euler_angle;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "stm32_node_client");
    ros::NodeHandle n;

    ros::Subscriber imu_sub =
      n.subscribe<geometry_msgs::QuaternionStamped>("/imu_data", 1000, imuCallback);

    ros::Publisher euler_pub =
      n.advertise<geometry_msgs::Vector3Stamped>("euler_angle", 1000);

    ros::spin();

    return 0;
}
