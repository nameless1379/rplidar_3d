
#include "ros/ros.h"
#include "geometry_msgs/QuaternionStamped.h"
#include "geometry_msgs/Vector3Stamped.h"

#define RAD2DEG(x) ((x)*180./M_PI)

void imuCallback(const geometry_msgs::QuaternionStamped::ConstPtr& imu_msg)
{
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
