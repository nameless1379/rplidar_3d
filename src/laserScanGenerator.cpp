#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/QuaternionStamped.h"

#include <tf2/LinearMath/Transform.h>

#define DEG2RAD(x) ((x)*M_PI/180.)

void publish_pos_msg(ros::Publisher *pub, std::string frame_id)
{
    geometry_msgs::QuaternionStamped pos_msg;

    static float stepper_angle = 0.0f; //Sim stepper angle data

    stepper_angle += DEG2RAD(1.8);

    pos_msg.header.stamp = ros::Time::now();
    pos_msg.header.frame_id = frame_id;

    tf2::Quaternion q;
    q.setEulerZYX(stepper_angle,DEG2RAD(22.5),0);

    pos_msg.quaternion.x = (double)(q.x());
    pos_msg.quaternion.y = (double)(q.y());
    pos_msg.quaternion.z = (double)(q.z());
    pos_msg.quaternion.w = (double)(q.w());

    pub->publish(pos_msg);
}

void publish_scan(ros::Publisher *pub,
                  size_t node_count,
                  double scan_time, bool inverted,
                  float angle_min, float angle_max,
                  std::string frame_id)
{
  sensor_msgs::LaserScan scan_msg;

  ros::Duration duration(scan_time + 0.1);

  scan_msg.header.stamp = ros::Time::now() - duration;
  scan_msg.header.frame_id = frame_id;

  bool reversed = (angle_max > angle_min);
  if ( reversed ) {
    scan_msg.angle_min =  M_PI - angle_max;
    scan_msg.angle_max =  M_PI - angle_min;
  } else {
    scan_msg.angle_min =  M_PI - angle_min;
    scan_msg.angle_max =  M_PI - angle_max;
  }
  scan_msg.angle_increment =
      (scan_msg.angle_max - scan_msg.angle_min) / (double)(node_count-1);

  scan_msg.scan_time = scan_time;
  scan_msg.time_increment = scan_time / (double)(node_count-1);
  scan_msg.range_min = 0.15;
  scan_msg.range_max = 8.0;

  scan_msg.intensities.resize(node_count);
  scan_msg.ranges.resize(node_count);

  bool reverse_data = (!inverted && reversed) || (inverted && !reversed);
  if (!reverse_data) {
      for (size_t i = 0; i < node_count; i++) {
          float read_value = 3.0f;
          if (read_value == 0.0)
              scan_msg.ranges[i] = std::numeric_limits<float>::infinity();
          else
              scan_msg.ranges[i] = read_value;
          scan_msg.intensities[i] = 47;
      }
  } else {
      for (size_t i = 0; i < node_count; i++) {
          float read_value = 3.0f;
          if (read_value == 0.0)
              scan_msg.ranges[node_count-1-i] = std::numeric_limits<float>::infinity();
          else
              scan_msg.ranges[node_count-1-i] = read_value;
          scan_msg.intensities[node_count-1-i] = 47;
      }
  }

  pub->publish(scan_msg);
}

int main(int argc, char * argv[])
{
    ros::init(argc, argv, "laserscan_node");

    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");
    ros::Publisher scan_pub = nh.advertise<sensor_msgs::LaserScan>("/gen_scan", 1000);
    ros::Publisher pos_pub = nh.advertise<geometry_msgs::QuaternionStamped>("/gen_pos", 1000);


    ros::Rate r(100);
    unsigned int count = 0;

    while (ros::ok())
    {
        count++;
        if(count == 20)
        {
          count = 0;
          publish_scan(&scan_pub,
                          300,
                          0.2, false,
                          DEG2RAD(0.0f), DEG2RAD(359.0f),
                          "laser_frame");
        }
        publish_pos_msg(&pos_pub, "PCL2_frame");

        r.sleep();
        ros::spinOnce();
    }

    return 0;
}
