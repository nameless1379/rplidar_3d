#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"

#define DEG2RAD(x) ((x)*M_PI/180.)

void publish_scan(ros::Publisher *pub,
                  size_t node_count,
                  double scan_time, bool inverted,
                  float angle_min, float angle_max,
                  std::string frame_id)
{
  sensor_msgs::LaserScan scan_msg;

  scan_msg.header.stamp = ros::Time::now();;
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


    ros::Rate r(200);
    while (ros::ok())
    {
        publish_scan(&scan_pub,
                        300,
                        0.138, false,
                        DEG2RAD(0.0f), DEG2RAD(359.0f),
                        "laser_frame");

        r.sleep();
        ros::spinOnce();
    }

    return 0;
}
