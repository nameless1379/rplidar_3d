#ifndef _ICP_NODE_H_
#define _ICP_NODE_H_

#include "sensor_msgs/PointCloud2.h"
#include "geometry_msgs/PoseStamped.h"

#include "pcl_conversions/pcl_conversions.h"
#include "pcl_ros/point_cloud.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"

#include <tf2/LinearMath/Transform.h>
#include "math_utility.h"

class ICP_Align{
  public:

    ICP_Align() : _inited(false), point_total(0), init_count(0), points_count(0){}
    ICP_Align(ros::NodeHandle& n, const unsigned int init_count) :
      _inited(false), point_total(0), init_count(init_count), points_count(0)
    {
      std::string sub_input_topic, sub_ref_topic, pub_pos_topic, pub_cloud_topic;

      n.param<std::string>("PCL_in_topic",   sub_input_topic,  "/PCL");
      n.param<std::string>("PCL_ref_topic",  sub_ref_topic,    "/octomap_point_cloud_centers");
      n.param<std::string>("Corr_pos_topic", pub_pos_topic,    "/corr_pos");
      n.param<std::string>("PCL_out_topic",  pub_cloud_topic,  "/cloud_in");
      n.param<double>("LPF_Cutoff_Freq",  LPF_Cutoff_Freq,  0.5);

      cloud_sub = n.subscribe(sub_input_topic, 1000, &ICP_Align::cloud_callback, this);
      ref_sub = n.subscribe(sub_ref_topic, 20000, &ICP_Align::ref_callback, this);
      pos_pub = n.advertise<geometry_msgs::PoseStamped>(pub_pos_topic, 200);
      cloud_pub = n.advertise<pcl::PointCloud<pcl::PointXYZ> >(pub_cloud_topic, 20000);

      pos_error.setValue(0.0, 0.0, 0.0);
    }

    ~ICP_Align(){}

    void cloud_callback(const sensor_msgs::PointCloud2::ConstPtr& cloud_in);
    void ref_callback(const sensor_msgs::PointCloud2::ConstPtr& cloud_in)
    {
      pcl::fromROSMsg(*cloud_in, cloud_ref);
    }

  private:
    unsigned long point_total;

    math::LowPassFilter2p filterX, filterY, filterZ, filterYaw;
    double LPF_Cutoff_Freq;

    ros::Subscriber cloud_sub, ref_sub;
    ros::Publisher  cloud_pub, pos_pub;

    geometry_msgs::PoseStamped pos;
    pcl::PointCloud<pcl::PointXYZ> cloud_raw, cloud_ref;
    tf2::Vector3 pos_error; //Store the total pos error corrected by LIDAR icp

    void publish_pos_msg(
      double tX, double tY, double tZ, double qX, double qY, double qZ, double qW
    );

    void publish_pos_reset_msg()
    {
      publish_pos_msg(0.0, 0.0, 0.0, 100.0, 100.0, 100.0, 100.0);
      //Transmitting an invalid quaternion of all 100.0 will trigger a imu wheel odeometry reset
    }

    bool _inited;
    const unsigned int init_count; //Num of points needed for initialization of ICP process
    unsigned long points_count; //Total num of points received
};

#endif
