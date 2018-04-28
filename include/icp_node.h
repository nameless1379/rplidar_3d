#ifndef _ICP_NODE_H_
#define _ICP_NODE_H_

#include "sensor_msgs/PointCloud2.h"
#include "geometry_msgs/PoseStamped.h"

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

      cloud_sub = n.subscribe(sub_input_topic, 200, &ICP_Align::cloud_callback, this);
      ref_sub = n.subscribe(sub_ref_topic, 200, &ICP_Align::ref_callback, this);
      pos_pub = n.advertise<geometry_msgs::PoseStamped>(pub_pos_topic, 200);
      cloud_pub = n.advertise<sensor_msgs::PointCloud2>(pub_cloud_topic, 200);
    }

    ~ICP_Align(){}

    void align();

    void cloud_callback(const sensor_msgs::PointCloud2::ConstPtr& cloud_in);
    void ref_callback(const sensor_msgs::PointCloud2::ConstPtr& cloud_in)
    {
      ref_pcl.header = cloud_in->header;
      ref_pcl.height = cloud_in->height;
      ref_pcl.fields = cloud_in->fields;
      ref_pcl.is_bigendian = cloud_in->is_bigendian;
      ref_pcl.point_step = cloud_in->point_step;
      ref_pcl.is_dense = cloud_in->is_dense;

      ref_pcl.width = cloud_in->width;
      ref_pcl.row_step = cloud_in->width * cloud_in->point_step;
      ref_pcl.data.resize(cloud_in->row_step * cloud_in->height);

      memcpy(&ref_pcl.data[0], &(cloud_in->data[0]), cloud_in->row_step * cloud_in->height);
    }

  private:
    unsigned long point_total;

    ros::Subscriber cloud_sub, ref_sub;
    ros::Publisher  cloud_pub, pos_pub;

    geometry_msgs::PoseStamped pos;
    sensor_msgs::PointCloud2 ref_pcl;

    void publish_pos_msg(
      double translation[3], double rotation[4]
    );

    void publish_pos_reset_msg()
    {
      double translation[3] = {0, 0, 0};
      double rotation[4] = {100.0, 100.0, 100.0, 100.0};

      publish_pos_msg(translation, rotation);
      //Transmitting an invalid quaternion of all 100.0 will trigger a imu wheel odeometry reset
    }

    bool _inited;
    const unsigned int init_count; //Num of points needed for initialization of ICP process
    unsigned long points_count; //Total num of points received
};

#endif
