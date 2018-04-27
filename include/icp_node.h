#ifndef _ICP_NODE_H_
#define _ICP_NODE_H_

class ICP_Align{
  public:

    ICP_Align() : point_total(0){}
    ICP_Align(ros::NodeHandle n) :
    point_total(0)
    {

    }

    ~ICP_Align();

    void cloud_callback(const sensor_msgs::PointCloud2::ConstPtr& cloud_in);

  private:
    unsigned long point_total;

    ros::Subscriber cloud_sub;
    ros::Publisher  cloud_pub;
};

#endif
