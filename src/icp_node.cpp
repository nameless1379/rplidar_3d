#include "ros/ros.h"
#include "icp_node.h"
#include <Eigen/Core>

#include "pcl_conversions/pcl_conversions.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"

void ICP_Align::publish_pos_msg(
  double translation[3], double rotation[4]
  )
{
  pos.header.stamp = ros::Time::now();
  pos.header.frame_id = "map";

  pos.pose.position.x = translation[0];
  pos.pose.position.y = translation[1];
  pos.pose.position.z = translation[2];

  pos.pose.orientation.x = rotation[0];
  pos.pose.orientation.y = rotation[1];
  pos.pose.orientation.z = rotation[2];
  pos.pose.orientation.w = rotation[3];

  pos_pub.publish(pos);
}

void ICP_Align::align()
{
  /*
  // Align
  pcl::IterativeClosestPointNonLinear<PointNormalT, PointNormalT> reg;
  reg.setTransformationEpsilon (1e-6);
  // Set the maximum distance between two correspondences (src<->tgt) to 10cm
  // Note: adjust this based on the size of your datasets
  reg.setMaxCorrespondenceDistance (0.1);
  // Set the point representation
  reg.setPointRepresentation (boost::make_shared<const MyPointRepresentation> (point_representation));

  reg.setInputSource (points_with_normals_src);
  reg.setInputTarget (points_with_normals_tgt);
  W
  //
  // Run the same optimization in a loop and visualize the results
  Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity (), prev, targetToSource;
  PointCloudWithNormals::Ptr reg_result = points_with_normals_src;
  reg.setMaximumIterations (4);
  for (int i = 0; i < 30; ++i)
  {

    // save cloud for visualization purpose
    points_with_normals_src = reg_result;

    // Estimate
    reg.setInputSource (points_with_normals_src);
    reg.align (*reg_result);

		//accumulate transformation between each Iteration
    Ti = reg.getFinalTransformation () * Ti;

		//if the difference between this transformation and the previous one
		//is smaller than the threshold, refine the process by reducing
		//the maximal correspondence distance
    if (fabs ((reg.getLastIncrementalTransformation () - prev).sum ()) < reg.getTransformationEpsilon ())
      reg.setMaxCorrespondenceDistance (reg.getMaxCorrespondenceDistance () - 0.001);

    prev = reg.getLastIncrementalTransformation ();
  }*/

}

void ICP_Align::cloud_callback(const sensor_msgs::PointCloud2::ConstPtr& cloud_in)
{
  points_count += cloud_in->width;
  if(points_count > init_count)
  {
    if(!_inited)
    {
      publish_pos_reset_msg();
      _inited = true;
    }
    cloud_pub.publish(cloud_in);//Do nothing for now
  }
  else //do nothing but to put these points to map,
       //we expect that the vehicle shall not move during initialization
    cloud_pub.publish(cloud_in);
}

int main(int argc, char * argv[])
{
    ros::init(argc, argv, "ICP_node");

    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    int init_count = 0;
    nh_private.param<int>("Init_count", init_count, 30000);

    ICP_Align icp(nh, init_count);

    ros::spin();

    return 0;
}
