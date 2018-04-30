#include "ros/ros.h"
#include "icp_node.h"
#include <Eigen/Core>

#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>

void ICP_Align::publish_pos_msg(
  double tX, double tY, double tZ, double qX, double qY, double qZ, double qW
  )
{
  pos.header.stamp = ros::Time::now();
  pos.header.frame_id = "map";

  pos.pose.position.x = tX;
  pos.pose.position.y = tY;
  pos.pose.position.z = tZ;

  pos.pose.orientation.x = qX;
  pos.pose.orientation.y = qY;
  pos.pose.orientation.z = qZ;
  pos.pose.orientation.w = qW;

  pos_pub.publish(pos);
}

void ICP_Align::cloud_callback(const sensor_msgs::PointCloud2::ConstPtr& cloud_in)
{
  points_count += cloud_in->width;
  pcl::PointCloud<pcl::PointXYZ>::ConstPtr p_cloud_raw(cloud_raw.makeShared());
  pcl::PointCloud<pcl::PointXYZ>::ConstPtr p_cloud_ref(cloud_ref.makeShared());

  //Pre-transform, cancel the wheel sliding estimated by ICP
  for(size_t i = 0; i < cloud_in->width; ++i)
  {
    // Apply the transform to the current point
    float *pstep = (float*)&(cloud_in->data[i * cloud_in->point_step + 0]);

    // Copy transformed point into cloud
    pstep[0] += pos_error.x ();
    pstep[1] += pos_error.y ();
    pstep[2] += pos_error.z ();
  }

  pcl::fromROSMsg (*cloud_in , cloud_raw);

  if(points_count > init_count)
  {
    if(!_inited)
    {
      publish_pos_reset_msg();
      _inited = true;
    }

    // Align
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> reg;

    // Set the maximum distance between two correspondences (src<->tgt) to 10cm
    // Note: adjust this based on the size of your datasets
    pcl::PointCloud<pcl::PointXYZ> cloud_aligned; //Required for align() function, we may just discard this

    reg.setTransformationEpsilon (1e-4);
    reg.setInputSource (p_cloud_ref);
    reg.setInputTarget (p_cloud_raw);
    reg.setMaxCorrespondenceDistance (0.5);
    //reg.setEuclideanFitnessEpsilon (0.1);
    reg.setRANSACOutlierRejectionThreshold (0.005);

    reg.setMaximumIterations (10);
    reg.align (cloud_aligned);
    Eigen::Matrix4f T; //The transformation matrix

    if (reg.hasConverged ())
    {
      //std::cout << "ICP has converged\r\n"<< std::endl;
      T = reg.getFinalTransformation ();
      //Apply filter to the transform matrix

      double dX = -static_cast<double>(T(0,3)),
             dY = -static_cast<double>(T(1,3)),
             dZ = -static_cast<double>(T(2,3)),
             dYaw = atan2(static_cast<double>(T(1,0)), static_cast<double>(T(0,0)));

      dX = filterX.apply(dX);
      dY = filterY.apply(dY);
      dZ = filterZ.apply(dZ);
      dYaw = filterYaw.apply(dYaw);

      //Filtered transformation from ICP output
      tf2::Matrix3x3 tf3d;
      tf3d.setValue(cos(dYaw),  sin(dYaw),   0.0,
                   -sin(dYaw),  cos(dYaw),   0.0,
                          0.0,        0.0,   1.0);

      tf2::Quaternion tfqt;
      tf3d.getRotation(tfqt);

      tf2::Vector3 new_error;
      new_error.setValue(dX, dY, dZ);

      pos_error += new_error * 0.25;

      tf2::Transform transform;
      transform.setOrigin(new_error);
      transform.setRotation(tfqt);

      std::cout<<"TF_origin:"<<dX<<","<<dY<<","<<dZ<<","<<-dYaw*180/M_PI<<std::endl;
      //we want to loop through all the points in the cloud
      for(size_t i = 0; i < cloud_in->width; ++i)
      {
        // Apply the transform to the current point
        float *pstep = (float*)&(cloud_in->data[i * cloud_in->point_step + 0]);

        tf2::Vector3 point_in (pstep[0], pstep[1], pstep[2]);
        tf2::Vector3 point_out = transform * point_in;

        // Copy transformed point into cloud
        pstep[0] = point_out.x ();
        pstep[1] = point_out.y ();
        pstep[2] = point_out.z ();
      }

      publish_pos_msg(static_cast<double>(pos_error.getX()),
                      static_cast<double>(pos_error.getY()),
                      static_cast<double>(pos_error.getZ()),
                      static_cast<double>(tfqt.getX()),
                      static_cast<double>(tfqt.getY()),
                      static_cast<double>(tfqt.getZ()),
                      static_cast<double>(tfqt.getW()));
      cloud_pub.publish(cloud_in);//Acquired the correct transform
    }
    else
    {
      printf("ICP has not converged.\n");
    }
  }
  else //do nothing but to put these points to map,
       //we expect that the vehicle shall not move during initialization
  {
    //Sample the pcl input frequency in order to initialize filters
    static ros::Time timeStamp;
    static int msg_count = 0;
    if(!msg_count)
      timeStamp = ros::Time::now();
    else if(msg_count == 10)
    {
      double LPF_Sample_Freq = 10/((ros::Time::now() - timeStamp).toSec());
      std::cout<<"Raw PCL output freq: "<<LPF_Sample_Freq<<std::endl;

      //Initialize filters now
      filterX.set_cutoff_frequency(LPF_Sample_Freq, LPF_Cutoff_Freq);
      filterY.set_cutoff_frequency(LPF_Sample_Freq, LPF_Cutoff_Freq);
      filterZ.set_cutoff_frequency(LPF_Sample_Freq, LPF_Cutoff_Freq);
      filterYaw.set_cutoff_frequency(LPF_Sample_Freq, LPF_Cutoff_Freq);
    }
    msg_count++;

    cloud_pub.publish(cloud_in); //Do nothing to the cloud for now
  }
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
