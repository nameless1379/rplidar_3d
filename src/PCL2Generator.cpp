#include "PCL2Generator.h"
#include <algorithm>
#include <tf2/LinearMath/Transform.h>

#define DEG2RAD(x) ((x)*M_PI/180.)

namespace laser_geometry
{
  const boost::numeric::ublas::matrix<double>& LaserProjection::getUnitVectors_(double angle_min, double angle_max, double angle_increment, unsigned int length)
  {
    boost::mutex::scoped_lock guv_lock(this->guv_mutex_);

    //construct string for lookup in the map
    std::stringstream anglestring;
    anglestring <<angle_min<<","<<angle_max<<","<<angle_increment<<","<<length;
    std::map<std::string, boost::numeric::ublas::matrix<double>* >::iterator it;
    it = unit_vector_map_.find(anglestring.str());
    //check the map for presense
    if (it != unit_vector_map_.end())
      return *((*it).second);     //if present return

    boost::numeric::ublas::matrix<double> * tempPtr = new boost::numeric::ublas::matrix<double>(2,length);
    for (unsigned int index = 0;index < length; index++)
      {
        (*tempPtr)(0,index) = cos(angle_min + (double) index * angle_increment);
        (*tempPtr)(1,index) = sin(angle_min + (double) index * angle_increment);
      }
    //store
    unit_vector_map_[anglestring.str()] = tempPtr;
    //and return
    return *tempPtr;
  };

  LaserProjection::~LaserProjection()
  {
    std::map<std::string, boost::numeric::ublas::matrix<double>*>::iterator it;
    it = unit_vector_map_.begin();
    while (it != unit_vector_map_.end())
      {
        delete (*it).second;
        it++;
      }
  };

  void LaserProjection::projectLaser_ (const sensor_msgs::LaserScan& scan_in,
                                      sensor_msgs::PointCloud2 &cloud_out,
                                      double range_cutoff,
                                      int channel_options)
  {
    size_t n_pts = scan_in.ranges.size ();
    Eigen::ArrayXXd ranges (n_pts, 2);
    Eigen::ArrayXXd output (n_pts, 2);

    // Get the ranges into Eigen format
    for (size_t i = 0; i < n_pts; ++i)
    {
      ranges (i, 0) = (double) scan_in.ranges[i];
      ranges (i, 1) = (double) scan_in.ranges[i];
    }

    // Check if our existing co_sine_map is valid
    if (co_sine_map_.rows () != (int)n_pts || angle_min_ != scan_in.angle_min || angle_max_ != scan_in.angle_max )
    {
      ROS_DEBUG ("[projectLaser] No precomputed map given. Computing one.");
      co_sine_map_ = Eigen::ArrayXXd (n_pts, 2);
      angle_min_ = scan_in.angle_min;
      angle_max_ = scan_in.angle_max;
      // Spherical->Cartesian projection
      for (size_t i = 0; i < n_pts; ++i)
      {
        co_sine_map_ (i, 0) = cos (scan_in.angle_min + (double) i * scan_in.angle_increment);
        co_sine_map_ (i, 1) = sin (scan_in.angle_min + (double) i * scan_in.angle_increment);
      }
    }

    output = ranges * co_sine_map_;

    // Set the output cloud accordingly
    cloud_out.header = scan_in.header;
    cloud_out.height = 1;
    cloud_out.width  = scan_in.ranges.size ();
    cloud_out.fields.resize (3);
    cloud_out.fields[0].name = "x";
    cloud_out.fields[0].offset = 0;
    cloud_out.fields[0].datatype = sensor_msgs::PointField::FLOAT32;
    cloud_out.fields[0].count = 1;
    cloud_out.fields[1].name = "y";
    cloud_out.fields[1].offset = 4;
    cloud_out.fields[1].datatype = sensor_msgs::PointField::FLOAT32;
    cloud_out.fields[1].count = 1;
    cloud_out.fields[2].name = "z";
    cloud_out.fields[2].offset = 8;
    cloud_out.fields[2].datatype = sensor_msgs::PointField::FLOAT32;
    cloud_out.fields[2].count = 1;

    // Define 4 indices in the channel array for each possible value type
    int idx_intensity = -1, idx_index = -1, idx_distance = -1, idx_timestamp = -1, idx_vpx = -1, idx_vpy = -1, idx_vpz = -1;

    //now, we need to check what fields we need to store
    int offset = 12;
    if ((channel_options & channel_option::Intensity) && scan_in.intensities.size() > 0)
    {
      int field_size = cloud_out.fields.size();
      cloud_out.fields.resize(field_size + 1);
      cloud_out.fields[field_size].name = "intensity";
      cloud_out.fields[field_size].datatype = sensor_msgs::PointField::FLOAT32;
      cloud_out.fields[field_size].offset = offset;
      cloud_out.fields[field_size].count = 1;
      offset += 4;
      idx_intensity = field_size;
    }

    if ((channel_options & channel_option::Index))
    {
      int field_size = cloud_out.fields.size();
      cloud_out.fields.resize(field_size + 1);
      cloud_out.fields[field_size].name = "index";
      cloud_out.fields[field_size].datatype = sensor_msgs::PointField::INT32;
      cloud_out.fields[field_size].offset = offset;
      cloud_out.fields[field_size].count = 1;
      offset += 4;
      idx_index = field_size;
    }

    if ((channel_options & channel_option::Distance))
    {
      int field_size = cloud_out.fields.size();
      cloud_out.fields.resize(field_size + 1);
      cloud_out.fields[field_size].name = "distances";
      cloud_out.fields[field_size].datatype = sensor_msgs::PointField::FLOAT32;
      cloud_out.fields[field_size].offset = offset;
      cloud_out.fields[field_size].count = 1;
      offset += 4;
      idx_distance = field_size;
    }

    if ((channel_options & channel_option::Timestamp))
    {
      int field_size = cloud_out.fields.size();
      cloud_out.fields.resize(field_size + 1);
      cloud_out.fields[field_size].name = "stamps";
      cloud_out.fields[field_size].datatype = sensor_msgs::PointField::FLOAT32;
      cloud_out.fields[field_size].offset = offset;
      cloud_out.fields[field_size].count = 1;
      offset += 4;
      idx_timestamp = field_size;
    }

    if ((channel_options & channel_option::Viewpoint))
    {
      int field_size = cloud_out.fields.size();
      cloud_out.fields.resize(field_size + 3);

      cloud_out.fields[field_size].name = "vp_x";
      cloud_out.fields[field_size].datatype = sensor_msgs::PointField::FLOAT32;
      cloud_out.fields[field_size].offset = offset;
      cloud_out.fields[field_size].count = 1;
      offset += 4;

      cloud_out.fields[field_size + 1].name = "vp_y";
      cloud_out.fields[field_size + 1].datatype = sensor_msgs::PointField::FLOAT32;
      cloud_out.fields[field_size + 1].offset = offset;
      cloud_out.fields[field_size + 1].count = 1;
      offset += 4;

      cloud_out.fields[field_size + 2].name = "vp_z";
      cloud_out.fields[field_size + 2].datatype = sensor_msgs::PointField::FLOAT32;
      cloud_out.fields[field_size + 2].offset = offset;
      cloud_out.fields[field_size + 2].count = 1;
      offset += 4;

      idx_vpx = field_size;
      idx_vpy = field_size + 1;
      idx_vpz = field_size + 2;
    }

    cloud_out.point_step = offset;
    cloud_out.row_step   = cloud_out.point_step * cloud_out.width;
    cloud_out.data.resize (cloud_out.row_step   * cloud_out.height);
    cloud_out.is_dense = false;

    if (range_cutoff < 0)
      range_cutoff = scan_in.range_max;

    unsigned int count = 0;
    for (size_t i = 0; i < n_pts; ++i)
    {
      //check to see if we want to keep the point
      const float range = scan_in.ranges[i];
      if (range < range_cutoff && range >= scan_in.range_min)
      {
        float *pstep = (float*)&cloud_out.data[count * cloud_out.point_step];

        // Copy XYZ
        pstep[0] = output (i, 0);
        pstep[1] = output (i, 1);
        pstep[2] = 0;

        // Copy intensity
        if(idx_intensity != -1)
          pstep[idx_intensity] = scan_in.intensities[i];

        //Copy index
        if(idx_index != -1)
          ((int*)(pstep))[idx_index] = i;

        // Copy distance
        if(idx_distance != -1)
          pstep[idx_distance] = range;

        // Copy timestamp
        if(idx_timestamp != -1)
          pstep[idx_timestamp] = i * scan_in.time_increment;

        // Copy viewpoint (0, 0, 0)
        if(idx_vpx != -1 && idx_vpy != -1 && idx_vpz != -1)
        {
          pstep[idx_vpx] = 0;
          pstep[idx_vpy] = 0;
          pstep[idx_vpz] = 0;
        }

        //make sure to increment count
        ++count;
      }
    }

    //resize if necessary
    cloud_out.width = count;
    cloud_out.row_step   = cloud_out.point_step * cloud_out.width;
    cloud_out.data.resize (cloud_out.row_step   * cloud_out.height);
  }

  void LaserProjection::transformLaserScanToPointCloud(const std::string &target_frame,
                                                        const sensor_msgs::LaserScan &scan_in,
                                                        sensor_msgs::PointCloud2 &cloud_out,
                                                        tf2::Quaternion quat_start,
                                                        tf2::Vector3 origin_start,
                                                        tf2::Quaternion quat_end,
                                                        tf2::Vector3 origin_end,
                                                        double range_cutoff,
                                                        int channel_options)
  {
    //check if the user has requested the index field
    bool requested_index = false;
    if ((channel_options & channel_option::Index))
      requested_index = true;

    //we'll enforce that we get index values for the laser scan so that we
    //ensure that we use the correct timestamps
    channel_options |= channel_option::Index;

    projectLaser_(scan_in, cloud_out, range_cutoff, channel_options);

    //we'll assume no associated viewpoint by default
    bool has_viewpoint = false;
    uint32_t vp_x_offset = 0;

    //we need to find the offset of the intensity field in the point cloud
    //we also know that the index field is guaranteed to exist since we
    //set the channel option above. To be really safe, it might be worth
    //putting in a check at some point, but I'm just going to put in an
    //assert for now
    uint32_t index_offset = 0;
    for(unsigned int i = 0; i < cloud_out.fields.size(); ++i)
    {
      if(cloud_out.fields[i].name == "index")
      {
        index_offset = cloud_out.fields[i].offset;
      }

      //we want to check if the cloud has a viewpoint associated with it
      //checking vp_x should be sufficient since vp_x, vp_y, and vp_z all
      //get put in together
      if(cloud_out.fields[i].name == "vp_x")
      {
        has_viewpoint = true;
        vp_x_offset = cloud_out.fields[i].offset;
      }
    }

    ROS_ASSERT(index_offset > 0);

    cloud_out.header.frame_id = target_frame;

    tf2::Transform cur_transform ;

    double ranges_norm = 1 / ((double) scan_in.ranges.size () - 1.0);

    //we want to loop through all the points in the cloud
    for(size_t i = 0; i < cloud_out.width; ++i)
    {
      // Apply the transform to the current point
      float *pstep = (float*)&cloud_out.data[i * cloud_out.point_step + 0];

      //find the index of the point
      uint32_t pt_index;
      memcpy(&pt_index, &cloud_out.data[i * cloud_out.point_step + index_offset], sizeof(uint32_t));

      // Assume constant motion during the laser-scan, and use slerp to compute intermediate transforms
      tfScalar ratio = pt_index * ranges_norm;

      //! \todo Make a function that performs both the slerp and linear interpolation needed to interpolate a Full Transform (Quaternion + Vector)
      // Interpolate translation
      tf2::Vector3 v (0, 0, 0);
      v.setInterpolate3 (origin_start, origin_end, ratio);
      cur_transform.setOrigin (v);

      // Compute the slerp-ed rotation
      cur_transform.setRotation (slerp (quat_start, quat_end , ratio));

      tf2::Vector3 point_in (pstep[0], pstep[1], pstep[2]);
      tf2::Vector3 point_out = cur_transform * point_in;

      // Copy transformed point into cloud
      pstep[0] = point_out.x ();
      pstep[1] = point_out.y ();
      pstep[2] = point_out.z ();

      // Convert the viewpoint as well
      if(has_viewpoint)
      {
        float *vpstep = (float*)&cloud_out.data[i * cloud_out.point_step + vp_x_offset];
        point_in = tf2::Vector3 (vpstep[0], vpstep[1], vpstep[2]);
        point_out = cur_transform * point_in;

        // Copy transformed point into cloud
        vpstep[0] = point_out.x ();
        vpstep[1] = point_out.y ();
        vpstep[2] = point_out.z ();
      }
    }

    //if the user didn't request the index field, then we need to copy the PointCloud and drop it
    if(!requested_index)
    {
      sensor_msgs::PointCloud2 cloud_without_index;

      //copy basic meta data
      cloud_without_index.header = cloud_out.header;
      cloud_without_index.width = cloud_out.width;
      cloud_without_index.height = cloud_out.height;
      cloud_without_index.is_bigendian = cloud_out.is_bigendian;
      cloud_without_index.is_dense = cloud_out.is_dense;

      //copy the fields
      cloud_without_index.fields.resize(cloud_out.fields.size());
      unsigned int field_count = 0;
      unsigned int offset_shift = 0;
      for(unsigned int i = 0; i < cloud_out.fields.size(); ++i)
      {
        if(cloud_out.fields[i].name != "index")
        {
          cloud_without_index.fields[field_count] = cloud_out.fields[i];
          cloud_without_index.fields[field_count].offset -= offset_shift;
          ++field_count;
        }
        else
        {
          //once we hit the index, we'll set the shift
          offset_shift = 4;
        }
      }

      //resize the fields
      cloud_without_index.fields.resize(field_count);

      //compute the size of the new data
      cloud_without_index.point_step = cloud_out.point_step - offset_shift;
      cloud_without_index.row_step   = cloud_without_index.point_step * cloud_without_index.width;
      cloud_without_index.data.resize (cloud_without_index.row_step   * cloud_without_index.height);

      uint32_t i = 0;
      uint32_t j = 0;
      //copy over the data from one cloud to the other
      while (i < cloud_out.data.size())
      {
        if((i % cloud_out.point_step) < index_offset || (i % cloud_out.point_step) >= (index_offset + 4))
        {
          cloud_without_index.data[j++] = cloud_out.data[i];
        }
        i++;
      }

      //make sure to actually set the output
      cloud_out = cloud_without_index;
    }
  }

  void LaserProjection::scan_callBack_(const sensor_msgs::LaserScan::ConstPtr& scan)
  {
    sensor_msgs::PointCloud2 cloud_out;

    tf2::Quaternion q1,q2;
    q1.setEulerZYX(0,DEG2RAD(22.5),0);
    q2.setEulerZYX(DEG2RAD(45),DEG2RAD(22.5),0);
    transformLaserScanToPointCloud ("PCL2_frame",
                                    *scan, cloud_out,
                                    q1,tf2::Vector3(0, 0, 0),q2,tf2::Vector3(0, 0, 0));
    cloud_pub.publish(cloud_out);
  }
} //laser_geometry

int main(int argc, char * argv[])
{
    ros::init(argc, argv, "PCL_node");

    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    laser_geometry::LaserProjection Laser(nh);

    ros::spin();

    return 0;
}
