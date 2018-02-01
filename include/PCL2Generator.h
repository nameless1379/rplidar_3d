/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef PCL2_GENERATOR_H
#define PCL2_GENERATOR_H

#include "ros/ros.h"

#include <map>
#include <iostream>
#include <sstream>

#include "boost/numeric/ublas/matrix.hpp"
#include "boost/thread/mutex.hpp"

#include <tf/tf.h>

#include <Eigen/Core>

#include "sensor_msgs/LaserScan.h"
#include <sensor_msgs/PointCloud2.h>
#include "geometry_msgs/QuaternionStamped.h"

#define POS_BUFFER_SIZE 200U

namespace laser_geometry
{
  // NOTE: invalid scan errors (will be present in LaserScan.msg in D-Turtle)
  const float LASER_SCAN_INVALID   = -1.0;
  const float LASER_SCAN_MIN_RANGE = -2.0;
  const float LASER_SCAN_MAX_RANGE = -3.0;

  namespace channel_option
  {
  //! Enumerated output channels options.
  /*!
   * An OR'd set of these options is passed as the final argument of
   * the projectLaser and transformLaserScanToPointCloud calls to
   * enable generation of the appropriate set of additional channels.
   */
    enum ChannelOption
      {
        None = 0x00,      //!< Enable no channels
        Intensity = 0x01, //!< Enable "intensities" channel
        Index     = 0x02, //!< Enable "index" channel
        Distance  = 0x04, //!< Enable "distances" channel
        Timestamp = 0x08, //!< Enable "stamps" channel
        Viewpoint = 0x10, //!< Enable "viewpoint" channel
        Default   = (Intensity | Index) //!< Enable "intensities" and "index" channels
      };
  }

  //! \brief A Class to Project Laser Scan
  /*!
   * This class will project laser scans into point clouds.  It caches
   * unit vectors between runs (provided the angular resolution of
   * your scanner is not changing) to avoid excess computation.
   *
   * By default all range values less than the scanner min_range, and
   * greater than the scanner max_range are removed from the generated
   * point cloud, as these are assumed to be invalid.
   *
   * If it is important to preserve a mapping between the index of
   * range values and points in the cloud, the recommended approach is
   * to pre-filter your laser_scan message to meet the requiremnt that all
   * ranges are between min and max_range.
   *
   * The generated PointClouds have a number of channels which can be enabled
   * through the use of ChannelOption.
   * - channel_option::Intensity - Create a channel named "intensities" with the intensity of the return for each point
   * - channel_option::Index - Create a channel named "index" containing the index from the original array for each point
   * - channel_option::Distance - Create a channel named "distances" containing the distance from the laser to each point
   * - channel_option::Timestamp - Create a channel named "stamps" containing the specific timestamp at which each point was measured
   */
  class LaserProjection
    {

    public:

      LaserProjection() :
        angle_min_(0), angle_max_(0), pos_buffer_num(-1), sync(0.0), points_in_cloud(0), total_points(0){}

      LaserProjection(ros::NodeHandle n, const int points, const double sync = 0.0) :
        angle_min_(0), angle_max_(0), pos_buffer_num(-1), sync(sync), points_in_cloud(points), total_points(0)
      {
          scan_sub = n.subscribe("/scan", 1000, &LaserProjection::scan_callBack, this);
          pos_sub = n.subscribe("/lidar_pos",1000,&LaserProjection::pos_callBack, this);
          cloud_pub = n.advertise<sensor_msgs::PointCloud2>("/PCL", 1000);
      }


      //! Destructor to deallocate stored unit vectors
      ~LaserProjection();

      //! Project a sensor_msgs::LaserScan into a sensor_msgs::PointCloud2
      /*!
       * Project a single laser scan from a linear array into a 3D
       * point cloud.  The generated cloud will be in the same frame
       * as the original laser scan.
       *
       * \param scan_in The input laser scan
       * \param cloud_out The output point cloud
       * \param range_cutoff An additional range cutoff which can be
       *   applied to discard everything above it.
       *   Defaults to -1.0, which means the laser scan max range.
       * \param channel_option An OR'd set of channels to include.
       *   Options include: channel_option::Default,
       *   channel_option::Intensity, channel_option::Index,
       *   channel_option::Distance, channel_option::Timestamp.
       */
      void scan_callBack(const sensor_msgs::LaserScan::ConstPtr& scan)
      {
        scan_callBack_(scan);
      }

      void pos_callBack(const geometry_msgs::QuaternionStamped::ConstPtr& pos)
      {
        pos_buffer_num++;
        if(pos_buffer_num == POS_BUFFER_SIZE)
          pos_buffer_num = 0;

        pos_buffer[pos_buffer_num].header = pos->header;
        pos_buffer[pos_buffer_num].quaternion = pos->quaternion;
      }

      void projectLaser (const sensor_msgs::LaserScan& scan_in,
                         sensor_msgs::PointCloud2 &cloud_out,
                         double range_cutoff = -1.0,
                         int channel_options = channel_option::Default)
      {
        projectLaser_(scan_in, cloud_out, range_cutoff, channel_options);
      }

      void transformLaserScanToPointCloud (const std::string &target_frame,
                                            const sensor_msgs::LaserScan &scan_in,
                                            sensor_msgs::PointCloud2 &cloud_out,
                                            tf2::Quaternion quat_start,
                                            tf2::Vector3 origin_start,
                                            tf2::Quaternion quat_end,
                                            tf2::Vector3 origin_end,
                                            double range_cutoff = -1.0,
                                            int channel_options = channel_option::Default);

    protected:

      //! Internal protected representation of getUnitVectors
      /*!
       * This function should not be used by external users, however,
       * it is left protected so that test code can evaluate it
       * appropriately.
       */
      const boost::numeric::ublas::matrix<double>& getUnitVectors_(double angle_min,
                                                                   double angle_max,
                                                                   double angle_increment,
                                                                   unsigned int length);
      void append_cloud(const sensor_msgs::PointCloud2& cloud_in);
      ros::Subscriber scan_sub;
      ros::Subscriber pos_sub;
      ros::Publisher cloud_pub;
    private:


      //! Internal hidden representation of projectLaser
      void projectLaser_ (const sensor_msgs::LaserScan& scan_in,
                          sensor_msgs::PointCloud2 &cloud_out,
                          double range_cutoff,
                          int channel_options);

      void scan_callBack_(const sensor_msgs::LaserScan::ConstPtr& scan);


      //! Internal map of pointers to stored values
      std::map<std::string,boost::numeric::ublas::matrix<double>* > unit_vector_map_;
      float angle_min_;
      float angle_max_;
      Eigen::ArrayXXd co_sine_map_;
      boost::mutex guv_mutex_;

      int pos_buffer_num;
      geometry_msgs::QuaternionStamped pos_buffer[POS_BUFFER_SIZE];

      sensor_msgs::PointCloud2 cloud; //Stores the point cloud data for two revolutions of stepper motor
      const double sync; //To better synchonize the on-board sensor data and LIDAR,
                  //we may want to slightly push the timeStamp of on_board data toward past
      const unsigned int points_in_cloud; //Number of LIDAR frames to store in a cloud
      unsigned long total_points;
    };
}

#endif //LASER_SCAN_UTILS_LASERSCAN_H
