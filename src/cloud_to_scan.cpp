/*                                                                                                                       
 * Copyright (c) 2010, Willow Garage, Inc.                                                                               
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

#include "ros/ros.h"
#include "pluginlib/class_list_macros.h"
#include "nodelet/nodelet.h"
#include "sensor_msgs/LaserScan.h"
#include "pcl/point_cloud.h"
#include "pcl_ros/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/ros/conversions.h"
#include "pcl_ros/impl/transforms.hpp"
#include "tf/transform_listener.h"

namespace fakelaser
{
typedef pcl::PointXYZ PointType;
typedef pcl::PointCloud<PointType> PointCloud;

class CloudToScan : public nodelet::Nodelet
{
public:
  //Constructor
  CloudToScan(): min_height_(0.10), max_height_(0.15), output_frame_id_("/camera_depth_frame"),to_frame_("")
  {
  };

private:
  virtual void onInit()
  {
    ros::NodeHandle& nh = getNodeHandle();
    ros::NodeHandle& private_nh = getPrivateNodeHandle();
    
    private_nh.getParam("min_height", min_height_);
    private_nh.getParam("max_height", max_height_);
    private_nh.getParam("frame_id", to_frame_);

    private_nh.getParam("output_frame_id", output_frame_id_);
    pub_ = nh.advertise<sensor_msgs::LaserScan>("fakescan", 10);
    sub_ = nh.subscribe<PointCloud>("cloud", 10, &CloudToScan::callback, this);
  };

  void callback(const PointCloud::ConstPtr& cloud)
  {
    sensor_msgs::LaserScanPtr output(new sensor_msgs::LaserScan());
    NODELET_DEBUG("Got cloud");
    //Copy Header
    output->header = cloud->header;
    output->header.frame_id = output_frame_id_;
    output->angle_min = -1.57079637051;
    output->angle_max = 1.56466042995;
    output->angle_increment = 0.006;//0.00613592332229;
    output->time_increment = 9.76562514552e-05;
    output->scan_time = 0.10000000149;
    output->range_min = 0.2;
    output->range_max = 5.59999990463;
    
    uint32_t ranges_size = std::ceil((output->angle_max - output->angle_min) / output->angle_increment);
    output->ranges.assign(ranges_size, output->range_max + 1.0);

    PointCloud output_cloud_;
    if (to_frame_!="") {
      bool found_transform = tf_.waitForTransform(cloud->header.frame_id, to_frame_,cloud->header.stamp, ros::Duration(10.0));
      if (found_transform)
      {
        pcl_ros::transformPointCloud(to_frame_,*cloud, output_cloud_,tf_);
      }
      else {
        ROS_WARN("No transform found between %s and %s", cloud->header.frame_id.c_str(),to_frame_.c_str());
      }
    }
    else {
        output_cloud_ = *cloud;
    }

    for (PointCloud::const_iterator it = output_cloud_.begin(); it != output_cloud_.end(); it++)
    {
      const float &side = it->y;
      const float &vert = it->z;
      const float &fwd = it->x;

      if ( std::isnan(fwd) || std::isnan(side) || std::isnan(vert) )
      {
        NODELET_DEBUG("rejected for nan in point(%f, %f, %f)\n", fwd, side, vert);
        continue;
      }
      if ((vert > max_height_) || (vert < min_height_))
      {
        NODELET_DEBUG("rejected for height %f not in range (%f, %f)\n", vert, min_height_, max_height_);
        continue;
      }
      double angle = atan2(side,fwd);
      if ((angle < output->angle_min) || (angle > output->angle_max))
      {
        NODELET_DEBUG("rejected for angle %f not in range (%f, %f)\n", angle, output->angle_min, output->angle_max);
        continue;
      }

      int index = (angle - output->angle_min) / output->angle_increment;
      double range_sq = fwd*fwd + side*side;
      if (output->ranges[index] * output->ranges[index] > range_sq)
        output->ranges[index] = sqrt(range_sq);
      //if(output->ranges[index])
    }
    output->header.stamp = ros::Time::now();
    pub_.publish(output);
  }

  double min_height_, max_height_;
  std::string output_frame_id_;
  tf::TransformListener tf_;
  std::string to_frame_;

  ros::Publisher pub_;
  ros::Subscriber sub_;
  
};


PLUGINLIB_DECLARE_CLASS(fakelaser, CloudToScan, fakelaser::CloudToScan, nodelet::Nodelet);
}
