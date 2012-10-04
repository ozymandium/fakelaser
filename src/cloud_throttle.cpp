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
#include "pcl_ros/transforms.h"
#include "pcl_ros/impl/transforms.hpp"
#include "tf/transform_listener.h"

namespace fakelaser
{
typedef pcl::PointXYZ PointType;
typedef pcl::PointCloud<PointType> PointCloud;

class CloudThrottle : public nodelet::Nodelet
{
public:
  //Constructor
  CloudThrottle(): max_update_rate_(0)
  {
      to_frame_="";
  };

private:
  tf::TransformListener tf_;
  std::string to_frame_;
  ros::Time last_update_;
  double max_update_rate_;
  virtual void onInit()
  {
    ros::NodeHandle& nh = getNodeHandle();
    ros::NodeHandle& private_nh = getPrivateNodeHandle();
    
    private_nh.getParam("max_rate", max_update_rate_);
    private_nh.getParam("frame_id", to_frame_);

    pub_ = nh.advertise<PointCloud>("cloud_out", 10);
    sub_ = nh.subscribe<PointCloud>("cloud_in", 10, &CloudThrottle::callback, this);
  };

  void callback(const PointCloud::ConstPtr& cloud)
  {
    if (max_update_rate_ > 0.0)
    {
      NODELET_DEBUG("update set to %f", max_update_rate_);
      if ( last_update_ + ros::Duration(1.0/max_update_rate_) > ros::Time::now())
      {
        NODELET_DEBUG("throttle last update at %f skipping", last_update_.toSec());
        return;
      }
    }
    else
      NODELET_DEBUG("update_rate unset continuing");
    last_update_ = ros::Time::now();

    if (to_frame_!="") {
      PointCloud output_cloud_;
      bool found_transform = tf_.waitForTransform(cloud->header.frame_id, to_frame_,cloud->header.stamp, ros::Duration(10.0));
      if (found_transform)
      {
        pcl_ros::transformPointCloud(to_frame_,*cloud, output_cloud_,tf_);
        pub_.publish (output_cloud_);
      }
      else {
        ROS_WARN("No transform found between %s and %s", cloud->header.frame_id.c_str(),to_frame_.c_str());
      }
    }
    else {
      pub_.publish(cloud);
    }
  }

  ros::Publisher pub_;
  ros::Subscriber sub_;
  
};


PLUGINLIB_DECLARE_CLASS(fakelaser, CloudThrottle, fakelaser::CloudThrottle, nodelet::Nodelet);
}
