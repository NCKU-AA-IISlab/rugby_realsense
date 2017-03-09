/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2017, (ROC) Advanced Robotics, Ltd.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of Advanced Robotics, Ltd. nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Author: HaoChih, LIN
* Email: f44006076@gmail.com
*********************************************************************/

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <iostream>
#include <nodelet/nodelet.h>
#include <pcl/point_cloud.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/filter.h>


namespace rugby_realsense 
{

	class PointCloudSetValid : public nodelet::Nodelet
	{                
	  ros::Subscriber pcd_sub;			     
	  ros::Publisher pcd_pub;                   
	  virtual void onInit();
	  void pcd_callback(const sensor_msgs::PointCloud2ConstPtr& points_in);
	};


	void PointCloudSetValid::onInit()
	{
	  ros::NodeHandle& nh_ = getNodeHandle();
	  pcd_sub = nh_.subscribe("/input_invalid", 1, &PointCloudSetValid::pcd_callback, this);
	  pcd_pub = nh_.advertise<sensor_msgs::PointCloud2>("/output_valid",10, true);
	}


	void PointCloudSetValid::pcd_callback(const sensor_msgs::PointCloud2ConstPtr& points_in)
	{
	  // Convert the sensor_msgs/PointCloud2 data to pcl::PointCloud
	  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZ>);
	  pcl::fromROSMsg (*points_in, *cloud_out);
	  
          //remove NAN points from the cloud
	  std::vector<int> indices;
	  pcl::removeNaNFromPointCloud(*cloud_out,*cloud_out, indices);

	  // Convert the pcl::PointCloud to sensor_msgs/PointCloud2
  	  sensor_msgs::PointCloud2::Ptr points_out(new sensor_msgs::PointCloud2);
  	  pcl::toROSMsg( *cloud_out, *points_out );
	  
	  // Publish the result
	  pcd_pub.publish(points_out);
	}

} //namespace rugby_realsense

// Register as nodelet
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rugby_realsense::PointCloudSetValid, nodelet::Nodelet);
