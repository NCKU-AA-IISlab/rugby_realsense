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
#include <iostream>
#include <nodelet/nodelet.h>
#include <sensor_msgs/PointCloud2.h>
#include <laser_geometry/laser_geometry.h>
#include <sensor_msgs/LaserScan.h>


namespace rugby_realsense 
{

	class LaserToPointCloud : public nodelet::Nodelet
	{                
	  ros::Subscriber scan_sub_;			     
	  ros::Publisher pcd_pub_;  
	  laser_geometry::LaserProjection projector_;                 
	  virtual void onInit();
	  void scan_callback(const sensor_msgs::LaserScanConstPtr& scan_in);
	};


	void LaserToPointCloud::onInit()
	{
	  ros::NodeHandle& nh_ = getNodeHandle();
	  scan_sub_ = nh_.subscribe("/scan", 1, &LaserToPointCloud::scan_callback, this);
	  pcd_pub_  = nh_.advertise<sensor_msgs::PointCloud2>("/points",10, true);
	}


	void LaserToPointCloud::scan_callback(const sensor_msgs::LaserScanConstPtr& scan_in)
	{
	  sensor_msgs::PointCloud2::Ptr cloud(new sensor_msgs::PointCloud2);
	  projector_.projectLaser(*scan_in, *cloud);
	  //Publish the cloud.
 	  pcd_pub_.publish(cloud);
	}

} //namespace rugby_realsense

// Register as nodelet
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rugby_realsense::LaserToPointCloud, nodelet::Nodelet);
