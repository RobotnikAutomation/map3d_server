/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
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
 *   * Neither the name of Willow Garage, Inc. nor the names of its
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
 * $Id: pcd_to_pointcloud.cpp 33238 2010-03-11 00:46:58Z rusu $
 * $Id: map3d_to_costmap.cpp 33238 2019-08-30 mbosch $
 *
 */

/**
\author Marc Bosch-Jorge
@b map3d_to_costmap is a simple node that serves a 3d map stored as a PCD (Point Cloud Data), derived from
pcl_ros/pcd_to_pointcloud.
\author Radu Bogdan Rusu
@b pcd_to_pointcloud is a simple node that loads PCD (Point Cloud Data) files from disk and publishes them as ROS
messages on the network.
 **/

// STL
#include <chrono>
#include <thread>

// ROS core
#include <ros/ros.h>

#include <nav_msgs/OccupancyGrid.h>

#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <string>
#include <sstream>

// helper function to return parsed parameter or default value
template <typename T>
T get_param(std::string const& name, T default_value)
{
  T value;
  ros::param::param<T>(name, value, default_value);
  return value;
}

typedef pcl::PointXYZ PointType;

class map3d_to_costmap
{
  ros::NodeHandle nh;
  // the topic to publish at, will be overwritten to give the remapped name
  std::string cloud_topic;
  // source file name, will be overwritten to produce actually configured file
  std::string file_name;
  // republish interval in seconds
  double interval;
  // tf2 frame_id
  std::string frame_id;
  // latched topic enabled/disabled
  bool latch;
  // pointcloud message and publisher
  sensor_msgs::PointCloud2 cloud;
  pcl::PointCloud<PointType> pointcloud;
  nav_msgs::OccupancyGrid grid;
  ros::Publisher cloud_pub, grid_pub;
  // timer to handle republishing
  ros::Timer timer;

  int threshold;
  double resolution;

  void publish()
  {
    ROS_DEBUG_STREAM_ONCE("Publishing pointcloud");
    ROS_DEBUG_STREAM_ONCE(" * number of points: " << cloud.width * cloud.height);
    ROS_DEBUG_STREAM_ONCE(" * frame_id: " << cloud.header.frame_id);
    ROS_DEBUG_STREAM_ONCE(" * topic_name: " << cloud_topic);
    int num_subscribers = cloud_pub.getNumSubscribers();
    if (num_subscribers > 0)
    {
      ROS_DEBUG("Publishing data to %d subscribers.", num_subscribers);
    }
    // update timestamp and publish
    cloud.header.stamp = ros::Time::now();
    cloud_pub.publish(cloud);
    grid_pub.publish(grid);
  }

  void timer_callback(ros::TimerEvent const&)
  {
    // just re-publish
    publish();
  }

public:
  map3d_to_costmap()
    : cloud_topic("mapcloud")
    , file_name("")
    , interval(0.0)
    , frame_id("map")
    , latch(false)
    , threshold(100)
    , resolution(0.1)
  {
    // update potentially remapped topic name for later logging
    cloud_topic = nh.resolveName(cloud_topic);
  }

  void parse_ros_params()
  {
    file_name = get_param("~file_name", file_name);
    interval = get_param("~interval", interval);
    frame_id = get_param("~frame_id", frame_id);
    latch = get_param("~latch", latch);
    threshold = get_param("~threshold", threshold);
    resolution = get_param("~resolution", resolution);
  }

  void parse_cmdline_args(int argc, char** argv)
  {
    if (argc > 1)
    {
      file_name = argv[1];
    }
    if (argc > 2)
    {
      std::stringstream str(argv[2]);
      double x;
      if (str >> x)
        interval = x;
    }
  }

  bool try_load_pointcloud()
  {
    if (file_name.empty())
    {
      ROS_ERROR_STREAM("Can't load pointcloud: no file name provided");
      return false;
    }
    else if (pcl::io::loadPCDFile(file_name, cloud) < 0)
    {
      ROS_ERROR_STREAM("Failed to parse pointcloud from file ('" << file_name << "')");
      return false;
    }
    pcl::io::loadPCDFile(file_name, pointcloud);
    project_costmap();
    // success: set frame_id appropriately
    cloud.header.frame_id = frame_id;
    return true;
  }

  bool project_costmap()
  {
    for (auto& point : pointcloud)
    {
      if (point.z > -1 and point.z < 4)
        point.z = 0;
      else
        point.z = 100;
    }

    pcl::PointXYZ min_point, max_point;
    pcl::getMinMax3D(pointcloud, min_point, max_point);

    min_point.x = -50;
    min_point.y = -15;

    max_point.x = 15;
    max_point.y = 3;

    pcl::io::savePCDFile("/home/mbosch/projected.pcd", pointcloud);

    pcl::PointCloud<PointType>::Ptr p(new pcl::PointCloud<PointType>());
    *p = pointcloud;
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(p);

    std::vector<int> indices;
    std::vector<float> distances;
    // double resolution = 0.5;
    // int threshold = 350;

    grid.info.resolution = resolution;
    grid.info.width = ((int)((max_point.x - min_point.x) / resolution)) + 1;
    grid.info.height = ((int)((max_point.y - min_point.y) / resolution)) + 1;
    grid.info.origin.position.x = min_point.x;
    grid.info.origin.position.y = min_point.y;
    size_t grid_size = grid.info.width * grid.info.height;
    grid.data = std::vector<int8_t>(grid_size, 0);

    int maxn = 0;
    int index = 0;
    for (double y = min_point.y; y <= max_point.y; y += resolution)
    {
      for (double x = min_point.x; x <= max_point.x; x += resolution)
      {
        PointType point(x, y, 0);
        int neighbours = kdtree.radiusSearch(point, resolution, indices, distances);
        if (neighbours > threshold)
          grid.data[index] = 100;
        else
          grid.data[index] = 0;
        // grid.data[index] = (neighbours < 100) ? neighbours : 100;
        index++;
        if (maxn < neighbours)
          maxn = neighbours;
      }
    }

    ROS_INFO("Index: %d, size: %d", index, grid.data.size());
    ROS_INFO("Max neighbours: %d", maxn);
    return true;
  }

  void init_run()
  {
    // init publisher
    cloud_pub = nh.advertise<sensor_msgs::PointCloud2>(cloud_topic, 1, latch);
    grid_pub = nh.advertise<nav_msgs::OccupancyGrid>("grid", 1, latch);
    // treat publishing once as a special case to interval publishing
    bool oneshot = interval <= 0;
    timer = nh.createTimer(ros::Duration(interval), &map3d_to_costmap::timer_callback, this, oneshot);
  }

  void print_config_info()
  {
    ROS_INFO_STREAM("Recognized the following parameters");
    ROS_INFO_STREAM(" * file_name: " << file_name);
    ROS_INFO_STREAM(" * interval: " << interval);
    ROS_INFO_STREAM(" * frame_id: " << frame_id);
    ROS_INFO_STREAM(" * topic_name: " << cloud_topic);
    ROS_INFO_STREAM(" * latch: " << std::boolalpha << latch);
  }

  void print_data_info()
  {
    ROS_INFO_STREAM("Loaded pointcloud with the following stats");
    ROS_INFO_STREAM(" * number of points: " << cloud.width * cloud.height);
    ROS_INFO_STREAM(" * total size [bytes]: " << cloud.data.size());
    ROS_INFO_STREAM(" * channel names: " << pcl::getFieldsList(cloud));
  }
};

int main(int argc, char** argv)
{
  // init ROS
  ros::init(argc, argv, "map3d_to_costmap");
  // set up node
  map3d_to_costmap node;
  // initializes from ROS parameters
  node.parse_ros_params();
  // also allow config to be provided via command line args
  // the latter take precedence
  node.parse_cmdline_args(argc, argv);
  // print info about effective configuration settings
  node.print_config_info();
  // try to load pointcloud from file
  if (!node.try_load_pointcloud())
  {
    return -1;
  }
  // print info about pointcloud
  node.print_data_info();
  // initialize run
  node.init_run();
  // blocking call to process callbacks etc
  ros::spin();
}
