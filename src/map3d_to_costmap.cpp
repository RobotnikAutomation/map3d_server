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
#include <pcl/filters/voxel_grid.h>

#include <string>
#include <sstream>

#include <dynamic_reconfigure/server.h>
#include <map3d_server/Map3dToCostmapConfig.h>

// helper function to return parsed parameter or default value
template <typename T>
T get_param(std::string const& name, T default_value)
{
  T value;
  ros::param::param<T>(name, value, default_value);
  return value;
}

typedef pcl::PointXYZ PointType;

struct Point
{
  double x, y, z;
};

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

  // manage reconfigure parameters
  dynamic_reconfigure::Server<map3d_server::Map3dToCostmapConfig> reconfigure_server;

  // is running, to not trigger
  bool running;

  int occupancy_threshold;
  double resolution;
  Point bb_min, bb_max;

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
    ROS_INFO("Timer");
    // just re-publish
    publish();
  }

public:
  map3d_to_costmap()
    : cloud_topic("mapcloud")
    , file_name("")
    , interval(0.0)
    , frame_id("map")
    , latch(true)
    , occupancy_threshold(100)
    , resolution(0.1)
    , running(false)
  {
    // update potentially remapped topic name for later logging
    cloud_topic = nh.resolveName(cloud_topic);
    reconfigure_server.setCallback(boost::bind(&map3d_to_costmap::reconfigureCallback, this, _1));
  }

  void parse_ros_params()
  {
    file_name = get_param("~file_name", file_name);
    interval = get_param("~interval", interval);
    frame_id = get_param("~frame_id", frame_id);
    latch = get_param("~latch", latch);
    // bb_min.x = 0;
    // bb_min.y = 0;
    // bb_min.z = 0;

    // bb_max.x = 10;
    // bb_max.y = 10;
    // bb_max.z = 10;
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
    project_to_costmap();

    // success: set frame_id appropriately
    cloud.header.frame_id = frame_id;

    return true;
  }

  bool project_to_costmap()
  {
    // to get the minimum and maximum points. could be useful to do some automation of the process
    // (now the values are hardcoded)
    pcl::PointXYZ min_point, max_point;
    pcl::getMinMax3D(pointcloud, min_point, max_point);

    ROS_INFO_STREAM("Projecting pointcloud to costmap");
    ROS_INFO_STREAM(" * points: " << pointcloud.points.size());
    ROS_INFO_STREAM(" * limits: (" << min_point.x << ", " << min_point.y << ", " << min_point.z << ") to ("
                                   << max_point.x << ", " << max_point.y << ", " << max_point.z << ")");

    ROS_INFO_STREAM(" * projected limits: (" << bb_min.x << ", " << bb_min.y << ", " << bb_min.z << ") to (" << bb_max.x
                                             << ", " << bb_max.y << ", " << bb_max.z << ")");
    pcl::PointCloud<PointType>::Ptr pointcloud_ptr(new pcl::PointCloud<PointType>());
    *pointcloud_ptr = pointcloud;

    pcl::VoxelGrid<PointType> downsample;
    downsample.setInputCloud(pointcloud_ptr);
    downsample.setLeafSize(resolution, resolution, resolution);
    downsample.filter(*pointcloud_ptr);

    for (auto& point : *pointcloud_ptr)
    {
      // remove points that are outside the boundind box, by setting their z to a huge value
      if (point.z > bb_min.z and point.z < bb_max.z)
        point.z = 0;
      else
        point.z = 100;
    }

    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(pointcloud_ptr);

    std::vector<int> indices;
    std::vector<float> distances;

    grid.header.stamp = ros::Time::now();
    grid.header.frame_id = frame_id;

    grid.info.resolution = resolution;

    if (bb_min.x < bb_max.x)
      grid.info.width = ((int)((bb_max.x - bb_min.x) / resolution)) + 1;
    else  // this is nonsense, so fuck it
      grid.info.width = 1;
    if (bb_min.y < bb_max.y)
      grid.info.height = ((int)((bb_max.y - bb_min.y) / resolution)) + 1;
    else  // this is nonsense, so fuck it
      grid.info.height = 1;

    grid.info.origin.position.x = bb_min.x;
    grid.info.origin.position.y = bb_min.y;
    size_t grid_size = grid.info.width * grid.info.height;
    grid.data = std::vector<int8_t>(grid_size, 0);

    int maxn = 0;
    int index = 0;
    for (int index_y = 0; index_y < grid.info.height; index_y++)
    {
      for (int index_x = 0; index_x < grid.info.width; index_x++)
      {
        double x = bb_min.x + index_x * resolution;
        double y = bb_min.y + index_y * resolution;
        PointType point(x, y, 0);
        int neighbours = kdtree.radiusSearch(point, resolution, indices, distances);
        if (neighbours > occupancy_threshold)
          grid.data[index] = 100;
        else
          grid.data[index] = 0;
        index++;
        if (maxn < neighbours)
          maxn = neighbours;
      }
    }

    ROS_INFO("Index: %d, size: %d", index, grid.data.size());
    ROS_INFO("Max neighbours: %d", maxn);
    // ROS_INFO_STREAM("Info: " << grid.info);

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
    running = true;
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

  void reconfigureCallback(map3d_server::Map3dToCostmapConfig& config)
  {
    ROS_INFO("Reconfigure called");
    bb_min.x = config.min_x;
    bb_min.y = config.min_y;
    bb_min.z = config.min_z;

    bb_max.x = config.max_x;
    bb_max.y = config.max_y;
    bb_max.z = config.max_z;

    resolution = config.resolution;
    occupancy_threshold = config.occupancy_threshold;
    // trigger reprojection only if node is running
    if (running)
    {
      project_to_costmap();
      publish();
    }
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
