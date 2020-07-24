/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of
* Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef ORBSLAM2_ROS_NODE_H_
#define ORBSLAM2_ROS_NODE_H_

#include <vector>
#include <unordered_map>

#include <ros/ros.h>
#include <ros/time.h>
#include <image_transport/image_transport.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>

#include <dynamic_reconfigure/server.h>
#include <orb_slam2_ros/dynamic_reconfigureConfig.h>

#include "orb_slam2_ros/SaveMap.h"

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/CameraInfo.h>

#include "System.h"
#include "Converter.h"

#include <occupancy_grid_extractor/types.h>
#include <occupancy_grid_extractor/utils.h>
#include <occupancy_grid_extractor/mapper_3d.h>
#include <occupancy_grid_extractor/occupancy_grid_extractor.h>

#include <nav_msgs/OccupancyGrid.h>
#include <navigation_utils/offline_tools.h>
#include <pal_map_utils/MapImgUtils.h>

#include <Eigen/Geometry>
#include <tf/transform_listener.h>
#include <angles/angles.h>

#include <future>
#include <boost/thread.hpp>

class Node
{
public:
  typedef std::unordered_map<double, cv::Mat> DoubleCvMatUnorderedMap;

  Node(ORB_SLAM2::System::eSensor sensor, ros::NodeHandle &node_handle,
       image_transport::ImageTransport &image_transport);
  ~Node();

protected:
  void Update();
  ORB_SLAM2::System *orb_slam_;

  ros::Time current_frame_time_;

  DoubleCvMatUnorderedMap image_dataset_;

  bool got_camera_info_ = false;
  Eigen::Matrix3d K_;

  bool got_camera_pose_ = false;
  tf::StampedTransform camera_pose_;

  tf::TransformListener listener_;
  std::string base_footprint_frame_id_ = "base_footprint";
  std::string robot_camera_frame_id_ = "torso_front_camera_depth_optical_frame";

private:
  void publishOccupancyGrid();
  void PublishMapPoints(std::vector<ORB_SLAM2::MapPoint *> map_points);
  void PublishRobotPose(cv::Mat position);
  void PublishPositionAsTransform(cv::Mat position);
  void PublishPositionAsPoseStamped(cv::Mat position);
  void PublishRenderedImage(cv::Mat image);
  void ParamsChangedCallback(orb_slam2_ros::dynamic_reconfigureConfig &config, uint32_t level);
  bool SaveMapSrv(orb_slam2_ros::SaveMap::Request &req, orb_slam2_ros::SaveMap::Response &res);

  void cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr &camera_info_msg);

  tf::Transform TransformFromMat(cv::Mat position_mat);
  sensor_msgs::PointCloud2 MapPointsToPointCloud(std::vector<ORB_SLAM2::MapPoint *> map_points);

  dynamic_reconfigure::Server<orb_slam2_ros::dynamic_reconfigureConfig> dynamic_param_server_;

  image_transport::Publisher rendered_image_publisher_;
  ros::Publisher map_points_publisher_;
  ros::Publisher pose_publisher_;

  ros::ServiceServer service_server_;

  std::string name_of_node_;
  ros::NodeHandle node_handle_;

  std::string map_frame_id_param_;
  std::string camera_frame_id_param_;
  std::string map_file_name_param_;
  std::string voc_file_name_param_;
  std::string settings_file_name_param_;
  bool load_map_param_;
  bool publish_pointcloud_param_;
  bool publish_pose_param_;
  int min_observations_per_point_;

  ros::Subscriber camera_info_sub_;
  ros::Publisher occupancy_grid_pub_;

protected:
  // pal stuff
  bool first_ = true;
  bool got_tf_ = false;

  tf::StampedTransform odom_tf_;
  tf::StampedTransform last_odom_tf_;
  tf::Transform last_corrected_pose_;


  tf::StampedTransform last_odom_pose_;

  tf::TransformBroadcaster tf_broadcaster;

  bool correct_global_frame_;

  std::string odom_frame_id_;
  std::string corrected_map_frame_id_;

  double minimum_travel_distance_;
  double minimum_travel_heading_;

  double transform_tolerance_;

  bool hasMovedEnough()
  {
    double travel_distance = (odom_tf_.getOrigin() - last_odom_tf_.getOrigin()).length();
    double travel_heading = angles::shortest_angular_distance(
        tf::getYaw(odom_tf_.getRotation()), tf::getYaw(last_odom_tf_.getRotation()));
    return (travel_distance > minimum_travel_distance_ || travel_heading > minimum_travel_heading_);
  }

  ros::Subscriber pose_sub_;
  void poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg);

  bool running_ = false;
  bool killed_ = false;
  std::future<void> pending_future_;
  boost::recursive_mutex lock_;
};

#endif  // ORBSLAM2_ROS_NODE_H_
