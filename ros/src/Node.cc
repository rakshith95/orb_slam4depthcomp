#include "Node.h"

#include <iostream>

Node::Node(ORB_SLAM2::System::eSensor sensor, ros::NodeHandle& node_handle,
           image_transport::ImageTransport& image_transport)
{
  name_of_node_ = ros::this_node::getName();
  node_handle_ = node_handle;
  min_observations_per_point_ = 2;

  // static parameters
  node_handle_.param(name_of_node_ + "/publish_pointcloud", publish_pointcloud_param_, true);
  node_handle_.param(name_of_node_ + "/publish_pose", publish_pose_param_, true);
  node_handle_.param<std::string>(name_of_node_ + "/pointcloud_frame_id",
                                  map_frame_id_param_, "map");
  node_handle_.param<std::string>(name_of_node_ + "/camera_frame_id",
                                  camera_frame_id_param_, "camera_link");
  node_handle_.param<std::string>(name_of_node_ + "/map_file", map_file_name_param_, "map.bin");
  node_handle_.param<std::string>(name_of_node_ + "/voc_file", voc_file_name_param_,
                                  "file_not_set");
  node_handle_.param<std::string>(name_of_node_ + "/settings_file",
                                  settings_file_name_param_, "file_not_set");
  node_handle_.param(name_of_node_ + "/load_map", load_map_param_, false);

  orb_slam_ = new ORB_SLAM2::System(voc_file_name_param_, settings_file_name_param_,
                                    sensor, map_file_name_param_, load_map_param_);

  service_server_ =
      node_handle_.advertiseService(name_of_node_ + "/save_map", &Node::SaveMapSrv, this);

  // Setup dynamic reconfigure
  dynamic_reconfigure::Server<orb_slam2_ros::dynamic_reconfigureConfig>::CallbackType dynamic_param_callback;
  dynamic_param_callback = boost::bind(&Node::ParamsChangedCallback, this, _1, _2);
  dynamic_param_server_.setCallback(dynamic_param_callback);

  rendered_image_publisher_ = image_transport.advertise(name_of_node_ + "/debug_image", 1);
  if (publish_pointcloud_param_)
  {
    map_points_publisher_ =
        node_handle_.advertise<sensor_msgs::PointCloud2>(name_of_node_ + "/map_points", 1);
  }

  // Enable publishing camera's pose as PoseStamped message
  if (publish_pose_param_)
  {
    pose_publisher_ = node_handle_.advertise<geometry_msgs::PoseWithCovarianceStamped>(
        name_of_node_ + "/pose", 1);
  }

  node_handle_.param(name_of_node_ + "/correct_global_frame", correct_global_frame_, false);
  if (correct_global_frame_)
  {
    node_handle_.param<std::string>(name_of_node_ + "/base_footprint_frame_id",
                                    base_footprint_frame_id_, "base_"
                                                              "footprint");
    node_handle_.param<std::string>(name_of_node_ + "/robot_camera_frame_id",
                                    robot_camera_frame_id_, "torso_front_"
                                                            "camera_link");
    node_handle_.param<std::string>(name_of_node_ + "/odom_frame_id", odom_frame_id_, "odom");
    node_handle_.param<std::string>(name_of_node_ + "/corrected_map_frame_id",
                                    corrected_map_frame_id_, "map");

    node_handle_.param(name_of_node_ + "/minimum_travel_distance", minimum_travel_distance_, 0.2);
    node_handle_.param(name_of_node_ + "/minimum_travel_heading", minimum_travel_heading_, 0.017);
    node_handle_.param(name_of_node_ + "/transform_tolerance", transform_tolerance_, 0.27);

    pose_sub_ = node_handle_.subscribe("/initialpose", 1, &Node::poseCallback, this);

    // let's wait for tf buffer to fill up
    ros::Duration(2.0).sleep();

    // listen to camera pose
    while (!got_tf_)
    {
      try
      {
        listener_.lookupTransform(base_footprint_frame_id_, robot_camera_frame_id_,
                                  ros::Time(0), camera_pose_);
        listener_.lookupTransform(base_footprint_frame_id_,
                                  "torso_front_camera_depth_optical_frame", ros::Time(0),
                                  camera_offset_);
      }
      catch (tf::TransformException ex)
      {
        ROS_ERROR("%s", ex.what());
        ros::Duration(1.0).sleep();
        continue;
      }
      got_tf_ = true;
    }
  }

  // subscribe to camera_info
  camera_info_sub_ =
      node_handle_.subscribe("/torso_front_camera/aligned_depth_to_color/camera_info", 1,
                             &Node::cameraInfoCallback, this);

  // occupancy_grid publisher
  occupancy_grid_pub_ = node_handle_.advertise<nav_msgs::OccupancyGrid>("/map", 1);

  ROS_INFO_STREAM("Started orb_slam2_ros node!!!");
}


Node::~Node()
{
  // Stop all threads
  orb_slam_->Shutdown();

  // Save camera trajectory
  orb_slam_->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

  delete orb_slam_;
}

void Node::Update()
{
  cv::Mat position = orb_slam_->GetCurrentPosition();

  if (!correct_global_frame_)  // old behavior
  {
    PublishPositionAsTransform(position);
  }
  else  // new behavior
  {
    PublishRobotPose(position);
  }

  if (publish_pose_param_)
  {
    if (position.empty())
    {
      position = cv::Mat::eye(4, 4, CV_32F);
    }
    PublishPositionAsPoseStamped(position);
  }

  PublishRenderedImage(orb_slam_->DrawCurrentFrame());

  if (publish_pointcloud_param_)
  {
    PublishMapPoints(orb_slam_->GetAllMapPoints());
  }

  // check if a keyframe has been added
  if (orb_slam_->tracker()->created_new_key_frame_)
  {
    // publish occupancy grid
    boost::recursive_mutex::scoped_lock lock(lock_);
    if (running_)
    {
      killed_ = true;
      pending_future_.wait();
    }
    pending_future_ = std::async(std::launch::async, &Node::publishOccupancyGrid, this);
  }
}

void Node::publishOccupancyGrid()
{
  if (!got_camera_info_ || !got_tf_)
  {
    return;
  }

  running_ = true;

  float depth_scale = 1e-3f;
  float min_distance = 0.35f;
  float max_distance = 1.85f;
  float merging_distance = 0.05f;
  double voxel_grid_resolution = 0.025;
  double occupancy_grid_resolution = 0.05;

  // instantiate mapper
  occupancy_grid_extractor::Mapper3d mapper(min_distance, max_distance, merging_distance,
                                            voxel_grid_resolution);
  mapper.setK(K_);

  // retrieve keyframe
  std::vector<ORB_SLAM2::KeyFrame*> vpKFs = orb_slam_->map()->GetAllKeyFrames();
  sort(vpKFs.begin(), vpKFs.end(), ORB_SLAM2::KeyFrame::lId);

  if (vpKFs.size() < 6)
  {
    running_ = false;
    return;
  }

  // assemble global cloud
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  occupancy_grid_extractor::Vector3dVector global_cloud;
  for (size_t i = 0; i < vpKFs.size(); i++)
  {
    if (killed_)
    {
      running_ = false;
      killed_ = false;
      return;
    }

    // get current keyframe
    ORB_SLAM2::KeyFrame* pKF = vpKFs[i];
    if (pKF->isBad())
    {
      continue;
    }

    // retrieve keyframe pose
    cv::Mat R = pKF->GetRotation().t();
    vector<float> q = ORB_SLAM2::Converter::toQuaternion(R);
    cv::Mat t = pKF->GetCameraCenter();
    pose.setIdentity();
    pose.translation() = Eigen::Vector3d(t.at<float>(0), t.at<float>(1), t.at<float>(2));
    pose.linear() = Eigen::Quaterniond(q[3], q[0], q[1], q[2]).toRotationMatrix();

    // retrieve keyframe depth image
    DoubleCvMatUnorderedMap::const_iterator it = image_dataset_.find(pKF->mTimeStamp);
    if (it == image_dataset_.end())
    {
      // TODO handle error
    }
    else
    {
      occupancy_grid_extractor::FloatImage current_image;
      occupancy_grid_extractor::convert_16UC1_to_32FC1(current_image, it->second, depth_scale);

      // process depth image
      mapper.processDepthImage(current_image, pose, global_cloud);
    }
  }

  // transform point cloud in global coordinate frame
  mapper.transformCloud(pose, global_cloud);

  // get camera pose wrt the robot
  Eigen::Isometry3d camera_pose = Eigen::Isometry3d::Identity();
  camera_pose.translation() =
      Eigen::Vector3d(camera_offset_.getOrigin().getX(), camera_offset_.getOrigin().getY(),
                      camera_offset_.getOrigin().getZ());
  camera_pose.linear() = Eigen::Quaterniond(camera_offset_.getRotation().getW(),
                                            camera_offset_.getRotation().getX(),
                                            camera_offset_.getRotation().getY(),
                                            camera_offset_.getRotation().getZ())
                             .toRotationMatrix();

  // transform point cloud in canonical coordinate frame
  mapper.transformCloud(camera_pose, global_cloud);

  // build occupancy grid
  navigation_utils::DenseGrid grid;
  occupancy_grid_extractor::OccupancyGridExtractor extractor(occupancy_grid_resolution);
  extractor.compute(global_cloud, grid);

  // convert grid to cv mat
  cv::Mat occupancy_img = navigation_utils::drawIndexImage(grid);

  // create occupancy grid
  nav_msgs::OccupancyGrid occupancy_grid;
  pal_map_utils::cvMat2occGrid(occupancy_img, occupancy_grid);

  // set resolution
  occupancy_grid.info.resolution = grid.resolution();

  // set origin
  tf::Transform map_origin;
  map_origin.setIdentity();
  map_origin.setOrigin(tf::Vector3(grid.originX(), grid.originY(), 0));
  tf::poseTFToMsg(map_origin, occupancy_grid.info.origin);

  // publish map
  occupancy_grid_pub_.publish(occupancy_grid);

  running_ = false;
}

void Node::PublishMapPoints(std::vector<ORB_SLAM2::MapPoint*> map_points)
{
  sensor_msgs::PointCloud2 cloud = MapPointsToPointCloud(map_points);
  map_points_publisher_.publish(cloud);
}

void Node::PublishRobotPose(cv::Mat position)
{
  ros::Time shifted_time = current_frame_time_ + ros::Duration(transform_tolerance_);

  if (!first_ && !hasMovedEnough())  // no need to update the tf
  {
    tf_broadcaster.sendTransform(tf::StampedTransform(
        camera_pose_, shifted_time, corrected_map_frame_id_, map_frame_id_param_));

    tf_broadcaster.sendTransform(tf::StampedTransform(
        last_corrected_pose_, shifted_time, corrected_map_frame_id_, odom_frame_id_));

    return;
  }
  else if (orb_slam_->tracker()->state() ==
           ORB_SLAM2::Tracking::OK)  // update corrected tf
  {
    tf::Transform corrected_tf =
        camera_pose_ * (TransformFromMat(position) * (odom_tf_ * camera_pose_).inverse());

    // extract only 2d tf
    tf::Transform only_2d_tf;
    only_2d_tf.setIdentity();
    only_2d_tf.setOrigin(
        tf::Vector3(corrected_tf.getOrigin().getX(), corrected_tf.getOrigin().getY(), 0));
    tf::Quaternion q;
    q.setEuler(0, 0, tf::getYaw(corrected_tf.getRotation()));
    only_2d_tf.setRotation(q);
    last_corrected_pose_ = only_2d_tf;

    // broadcast robot pose with transform estimated by visual slam
    tf_broadcaster.sendTransform(tf::StampedTransform(
        camera_pose_, shifted_time, corrected_map_frame_id_, map_frame_id_param_));

    tf_broadcaster.sendTransform(tf::StampedTransform(
        only_2d_tf, shifted_time, corrected_map_frame_id_, odom_frame_id_));
  }

  // update last odom
  last_odom_tf_.setData(odom_tf_);

  // set first to false
  first_ = false;
}

void Node::PublishPositionAsTransform(cv::Mat position)
{
  tf::Transform transform = TransformFromMat(position);
  static tf::TransformBroadcaster tf_broadcaster;
  tf_broadcaster.sendTransform(tf::StampedTransform(
      transform, current_frame_time_, map_frame_id_param_, camera_frame_id_param_));
}

void Node::PublishPositionAsPoseStamped(cv::Mat position)
{
  tf::Transform grasp_tf = TransformFromMat(position);
  tf::Stamped<tf::Pose> grasp_tf_pose(grasp_tf, current_frame_time_, map_frame_id_param_);
  //  geometry_msgs::PoseStamped pose_msg;
  //  tf::poseStampedTFToMsg(grasp_tf_pose, pose_msg.pose);
  geometry_msgs::PoseWithCovarianceStamped pose_msg;
  pose_msg.header.frame_id = map_frame_id_param_;
  pose_msg.header.stamp = current_frame_time_;
  tf::poseTFToMsg(grasp_tf, pose_msg.pose.pose);

  pose_publisher_.publish(pose_msg);
}

void Node::PublishRenderedImage(cv::Mat image)
{
  std_msgs::Header header;
  header.stamp = current_frame_time_;
  header.frame_id = map_frame_id_param_;
  const sensor_msgs::ImagePtr rendered_image_msg =
      cv_bridge::CvImage(header, "bgr8", image).toImageMsg();
  rendered_image_publisher_.publish(rendered_image_msg);
}

tf::Transform Node::TransformFromMat(cv::Mat position_mat)
{
  cv::Mat rotation(3, 3, CV_32F);
  cv::Mat translation(3, 1, CV_32F);

  rotation = position_mat.rowRange(0, 3).colRange(0, 3);
  translation = position_mat.rowRange(0, 3).col(3);

  tf::Matrix3x3 tf_camera_rotation(
      rotation.at<float>(0, 0), rotation.at<float>(0, 1), rotation.at<float>(0, 2),
      rotation.at<float>(1, 0), rotation.at<float>(1, 1), rotation.at<float>(1, 2),
      rotation.at<float>(2, 0), rotation.at<float>(2, 1), rotation.at<float>(2, 2));

  tf::Vector3 tf_camera_translation(translation.at<float>(0), translation.at<float>(1),
                                    translation.at<float>(2));

  // Coordinate transformation matrix from orb coordinate system to ros coordinate system
  const tf::Matrix3x3 tf_orb_to_ros(0, 0, 1, -1, 0, 0, 0, -1, 0);

  // Transform from orb coordinate system to ros coordinate system on camera coordinates
  tf_camera_rotation = tf_orb_to_ros * tf_camera_rotation;
  tf_camera_translation = tf_orb_to_ros * tf_camera_translation;

  // Inverse matrix
  tf_camera_rotation = tf_camera_rotation.transpose();
  tf_camera_translation = -(tf_camera_rotation * tf_camera_translation);

  // Transform from orb coordinate system to ros coordinate system on map coordinates
  tf_camera_rotation = tf_orb_to_ros * tf_camera_rotation;
  tf_camera_translation = tf_orb_to_ros * tf_camera_translation;

  return tf::Transform(tf_camera_rotation, tf_camera_translation);
}

sensor_msgs::PointCloud2 Node::MapPointsToPointCloud(std::vector<ORB_SLAM2::MapPoint*> map_points)
{
  if (map_points.size() == 0)
  {
    std::cout << "Map point vector is empty!" << std::endl;
  }

  sensor_msgs::PointCloud2 cloud;

  const int num_channels = 3;  // x y z

  cloud.header.stamp = current_frame_time_;
  cloud.header.frame_id = map_frame_id_param_;
  cloud.height = 1;
  cloud.width = map_points.size();
  cloud.is_bigendian = false;
  cloud.is_dense = true;
  cloud.point_step = num_channels * sizeof(float);
  cloud.row_step = cloud.point_step * cloud.width;
  cloud.fields.resize(num_channels);

  std::string channel_id[] = { "x", "y", "z" };
  for (int i = 0; i < num_channels; i++)
  {
    cloud.fields[i].name = channel_id[i];
    cloud.fields[i].offset = i * sizeof(float);
    cloud.fields[i].count = 1;
    cloud.fields[i].datatype = sensor_msgs::PointField::FLOAT32;
  }

  cloud.data.resize(cloud.row_step * cloud.height);

  unsigned char* cloud_data_ptr = &(cloud.data[0]);

  float data_array[num_channels];
  for (unsigned int i = 0; i < cloud.width; i++)
  {
    if (map_points.at(i)->nObs >= min_observations_per_point_)
    {
      data_array[0] =
          map_points.at(i)->GetWorldPos().at<float>(2);  // x. Do the transformation by
                                                         // just reading at the position
                                                         // of z instead of x
      data_array[1] = -1.0 * map_points.at(i)->GetWorldPos().at<float>(0);  // y. Do the
      // transformation by just
      // reading at the position
      // of x instead of y
      data_array[2] = -1.0 * map_points.at(i)->GetWorldPos().at<float>(1);  // z. Do the
      // transformation by just
      // reading at the position
      // of y instead of z
      // TODO dont hack the transformation but have a central conversion function for
      // MapPointsToPointCloud and
      // TransformFromMat

      memcpy(cloud_data_ptr + (i * cloud.point_step), data_array, num_channels * sizeof(float));
    }
  }

  return cloud;
}

void Node::poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
  tf::Transform map_tf;
  tf::poseMsgToTF(msg->pose.pose, map_tf);

  last_corrected_pose_ = map_tf * last_odom_tf_.inverse();
}

void Node::ParamsChangedCallback(orb_slam2_ros::dynamic_reconfigureConfig& config, uint32_t level)
{
  orb_slam_->EnableLocalizationOnly(config.localize_only);
  min_observations_per_point_ = config.min_observations_for_ros_map;

  if (config.reset_map)
  {
    orb_slam_->Reset();
    config.reset_map = false;
  }

  orb_slam_->SetMinimumKeyFrames(config.min_num_kf_in_map);
}

bool Node::SaveMapSrv(orb_slam2_ros::SaveMap::Request& req, orb_slam2_ros::SaveMap::Response& res)
{
  res.success = orb_slam_->SaveMap(req.name);

  if (res.success)
  {
    ROS_INFO_STREAM("Map was saved as " << req.name);
  }
  else
  {
    ROS_ERROR("Map could not be saved.");
  }

  orb_slam_->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

  if (orb_slam_->sensor() == ORB_SLAM2::System::STEREO)
  {
    ofstream out;
    out.open("dataset.txt");
    out << std::fixed;

    for (size_t i = 0; i < dpt_dataset_.size(); ++i)
    {
      out << dpt_dataset_[i].first << " " << dpt_dataset_[i].second << std::endl;
    }
    out.close();
    cout << endl << "depth images dataset saved!" << endl;
  }

  return res.success;
}

void Node::cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& camera_info_msg)
{
  K_ << camera_info_msg->K[0], camera_info_msg->K[1], camera_info_msg->K[2],
      camera_info_msg->K[3], camera_info_msg->K[4], camera_info_msg->K[5],
      camera_info_msg->K[6], camera_info_msg->K[7], camera_info_msg->K[8];
  got_camera_info_ = true;
  camera_info_sub_.shutdown();
}
