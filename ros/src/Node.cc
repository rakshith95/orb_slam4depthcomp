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

  node_handle_.param<std::string>(name_of_node_ + "/storage_path", storage_path_, "~");
  chdir(storage_path_.c_str());

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

    // listen to camera pose
    while (!got_tf_)
    {
      try
      {
        listener_.lookupTransform(base_footprint_frame_id_, robot_camera_frame_id_,
                                  ros::Time(0), camera_pose_);
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

void Node::storeData()
{
  std::cerr << "Storing data!!!" << std::endl;

  // Stop all threads
  orb_slam_->Shutdown();

  // Save camera trajectory
  orb_slam_->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

  delete orb_slam_;
}

void Node::Update()
{
  cv::Mat position = orb_slam_->GetCurrentPosition();
  if (position.empty())
  {
    position = cv::Mat::eye(4, 4, CV_32F);
  }

  PublishPositionAsTransform(position);

  if (publish_pose_param_)
  {
    PublishPositionAsPoseStamped(position);
  }

  PublishRenderedImage(orb_slam_->DrawCurrentFrame());

  if (publish_pointcloud_param_)
  {
    PublishMapPoints(orb_slam_->GetAllMapPoints());
  }
}

void Node::PublishMapPoints(std::vector<ORB_SLAM2::MapPoint*> map_points)
{
  sensor_msgs::PointCloud2 cloud = MapPointsToPointCloud(map_points);
  map_points_publisher_.publish(cloud);
}

void Node::PublishPositionAsTransform(cv::Mat position)
{
  // old behavior
  if (!correct_global_frame_)
  {
    tf::Transform transform = TransformFromMat(position);
    static tf::TransformBroadcaster tf_broadcaster;
    tf_broadcaster.sendTransform(tf::StampedTransform(
        transform, current_frame_time_, map_frame_id_param_, camera_frame_id_param_));
    return;
  }

  // new behavior
  if (!got_tf_)
  {
    ROS_WARN_STREAM("Still waiting for base_link->camera transform!!!");
    return;
  }

  tf::StampedTransform odom_tf;
  try
  {
    listener_.lookupTransform(odom_frame_id_, base_footprint_frame_id_, ros::Time(0), odom_tf);
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("%s", ex.what());
    ros::Duration(1.0).sleep();
    return;
  }

  ros::Time shifted_time = current_frame_time_ + ros::Duration(transform_tolerance_);

  if (!first_ && !hasMovedEnough(odom_tf))
  {
    tf_broadcaster.sendTransform(tf::StampedTransform(
        camera_pose_, shifted_time, corrected_map_frame_id_, map_frame_id_param_));

    tf_broadcaster.sendTransform(tf::StampedTransform(
        last_corrected_pose_, shifted_time, corrected_map_frame_id_, odom_frame_id_));
    return;
  }

  // update corrected tf
  tf::Transform current_tf = TransformFromMat(position);
  tf::Transform tf1 = camera_pose_.inverse() * odom_tf.inverse();
  tf::Transform tf2 = current_tf * tf1;
  tf::Transform corrected_tf = camera_pose_ * tf2;

  // extract only 2d tf
  tf::Transform only_2d_tf;
  only_2d_tf.setIdentity();
  only_2d_tf.setOrigin(
      tf::Vector3(corrected_tf.getOrigin().getX(), corrected_tf.getOrigin().getY(), 0));
  tf::Quaternion q;
  q.setEuler(0, 0, tf::getYaw(corrected_tf.getRotation()));
  only_2d_tf.setRotation(q);

  last_corrected_pose_ = only_2d_tf;

  tf_broadcaster.sendTransform(tf::StampedTransform(
      camera_pose_, shifted_time, corrected_map_frame_id_, map_frame_id_param_));

  tf_broadcaster.sendTransform(tf::StampedTransform(
      only_2d_tf, shifted_time, corrected_map_frame_id_, odom_frame_id_));

  first_ = false;
  last_odom_pose_.setData(odom_tf);
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
      data_array[1] = -1.0 *
                      map_points.at(i)->GetWorldPos().at<float>(
                          0);  // y. Do the transformation by just reading
                               // at the position of x instead of y
      data_array[2] = -1.0 *
                      map_points.at(i)->GetWorldPos().at<float>(
                          1);  // z. Do the transformation by just reading
                               // at the position of y instead of z
      // TODO dont hack the transformation but have a central conversion function for
      // MapPointsToPointCloud and
      // TransformFromMat

      memcpy(cloud_data_ptr + (i * cloud.point_step), data_array, num_channels * sizeof(float));
    }
  }

  return cloud;
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
