#include "RGBDNode.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "RGBD");
    ros::start();

    if(argc > 1) {
        ROS_WARN ("Arguments supplied via command line are neglected.");
    }

    ros::NodeHandle node_handle;

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    image_transport::ImageTransport image_transport (node_handle);

    RGBDNode node (ORB_SLAM2::System::RGBD, node_handle, image_transport);

    ros::spin();

    ros::shutdown();

    return 0;
}


RGBDNode::RGBDNode (const ORB_SLAM2::System::eSensor sensor, ros::NodeHandle &node_handle, image_transport::ImageTransport &image_transport) : Node (sensor, node_handle, image_transport) {
  image_transport::ImageTransport it(node_handle);
  image_transport::TransportHints hints("compressed");

  rgb_subscriber_ = new message_filters::Subscriber<sensor_msgs::Image> (node_handle, "/camera/rgb/image_raw", 1);
  depth_subscriber_ = new message_filters::Subscriber<sensor_msgs::Image> (node_handle, "/camera/depth_registered/image_raw", 1);
  sync_ = new message_filters::Synchronizer<sync_pol> (sync_pol(10), *rgb_subscriber_, *depth_subscriber_);

  // compressed_rgb_subscriber_ = new message_filters::Subscriber<sensor_msgs::CompressedImage> (node_handle, "/camera/rgb/image_raw/compressed", 1);
  // compressed_depth_subscriber_ = new message_filters::Subscriber<sensor_msgs::CompressedImage> (node_handle, "/camera/depth_registered/compressed", 1);
  compressed_rgb_subscriber_ = new  image_transport::SubscriberFilter(it, "/camera/rgb/image_raw", 1, hints);
  compressed_depth_subscriber_ = new  image_transport::SubscriberFilter(it, "/camera/depth_registered", 1, hints);

  sync_CompressedImageRawDepth_ = new message_filters::Synchronizer<sync_pol> (sync_pol(10), *compressed_rgb_subscriber_, *depth_subscriber_);
  sync_CompressedImageCompressedDepth_ = new message_filters::Synchronizer<sync_pol> (sync_pol(10), *compressed_rgb_subscriber_, *compressed_depth_subscriber_);

  if (!is_compressed_param_ && !is_OnlyImage_compressed_param_)
    sync_->registerCallback(boost::bind(&RGBDNode::ImageCallback, this, _1, _2));
  else
  {
    if (is_OnlyImage_compressed_param_)
    {  sync_CompressedImageRawDepth_->registerCallback(boost::bind(&RGBDNode::CompressedImageRawDepthCallback, this, _1, _2));
    }
    else
    {
      sync_CompressedImageCompressedDepth_->registerCallback(boost::bind(&RGBDNode::CompressedImageCompressedDepthCallback, this, _1, _2));
    }
  }
  
}


RGBDNode::~RGBDNode () {
  delete rgb_subscriber_;
  delete depth_subscriber_;
  delete sync_;
}


void RGBDNode::ImageCallback (const sensor_msgs::ImageConstPtr& msgRGB, const sensor_msgs::ImageConstPtr& msgD) {
  // Copy the ros image message to cv::Mat.
  cv_bridge::CvImageConstPtr cv_ptrRGB;
  try {
      cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
  } catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
  }

  cv_bridge::CvImageConstPtr cv_ptrD;
  try {
    cv_ptrD = cv_bridge::toCvShare(msgD);
  } catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
  }

  current_frame_time_ = msgRGB->header.stamp;

  orb_slam_->TrackRGBD(cv_ptrRGB->image,cv_ptrD->image,cv_ptrRGB->header.stamp.toSec());
  Update ();
}

void RGBDNode::CompressedImageRawDepthCallback (const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD) {
  // Copy the ros image message to cv::Mat.

  cv::Mat rgb_image; //Actually bgr
  cv_bridge::CvImagePtr cv_ptr;
  try {
      // rgb_image = cv::imdecode(cv::Mat(msgRGB->data),cv::IMREAD_UNCHANGED);//convert compressed image data to cv::Mat
      cv_ptr = cv_bridge::toCvCopy(msgRGB);

  } catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
  }

  cv_bridge::CvImageConstPtr cv_ptrD;
  try {
    cv_ptrD = cv_bridge::toCvShare(msgD);
  } catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
  }
  cv::cvtColor(rgb_image, rgb_image, cv::COLOR_BGR2RGB); //Convert to RGB
  current_frame_time_ = msgRGB->header.stamp;

  orb_slam_->TrackRGBD(cv_ptr->image,cv_ptrD->image,cv_ptrD->header.stamp.toSec());
  Update ();
}

void RGBDNode::CompressedImageCompressedDepthCallback (const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD) {
  // Copy the ros image message to cv::Mat.
  std::cout<<"OOOH WHEEE \n";
  cv::Mat rgb_image;
  cv_bridge::CvImagePtr cv_ptr;

  try {
      // rgb_image = cv::imdecode(cv::Mat(msgRGB->data),1);//convert compressed image data to cv::Mat
      cv_ptr = cv_bridge::toCvCopy(msgRGB);

  } catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception for image: %s", e.what());
      return;
  }
 
   cv_bridge::CvImageConstPtr cv_ptrD;
  try {
    cv_ptrD = cv_bridge::toCvShare(msgD);
  } catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
  }
/*
 cv_bridge::CvImagePtr cv_ptrD;
  cv::Mat Depth, Depth_single;
  try {
      // cv_ptrD = cv_bridge::toCvShare(msgD,);
      cv_ptrD = cv_bridge::toCvShare(msgD);
      // Depth = cv::imdecode(cv::Mat(msgD->data),cv::IMREAD_UNCHANGED);//convert compressed image Depth data to cv::Mat
      // cv::extractChannel(Depth, Depth_single, 0); 
  } catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception for depth: %s", e.what());
      return;
  }*/
// std::cout<<Depth.type()<<endl;
  current_frame_time_ = msgRGB->header.stamp;

  orb_slam_->TrackRGBD(cv_ptr->image,cv_ptrD->image,current_frame_time_.toSec());
  Update ();
}
