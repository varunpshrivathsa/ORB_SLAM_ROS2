#include <memory>
#include <string>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/synchronizer.h"

#include <opencv2/opencv.hpp>
#include <GL/glew.h>
#include "System.h"

class RGBDNode : public rclcpp::Node
{
public:
  RGBDNode()
  : Node("orbslam3_rgbd_node")
  {
    this->declare_parameter<std::string>("vocab_path", "/workspace/ORB_SLAM3/Vocabulary/ORBvoc.txt");
    this->declare_parameter<std::string>("config_path", "/workspace/orbslam_ros2/config/LIMO_D455.yaml");
    this->declare_parameter<std::string>("rgb_topic", "/camera/color/image_raw");
    this->declare_parameter<std::string>("depth_topic", "/camera/depth/image_raw");

    vocab_path_ = this->get_parameter("vocab_path").as_string();
    config_path_ = this->get_parameter("config_path").as_string();
    rgb_topic_ = this->get_parameter("rgb_topic").as_string();
    depth_topic_ = this->get_parameter("depth_topic").as_string();

    RCLCPP_INFO(this->get_logger(), "Starting ORB-SLAM3 RGB-D node");
    RCLCPP_INFO(this->get_logger(), "Vocab: %s", vocab_path_.c_str());
    RCLCPP_INFO(this->get_logger(), "Config: %s", config_path_.c_str());
    RCLCPP_INFO(this->get_logger(), "RGB topic: %s", rgb_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "Depth topic: %s", depth_topic_.c_str());

    slam_system_ = std::make_shared<ORB_SLAM3::System>(
      vocab_path_,
      config_path_,
      ORB_SLAM3::System::RGBD,
      true
    );

    rgb_sub_.subscribe(this, rgb_topic_);
    depth_sub_.subscribe(this, depth_topic_);

    sync_ = std::make_shared<Synchronizer>(SyncPolicy(10), rgb_sub_, depth_sub_);
    sync_->registerCallback(
      std::bind(&RGBDNode::rgbd_callback, this, std::placeholders::_1, std::placeholders::_2)
    );
  }

  ~RGBDNode()
  {
    if (slam_system_) {
      slam_system_->Shutdown();
      slam_system_->SaveTrajectoryTUM("CameraTrajectory.txt");
      slam_system_->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
    }
  }

private:
  using ImageMsg = sensor_msgs::msg::Image;
  using SyncPolicy = message_filters::sync_policies::ApproximateTime<ImageMsg, ImageMsg>;
  using Synchronizer = message_filters::Synchronizer<SyncPolicy>;

  void rgbd_callback(
    const sensor_msgs::msg::Image::ConstSharedPtr & rgb_msg,
    const sensor_msgs::msg::Image::ConstSharedPtr & depth_msg)
  {
    cv_bridge::CvImageConstPtr cv_rgb_ptr;
    cv_bridge::CvImageConstPtr cv_depth_ptr;

    try {
      cv_rgb_ptr = cv_bridge::toCvShare(rgb_msg, rgb_msg->encoding);
      cv_depth_ptr = cv_bridge::toCvShare(depth_msg, depth_msg->encoding);
    } catch (const cv_bridge::Exception & e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }

    cv::Mat rgb = cv_rgb_ptr->image.clone();
    cv::Mat depth = cv_depth_ptr->image.clone();

    double timestamp =
      rgb_msg->header.stamp.sec +
      rgb_msg->header.stamp.nanosec * 1e-9;

    slam_system_->TrackRGBD(rgb, depth, timestamp);
  }

  std::string vocab_path_;
  std::string config_path_;
  std::string rgb_topic_;
  std::string depth_topic_;

  std::shared_ptr<ORB_SLAM3::System> slam_system_;

  message_filters::Subscriber<ImageMsg> rgb_sub_;
  message_filters::Subscriber<ImageMsg> depth_sub_;
  std::shared_ptr<Synchronizer> sync_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RGBDNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}