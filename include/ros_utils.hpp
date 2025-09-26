#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/static_transform_broadcaster.h"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "sophus/se3.hpp"
#include "cv_bridge/cv_bridge.h"
#include <visualization_msgs/msg/marker.hpp>
#include "System.h"
#include "Map.h"

using ImageMsg = sensor_msgs::msg::Image;
using PcdMsg = sensor_msgs::msg::PointCloud2;
using PoseMsg = geometry_msgs::msg::PoseStamped;
using OdomMsg = nav_msgs::msg::Odometry;
using MarkerMsg = visualization_msgs::msg::Marker;

void publish_camera_pose(
  const rclcpp::Publisher<PoseMsg>::SharedPtr& publisher, 
  const rclcpp::Time& stamp,
  Sophus::SE3f& Twc,  
  const std::string& world_frame_id
);

void publish_body_odometry(
  const rclcpp::Publisher<OdomMsg>::SharedPtr& publisher,
  const rclcpp::Time& stamp, 
  Sophus::SE3f Twb, 
  Eigen::Vector3f Vwb, 
  Eigen::Vector3f Wwb,
  const std::string& world_frame_id,
  std::string odom_frame_id
);

void publish_tracking_img(
  const rclcpp::Publisher<ImageMsg>::SharedPtr& publisher,
  const rclcpp::Time& stamp,
  cv::Mat image, 
  std::string frame_id
); 

void publish_camera_tf(
  const std::shared_ptr<tf2_ros::TransformBroadcaster>& tf_broadcaster,
  const rclcpp::Time& stamp,
  Sophus::SE3f& Twc,
  std::string parent_frame_id,
  std::string child_frame_id
);

void publish_world_to_odom_tf(
  const std::shared_ptr<tf2_ros::TransformBroadcaster>& tf_broadcaster,
  const rclcpp::Time& stamp,
  Sophus::SE3f& Two,
  std::string parent_frame_id,
  std::string child_frame_id
); 

void publish_optical_to_frame_tf(
  const std::shared_ptr<tf2_ros::StaticTransformBroadcaster>& tf_static_broadcaster,
  const rclcpp::Time& stamp,
  std::string parent_frame_id,
  std::string child_frame_id
);

sensor_msgs::msg::PointCloud2 keypoints_to_pointcloud(std::vector<cv::KeyPoint>&, const rclcpp::Time&, const std::string&);
sensor_msgs::msg::PointCloud2 mappoint_to_pointcloud(const std::vector<ORB_SLAM3::MapPoint*>&, const rclcpp::Time&, std::string&);

//void publishKeyframeOutputs_(const rclcpp::Time& stamp_like);

void publish_keypoints(
  const rclcpp::Publisher<PcdMsg>::SharedPtr& publisher,
  const std::vector<ORB_SLAM3::MapPoint*>& tracked_map_points,
  const std::vector<cv::KeyPoint>& tracked_keypoints,
  const rclcpp::Time& msg_time,
  const std::string& world_frame_id
);

void publish_tracked_points(
  const rclcpp::Publisher<PcdMsg>::SharedPtr& publisher,
  const std::vector<ORB_SLAM3::MapPoint*>& tracked_points,
  const rclcpp::Time& msg_time,
  const std::string& world_frame_id
);

void publish_all_points(
  const rclcpp::Publisher<PcdMsg>::SharedPtr& publisher,
  const std::vector<ORB_SLAM3::MapPoint*>& map_points,
  const rclcpp::Time& msg_time,
  const std::string& world_frame_id
);

void publish_kf_markers(
  const rclcpp::Publisher<MarkerMsg>::SharedPtr& publisher,
  const std::vector<Sophus::SE3f>& vKFposes,
  const rclcpp::Time& msg_time,
  const std::string& world_frame_id
);

Eigen::Affine3f transform_to_eigen(
  const geometry_msgs::msg::TransformStamped& transform_stamped
);

Sophus::SE3f transform_to_SE3(
  const geometry_msgs::msg::TransformStamped& transform_stamped
);