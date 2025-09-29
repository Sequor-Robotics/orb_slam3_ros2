#include <ros_utils.hpp>

using ImageMsg = sensor_msgs::msg::Image;
using PcdMsg = sensor_msgs::msg::PointCloud2;
using PoseMsg = geometry_msgs::msg::PoseStamped;
using OdomMsg = nav_msgs::msg::Odometry;

void publish_camera_pose(
  const rclcpp::Publisher<PoseMsg>::SharedPtr& publisher, 
  const rclcpp::Time& stamp,
  Sophus::SE3f& Twc,  
  const std::string& world_frame_id)
{
  // Twc: pose matrix from world coordinate to camera coordinate
  geometry_msgs::msg::PoseStamped pose_msg;
  pose_msg.header.frame_id = world_frame_id;
  pose_msg.header.stamp = stamp;

  pose_msg.pose.position.x = Twc.translation().x();
  pose_msg.pose.position.y = Twc.translation().y();
  pose_msg.pose.position.z = Twc.translation().z();

  pose_msg.pose.orientation.w = Twc.unit_quaternion().coeffs().w();
  pose_msg.pose.orientation.x = Twc.unit_quaternion().coeffs().x();
  pose_msg.pose.orientation.y = Twc.unit_quaternion().coeffs().y();
  pose_msg.pose.orientation.z = Twc.unit_quaternion().coeffs().z();

  publisher->publish(pose_msg);
}

void publish_body_odometry(
  const rclcpp::Publisher<OdomMsg>::SharedPtr& publisher,
  const rclcpp::Time& stamp, 
  Sophus::SE3f Twb, 
  Eigen::Vector3f Vwb, 
  Eigen::Vector3f Wwb,
  const std::string& world_frame_id,
  std::string odom_frame_id)
{
  // Twb: pose matrix from world coordinate to body coordinate
  // Vwb: body velocity in world coordinate
  // Wwb: body angular velocity in world coordinate
  nav_msgs::msg::Odometry odom_msg;
  odom_msg.header.frame_id = world_frame_id;
  odom_msg.child_frame_id = odom_frame_id;
  odom_msg.header.stamp = stamp;

  // Coordinate Transform: OpenCV coordinate to ROS FLU coordinate
  Eigen::Matrix<float, 3, 3> cv_to_ros_rot; 
  Eigen::Matrix<float, 3, 1> cv_to_ros_trans; 
  cv_to_ros_rot << 0, -1, 0,
                  0, 0, -1,
                  1, 0, 0;
  cv_to_ros_trans << 0, 0, 0;
  Sophus::SE3f cv_to_ros(cv_to_ros_rot, cv_to_ros_trans);

  Twb = cv_to_ros * Twb; 
  odom_msg.pose.pose.position.x = Twb.translation().x();
  odom_msg.pose.pose.position.y = Twb.translation().y();
  odom_msg.pose.pose.position.z = Twb.translation().z();

  odom_msg.pose.pose.orientation.w = Twb.unit_quaternion().coeffs().w();
  odom_msg.pose.pose.orientation.x = Twb.unit_quaternion().coeffs().x();
  odom_msg.pose.pose.orientation.y = Twb.unit_quaternion().coeffs().y();
  odom_msg.pose.pose.orientation.z = Twb.unit_quaternion().coeffs().z();

  Vwb = cv_to_ros_rot * Vwb; 
  Wwb = cv_to_ros_rot * Wwb; 

  odom_msg.twist.twist.linear.x = Vwb.x();
  odom_msg.twist.twist.linear.y = Vwb.y();
  odom_msg.twist.twist.linear.z = Vwb.z();

  odom_msg.twist.twist.angular.x = Wwb.x();
  odom_msg.twist.twist.angular.y = Wwb.y();
  odom_msg.twist.twist.angular.z = Wwb.z();

  publisher->publish(odom_msg);
}

void publish_tracking_img(
  const rclcpp::Publisher<ImageMsg>::SharedPtr& publisher,
  const rclcpp::Time& stamp,
  cv::Mat image, 
  std::string frame_id)
{
  std_msgs::msg::Header header;

  header.stamp = stamp;
  header.frame_id = frame_id;

  sensor_msgs::msg::Image::SharedPtr image_msg = cv_bridge::CvImage(header, "bgr8", image).toImageMsg();

  publisher->publish(*image_msg.get());
}

void publish_camera_tf(
  const std::shared_ptr<tf2_ros::TransformBroadcaster>& tf_broadcaster,
  const rclcpp::Time& stamp,
  Sophus::SE3f& Twc,
  std::string parent_frame_id,
  std::string child_frame_id)
{
  // T: pose matrix from parent frame to child frame
  geometry_msgs::msg::TransformStamped t;

  t.header.stamp = stamp;
  t.header.frame_id = parent_frame_id;
  t.child_frame_id = child_frame_id;

  t.transform.translation.x = Twc.translation().x();
  t.transform.translation.y = Twc.translation().y();
  t.transform.translation.z = Twc.translation().z();

  t.transform.rotation.w = Twc.unit_quaternion().coeffs().w();
  t.transform.rotation.x = Twc.unit_quaternion().coeffs().x();
  t.transform.rotation.y = Twc.unit_quaternion().coeffs().y();
  t.transform.rotation.z = Twc.unit_quaternion().coeffs().z();

  tf_broadcaster->sendTransform(t);
}


void publish_world_to_odom_tf(
  const std::shared_ptr<tf2_ros::TransformBroadcaster>& tf_broadcaster,
  const rclcpp::Time& stamp,
  Sophus::SE3f& Two, 
  std::string parent_frame_id,
  std::string child_frame_id)
{
  // T: pose matrix from parent frame to child frame
  geometry_msgs::msg::TransformStamped t;

  t.header.stamp = stamp;
  t.header.frame_id = parent_frame_id;
  t.child_frame_id = child_frame_id;

  t.transform.translation.x = Two.translation().x();
  t.transform.translation.y = Two.translation().y();
  t.transform.translation.z = Two.translation().z();

  t.transform.rotation.w = Two.unit_quaternion().coeffs().w();
  t.transform.rotation.x = Two.unit_quaternion().coeffs().x();
  t.transform.rotation.y = Two.unit_quaternion().coeffs().y();
  t.transform.rotation.z = Two.unit_quaternion().coeffs().z();

  tf_broadcaster->sendTransform(t);
}

void publish_keypoints(
  const rclcpp::Publisher<PcdMsg>::SharedPtr& publisher,
  const std::vector<ORB_SLAM3::MapPoint*>& tracked_map_points,
  const std::vector<cv::KeyPoint>& tracked_keypoints,
  const rclcpp::Time& msg_time,
  const std::string& world_frame_id
)
{
  if (tracked_keypoints.empty()) return;

  std::vector<cv::KeyPoint> finalKeypoints;
  finalKeypoints.reserve(tracked_keypoints.size());

  for (size_t i = 0; i < tracked_map_points.size() && i < tracked_keypoints.size(); ++i) {
    if (tracked_map_points[i]) finalKeypoints.push_back(tracked_keypoints[i]);
  }

  sensor_msgs::msg::PointCloud2 cloud = keypoints_to_pointcloud(finalKeypoints, msg_time, world_frame_id);
  publisher->publish(cloud);
}

//////////////////// Conversions & helpers ////////////////////

sensor_msgs::msg::PointCloud2 keypoints_to_pointcloud(
  std::vector<cv::KeyPoint>& keypoints,
  const rclcpp::Time& msg_time, 
  const std::string& world_frame_id)
{
  const int num_channels = 3; // x y z

  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.stamp = msg_time;
  cloud.header.frame_id = world_frame_id;
  cloud.height = 1;
  cloud.width  = static_cast<uint32_t>(keypoints.size());
  cloud.is_bigendian = false;
  cloud.is_dense = true;
  cloud.point_step = num_channels * sizeof(float);
  cloud.row_step   = cloud.point_step * cloud.width;
  cloud.fields.resize(num_channels);

  const char* channel_id[] = {"x","y","z"};
  for (int i=0;i<num_channels;++i) {
    cloud.fields[i].name   = channel_id[i];
    cloud.fields[i].offset = i * sizeof(float);
    cloud.fields[i].count  = 1;
    cloud.fields[i].datatype = sensor_msgs::msg::PointField::FLOAT32;
  }

  cloud.data.resize(cloud.row_step * cloud.height);
  unsigned char* ptr = cloud.data.data();

  for (uint32_t i=0;i<cloud.width;++i) {
    float data_array[num_channels] = {
      keypoints[i].pt.x,
      keypoints[i].pt.y,
      0.0f
    };
    std::memcpy(ptr + (i * cloud.point_step), data_array, sizeof(data_array));
  }
  return cloud;
}

sensor_msgs::msg::PointCloud2 mappoint_to_pointcloud(
  const std::vector<ORB_SLAM3::MapPoint*>& map_points,
   const rclcpp::Time& msg_time,
    const std::string& world_frame_id)
{
  const int num_channels = 3;

  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.stamp = msg_time;
  cloud.header.frame_id = world_frame_id;
  cloud.height = 1;
  cloud.width  = static_cast<uint32_t>(map_points.size());
  cloud.is_bigendian = false;
  cloud.is_dense = true;
  cloud.point_step = num_channels * sizeof(float);
  cloud.row_step   = cloud.point_step * cloud.width;
  cloud.fields.resize(num_channels);

  const char* channel_id[] = {"x","y","z"};
  for (int i=0;i<num_channels;++i) {
    cloud.fields[i].name   = channel_id[i];
    cloud.fields[i].offset = i * sizeof(float);
    cloud.fields[i].count  = 1;
    cloud.fields[i].datatype = sensor_msgs::msg::PointField::FLOAT32;
  }

  cloud.data.resize(cloud.row_step * cloud.height);
  unsigned char* ptr = cloud.data.data();

  for (uint32_t i=0;i<cloud.width;++i) {
    if (map_points[i]) {
      Eigen::Vector3d P3Dw = map_points[i]->GetWorldPos().cast<double>();
      float data_array[num_channels] = {
        static_cast<float>(P3Dw.x()),
        static_cast<float>(P3Dw.y()),
        static_cast<float>(P3Dw.z())
      };
      std::memcpy(ptr + (i * cloud.point_step), data_array, sizeof(data_array));
    } else {
      const float nanv = std::numeric_limits<float>::quiet_NaN();
      float data_array[num_channels] = {nanv, nanv, nanv};
      std::memcpy(ptr + (i * cloud.point_step), data_array, sizeof(data_array));
      cloud.is_dense = false;
    }
  }
  return cloud;
}

void publish_tracked_points(
  const rclcpp::Publisher<PcdMsg>::SharedPtr& publisher,
  const std::vector<ORB_SLAM3::MapPoint*>& tracked_points,
  const rclcpp::Time& msg_time,
  const std::string& world_frame_id
)
{
  auto cloud = mappoint_to_pointcloud(tracked_points, msg_time, world_frame_id);
  publisher->publish(cloud);
}

void publish_all_points(
  const rclcpp::Publisher<PcdMsg>::SharedPtr& publisher,
  const std::vector<ORB_SLAM3::MapPoint*>& map_points,
  const rclcpp::Time& msg_time,
  const std::string& world_frame_id)
{
  auto cloud = mappoint_to_pointcloud(map_points, msg_time, world_frame_id);
  publisher->publish(cloud);
}

void publish_kf_markers(
  const rclcpp::Publisher<MarkerMsg>::SharedPtr& publisher,
  const std::vector<Sophus::SE3f>& vKFposes,
  const rclcpp::Time& msg_time,
  const std::string& world_frame_id)
{
  const int numKFs = static_cast<int>(vKFposes.size());
  if (numKFs == 0) return;

  visualization_msgs::msg::Marker kf_markers;
  kf_markers.header.frame_id = world_frame_id;
  kf_markers.header.stamp    = msg_time;
  kf_markers.ns   = "kf_markers";
  kf_markers.type = visualization_msgs::msg::Marker::SPHERE_LIST;
  kf_markers.action = visualization_msgs::msg::Marker::ADD;
  kf_markers.pose.orientation.w = 1.0;
  kf_markers.lifetime = rclcpp::Duration(0,0);

  kf_markers.id = 0;
  kf_markers.scale.x = 0.1;
  kf_markers.scale.y = 0.1;
  kf_markers.scale.z = 0.1;
  kf_markers.color.g = 1.0;
  kf_markers.color.a = 1.0;

  kf_markers.points.reserve(numKFs);
  for (int i = 0; i < numKFs; ++i) {
    geometry_msgs::msg::Point p;
    p.x = vKFposes[i].translation().x();
    p.y = vKFposes[i].translation().y();
    p.z = vKFposes[i].translation().z();
    kf_markers.points.push_back(p);
  }
  
  publisher->publish(kf_markers);
}

Eigen::Affine3f transform_to_eigen(
  const geometry_msgs::msg::TransformStamped& transform_stamped){
    Eigen::Vector3f translation(
      transform_stamped.transform.translation.x, 
      transform_stamped.transform.translation.y, 
      transform_stamped.transform.translation.z
    ); 

    Eigen::Quaternionf quat(
      transform_stamped.transform.rotation.w, 
      transform_stamped.transform.rotation.x, 
      transform_stamped.transform.rotation.y, 
      transform_stamped.transform.rotation.z
    ); 
    
    Eigen::Translation<float, 3> trans(translation); 
    Eigen::Matrix3f rot = quat.toRotationMatrix(); 
    Eigen::Affine3f affine = trans * rot; 

    return affine; 
  }


Sophus::SE3f transform_to_SE3(
  const geometry_msgs::msg::TransformStamped& transform_stamped){

    Eigen::Vector3f translation(
      transform_stamped.transform.translation.x, 
      transform_stamped.transform.translation.y, 
      transform_stamped.transform.translation.z
    ); 

    Eigen::Quaternionf quat(
      transform_stamped.transform.rotation.w, 
      transform_stamped.transform.rotation.x, 
      transform_stamped.transform.rotation.y, 
      transform_stamped.transform.rotation.z
    ); 

    Eigen::Matrix3f rotation = quat.toRotationMatrix(); 
    
    Sophus::SE3f se3(rotation, translation);

    return se3;
  }