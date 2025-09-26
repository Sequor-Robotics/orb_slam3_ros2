#include "stereo-inertial-node.hpp"

#include <opencv2/core/core.hpp>
#include <ros_utils.hpp>

using std::placeholders::_1;

StereoInertialNode::StereoInertialNode(ORB_SLAM3::System *SLAM, const string &strSettingsFile, const string &strDoRectify, const string &strDoEqual) :
    Node("ORB_SLAM3_ROS2"),
    SLAM_(SLAM)
{
    stringstream ss_rec(strDoRectify);
    ss_rec >> boolalpha >> doRectify_;

    stringstream ss_eq(strDoEqual);
    ss_eq >> boolalpha >> doEqual_;

    bClahe_ = doEqual_;
    std::cout << "Rectify: " << doRectify_ << std::endl;
    std::cout << "Equal: " << doEqual_ << std::endl;

    if (doRectify_)
    {
        // Load settings related to stereo calibration
        cv::FileStorage fsSettings(strSettingsFile, cv::FileStorage::READ);
        if(!fsSettings.isOpened())
        {
            cerr << "ERROR: Wrong path to settings" << endl;
            assert(0);
        }

        cv::Mat K_l, K_r, P_l, P_r, R_l, R_r, D_l, D_r;
        fsSettings["LEFT.K"] >> K_l;
        fsSettings["RIGHT.K"] >> K_r;

        fsSettings["LEFT.P"] >> P_l;
        fsSettings["RIGHT.P"] >> P_r;

        fsSettings["LEFT.R"] >> R_l;
        fsSettings["RIGHT.R"] >> R_r;

        fsSettings["LEFT.D"] >> D_l;
        fsSettings["RIGHT.D"] >> D_r;

        int rows_l = fsSettings["LEFT.height"];
        int cols_l = fsSettings["LEFT.width"];
        int rows_r = fsSettings["RIGHT.height"];
        int cols_r = fsSettings["RIGHT.width"];

        if (K_l.empty() || K_r.empty() || P_l.empty() || P_r.empty() || R_l.empty() || R_r.empty() || D_l.empty() || D_r.empty() ||
            rows_l == 0 || rows_r == 0 || cols_l == 0 || cols_r == 0)
        {
            cerr << "ERROR: Calibration parameters to rectify stereo are missing!" << endl;
            assert(0);
        }

        cv::initUndistortRectifyMap(K_l, D_l, R_l, P_l.rowRange(0, 3).colRange(0, 3), cv::Size(cols_l, rows_l), CV_32F, M1l_, M2l_);
        cv::initUndistortRectifyMap(K_r, D_r, R_r, P_r.rowRange(0, 3).colRange(0, 3), cv::Size(cols_r, rows_r), CV_32F, M1r_, M2r_);
    }

    /// without qos setting, imu is not subscribed by StereoInertialNode
    rclcpp::QoS imu_qos(10);
    imu_qos.best_effort();
    imu_qos.durability_volatile();
    //////////////////////////////////////// 
    // TODO: Add topic as ros2 parameter
    subImu_ = this->create_subscription<ImuMsg>("imu", imu_qos, std::bind(&StereoInertialNode::GrabImu, this, std::placeholders::_1));
    subImgLeft_ = this->create_subscription<ImageMsg>("camera/left", 10, std::bind(&StereoInertialNode::GrabImageLeft, this, std::placeholders::_1));
    subImgRight_ = this->create_subscription<ImageMsg>("camera/right", 10, std::bind(&StereoInertialNode::GrabImageRight, this, std::placeholders::_1));

    pubPose_ = this->create_publisher<PoseMsg>("camera_pose", 1);
    pubOdom_ = this->create_publisher<OdomMsg>("imu_odometry", 1);
    pubTrackImage_ = this->create_publisher<ImageMsg>("tracking_image", 1);
    pubTrackedKeypoints_ = this->create_publisher<PcdMsg>("tracked_keypoints", 1);
    pubTrackedPoints_ = this->create_publisher<PcdMsg>("tracked_mappoints", 1);
    pubAllPoints_ = this->create_publisher<PcdMsg>("all_mappoints", 1);
    pubKFMarkers_ = this->create_publisher<MarkerMsg>("kf_markers", 100);


    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
    tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // declare rosparameters
    this->declare_parameter("world_frame", "map");
    this->declare_parameter("odom_frame", "odom");
    this->declare_parameter("body_frame", "body_link");
    this->declare_parameter("body_optical_frame", "body_optical_link");
    this->declare_parameter("camera_optical_frame", "camera_optical_link");

    syncThread_ = new std::thread(&StereoInertialNode::SyncWithImu, this);
}

StereoInertialNode::~StereoInertialNode()
{
    // Delete sync thread
    syncThread_->join();
    delete syncThread_;

    // Stop all threads
    SLAM_->Shutdown();

    // Save camera trajectory
    SLAM_->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
}

void StereoInertialNode::GrabImu(const ImuMsg::SharedPtr msg)
{
    bufMutex_.lock();
    imuBuf_.push(msg);
    bufMutex_.unlock();
}

void StereoInertialNode::GrabImageLeft(const ImageMsg::SharedPtr msgLeft)
{
    bufMutexLeft_.lock();

    if (!imgLeftBuf_.empty())
        imgLeftBuf_.pop();
    imgLeftBuf_.push(msgLeft);

    bufMutexLeft_.unlock();
}

void StereoInertialNode::GrabImageRight(const ImageMsg::SharedPtr msgRight)
{
    bufMutexRight_.lock();

    if (!imgRightBuf_.empty())
        imgRightBuf_.pop();
    imgRightBuf_.push(msgRight);

    bufMutexRight_.unlock();
}

cv::Mat StereoInertialNode::GetImage(const ImageMsg::SharedPtr msg)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr;

    try
    {
        cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::MONO8);
    }
    catch (cv_bridge::Exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    }

    if (cv_ptr->image.type() == 0)
    {
        return cv_ptr->image.clone();
    }
    else
    {
        std::cerr << "Error image type" << std::endl;
        return cv_ptr->image.clone();
    }
}

void StereoInertialNode::SyncWithImu()
{
    const double maxTimeDiff = 0.01;

    while (1)
    {
        RCLCPP_INFO_ONCE(this->get_logger(), "SLAM running...");
        cv::Mat imLeft, imRight;
        double tImLeft = 0, tImRight = 0;
        if (!imgLeftBuf_.empty() && !imgRightBuf_.empty() && !imuBuf_.empty())
        {
            RCLCPP_INFO_ONCE(this->get_logger(), "Grab Image");
            //tImLeft = Utility::StampToSec(imgLeftBuf_.front()->header.stamp);
            //tImRight = Utility::StampToSec(imgRightBuf_.front()->header.stamp);

            // Peek messages so we can reuse the exact stamp later
            ImageMsg::SharedPtr left_msg  = imgLeftBuf_.front();
            ImageMsg::SharedPtr right_msg = imgRightBuf_.front();
            tImLeft  = Utility::StampToSec(left_msg->header.stamp);
            tImRight = Utility::StampToSec(right_msg->header.stamp);

            bufMutexRight_.lock();
            while ((tImLeft - tImRight) > maxTimeDiff && imgRightBuf_.size() > 1)
            {
                imgRightBuf_.pop();
                //tImRight = Utility::StampToSec(imgRightBuf_.front()->header.stamp);
                right_msg = imgRightBuf_.front();
                tImRight = Utility::StampToSec(right_msg->header.stamp);
            }
            bufMutexRight_.unlock();

            bufMutexLeft_.lock();
            while ((tImRight - tImLeft) > maxTimeDiff && imgLeftBuf_.size() > 1)
            {
                imgLeftBuf_.pop();
                //tImLeft = Utility::StampToSec(imgLeftBuf_.front()->header.stamp);
                left_msg = imgLeftBuf_.front();
                tImLeft = Utility::StampToSec(left_msg->header.stamp);
            }
            bufMutexLeft_.unlock();

            if ((tImLeft - tImRight) > maxTimeDiff || (tImRight - tImLeft) > maxTimeDiff)
            {
                RCLCPP_INFO_ONCE(this->get_logger(), "big time difference");
                continue;
            }
            if (tImLeft > Utility::StampToSec(imuBuf_.back()->header.stamp))
                continue;

            // Use the exact subscribed timestamp from the (synced) left image
            const rclcpp::Time stamp_left = left_msg->header.stamp;

            bufMutexLeft_.lock();
            imLeft = GetImage(left_msg);
            imgLeftBuf_.pop();
            bufMutexLeft_.unlock();

            bufMutexRight_.lock();
            imRight = GetImage(right_msg);
            imgRightBuf_.pop();
            bufMutexRight_.unlock();

            vector<ORB_SLAM3::IMU::Point> vImuMeas;
            Eigen::Vector3f Wbb; // body angular velocity in body frame
            bufMutex_.lock();
            if (!imuBuf_.empty())
            {
                RCLCPP_INFO_ONCE(this->get_logger(), "Grab Imu");
                // Load imu measurements from buffer
                vImuMeas.clear();
                while (!imuBuf_.empty() && Utility::StampToSec(imuBuf_.front()->header.stamp) <= tImLeft)
                {
                    double t = Utility::StampToSec(imuBuf_.front()->header.stamp);
                    cv::Point3f acc(imuBuf_.front()->linear_acceleration.x, imuBuf_.front()->linear_acceleration.y, imuBuf_.front()->linear_acceleration.z);
                    cv::Point3f gyr(imuBuf_.front()->angular_velocity.x, imuBuf_.front()->angular_velocity.y, imuBuf_.front()->angular_velocity.z);
                    vImuMeas.push_back(ORB_SLAM3::IMU::Point(acc, gyr, t));
                    Wbb = Eigen::Vector3f(imuBuf_.front()->angular_velocity.x, imuBuf_.front()->angular_velocity.y, imuBuf_.front()->angular_velocity.z);
                    imuBuf_.pop();
                }
            }
            bufMutex_.unlock();

            if (bClahe_)
            {
                clahe_->apply(imLeft, imLeft);
                clahe_->apply(imRight, imRight);
            }

            if (doRectify_)
            {
                cv::remap(imLeft, imLeft, M1l_, M2l_, cv::INTER_LINEAR);
                cv::remap(imRight, imRight, M1r_, M2r_, cv::INTER_LINEAR);
            }
            
            // Transform of camera in  world frame
            Sophus::SE3f Tcw = SLAM_->TrackStereo(imLeft, imRight, tImLeft, vImuMeas);
            Sophus::SE3f Twc = Tcw.inverse(); // Twc is imu optical frame pose in ROS FLU map coordinate
            RCLCPP_INFO_ONCE(this->get_logger(), "Camera pose in world frame: ");

            // publish topics
            std::string world_frame = this->get_parameter("world_frame").as_string();
            std::string odom_frame = this->get_parameter("odom_frame").as_string();
            std::string body_frame = this->get_parameter("body_frame").as_string();
            std::string body_optical_frame = this->get_parameter("body_optical_frame").as_string(); // unused?
            std::string camera_optical_frame = this->get_parameter("camera_optical_frame").as_string(); // unused?
            
            RCLCPP_INFO_ONCE(this->get_logger(), "Publishing camera pose in world frame: ");
            RCLCPP_INFO_ONCE(this->get_logger(), world_frame.c_str());
            RCLCPP_INFO_ONCE(this->get_logger(), odom_frame.c_str());
            RCLCPP_INFO_ONCE(this->get_logger(), body_frame.c_str());
            
            // define coordinate transforms ///
            // OpenCV to ROS FLU coordinate transforms
            Eigen::Matrix<float, 3, 3> cv_to_ros_rot; 
            Eigen::Matrix<float, 3, 1> cv_to_ros_trans; 
            cv_to_ros_rot << 0, 0, 1,
                            -1, 0, 0,
                            0, -1, 0;
            cv_to_ros_trans << 0, 0, 0;
            Sophus::SE3f cv_to_ros(cv_to_ros_rot, cv_to_ros_trans);
            std::cout << cv_to_ros.matrix() << std::endl; 

            // coordiante transform
            Twc = Twc * cv_to_ros.inverse(); // imu frame pose in ROS FLU map coorinate

            // Option1: publish map to odom tf from SLAM and odom to camera from VIO 
            //// TF processing ////
            // try {               
            //     geometry_msgs::msg::TransformStamped camera_to_odom = tf_buffer_->lookupTransform(body_frame, odom_frame, tf2::TimePointZero);
            //     Sophus::SE3f Tco= transform_to_SE3(camera_to_odom);
            //     Sophus::SE3f Two = Twc * Tco.inverse();
            //     publish_world_to_odom_tf(tf_broadcaster_, stamp_left, Two, world_frame, odom_frame);
            // } catch (const tf2::TransformException & ex) {
            //     RCLCPP_INFO(
            //     this->get_logger(), "Could not get transform %s to %s: %s",
            //     body_frame.c_str(), odom_frame.c_str(), ex.what());
            //     return;
            // }

            // Option2: publish map to camera tf from SLAM
            publish_camera_tf(tf_broadcaster_, stamp_left, Twc, world_frame, body_frame);
            publish_camera_pose(pubPose_, stamp_left, Twc, world_frame);
            publish_tracking_img(pubTrackImage_, stamp_left, SLAM_->GetCurrentFrame(), world_frame);
            //publish_keypoints(pubTrackedKeypoints_, SLAM_->GetTrackedMapPoints(), SLAM_->GetTrackedKeyPoints(), stamp_left, world_frame);
            publish_tracked_points(pubTrackedPoints_, SLAM_->GetTrackedMapPoints(), stamp_left, world_frame);
            publish_all_points(pubAllPoints_, SLAM_->GetAllMapPoints(), stamp_left, world_frame);
            publish_kf_markers(pubKFMarkers_, SLAM_->GetAllKeyframePoses(), stamp_left, world_frame);

            //publishKeyframeOutputs_(stamp_left);

            // std::chrono::milliseconds tSleep(1);
            // std::this_thread::sleep_for(tSleep);
        }
    }
}
