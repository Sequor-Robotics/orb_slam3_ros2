#include "stereo-inertial-node.hpp"

#include <opencv2/core/core.hpp>
#include <ros_utils.hpp>

using std::placeholders::_1;

StereoInertialNode::StereoInertialNode(ORB_SLAM3::System *SLAM, const string &strSettingsFile, const string &strDoRectify, const string &strDoEqual) :
    Node("ORB_SLAM3_ROS2", rclcpp::NodeOptions()
                                   .start_parameter_services(false)        // 파라미터 서비스 끔
                                   .start_parameter_event_publisher(false) // 파라미터 이벤트 퍼블리셔 끔
                                   .automatically_declare_parameters_from_overrides(false)), // 필요 시),
    SLAM_(SLAM)
{
    stringstream ss_rec(strDoRectify);
    ss_rec >> boolalpha >> doRectify_;

    stringstream ss_eq(strDoEqual);
    ss_eq >> boolalpha >> doEqual_;

    bClahe_ = doEqual_;
    std::cout << "Rectify: " << doRectify_ << std::endl;
    std::cout << "Equal: " << doEqual_ << std::endl;

    // ===== Added: Downsample parameters =====
    double ds_scale = 1.0; // default if not present
    {
        cv::FileStorage fsSettings(strSettingsFile, cv::FileStorage::READ);
        if(!fsSettings.isOpened())
        {
            std::cerr << "ERROR: Wrong path to settings: " << strSettingsFile << std::endl;
            assert(0);
        }

        // Prefer lowercase key as requested, but accept common variants.
        cv::FileNode cam = fsSettings["ImageScale"];
        if (!cam.empty())
        {
            ds_scale = static_cast<double>(cam);
        }

        if (ds_scale <= 0.0 || ds_scale > 1.0) {
            RCLCPP_ERROR(this->get_logger(),
                        "[ImageScale=%.3f] out of range.", 
                        ds_scale);
            cerr << "ImageScale out of range!" << endl; 
        } else {
            RCLCPP_INFO(this->get_logger(),
                        "Loaded ImageScale=%.3f from %s",
                        ds_scale, strSettingsFile.c_str());
        }
    }

    // ===== Updated: Downsample parameters =====
    // Use YAML value as the default; allow ROS param override from launch.py if provided.
    this->declare_parameter<double>("downsample.scale", ds_scale);
    this->declare_parameter<std::string>("downsample.method", "auto");
    ds_scale_  = this->get_parameter("downsample.scale").as_double();
    ds_method_ = this->get_parameter("downsample.method").as_string();

    if (ds_method_ != "auto" && ds_method_ != "area" && ds_method_ != "pyr") {
        RCLCPP_WARN(this->get_logger(),
                    "[downsample.method=%s] invalid. Use auto/area/pyr. Using auto.",
                    ds_method_.c_str());
        ds_method_ = "auto";
    }

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


// === Added: Downsample helper ===
namespace {
inline bool approx_equal(double a, double b, double eps = 1e-6) {
    return std::abs(a - b) <= eps * std::max(1.0, std::max(std::abs(a), std::abs(b)));
}
}

cv::Mat StereoInertialNode::DownsampleIfNeeded(const cv::Mat &src) const
{
    if (ds_scale_ >= 0.999) {
        return src.clone(); // no downsample
    }

    // Forced method
    if (ds_method_ == "area") {
        cv::Mat dst;
        cv::resize(src, dst, cv::Size(), ds_scale_, ds_scale_, cv::INTER_AREA);
        return dst;
    }

    // pyr or auto
    double inv = 1.0 / ds_scale_;
    double log2_inv = std::log2(inv);
    double r = std::round(log2_inv);
    bool near_pow2 = std::abs(log2_inv - r) < 1e-6 && (r >= 1.0);

    cv::Mat cur = src;
    int pyr_times = 0;

    // perform only one pyrDown regardless of ds_scale_
    if (ds_method_ == "pyr" || (ds_method_ == "auto" && near_pow2)) {
        cv::Mat tmp;
        cv::pyrDown(cur, tmp);  // one-time downsample by 1/2
        cur = std::move(tmp);

        // if ds_scale_ != 0.5, adjust slightly using INTER_AREA
        if (!approx_equal(ds_scale_, 0.5)) {
            double residual = ds_scale_ / 0.5;  // adjust relative to half-size
            cv::resize(cur, cur, cv::Size(), residual, residual, cv::INTER_AREA);
        }
        return cur.clone();
    }

    cv::Mat dst;
    cv::resize(src, dst, cv::Size(), ds_scale_, ds_scale_, cv::INTER_AREA);
    return dst;
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
        return cv::Mat();
    }

    if (cv_ptr->image.type() != CV_8UC1) {
        std::cerr << "Error image type (expect mono8). Got type=" << cv_ptr->image.type() << std::endl;
        return cv_ptr->image.clone();
    }

    // === Downsample here ===
    cv::Mat out = DownsampleIfNeeded(cv_ptr->image);
    return out;
}

void StereoInertialNode::SyncWithImu()
{
    // === 튜너블 파라미터(원하면 ROS param으로 뺄 것) ===
    const double warmupSec    = this->has_parameter("warmup_sec")
                                ? this->get_parameter("warmup_sec").as_double()
                                : 1.0;             // 첫 프레임에서 통합할 사전 IMU 구간 길이
    const int    minImuCount  = this->has_parameter("min_imu_count")
                                ? this->get_parameter("min_imu_count").as_int()
                                : 2;               // 한 프레임당 최소 IMU 샘플 수(경험치)

    // 직전에 처리한 좌이미지 시각(초) - 첫 회에는 NaN
    static double last_img_ts = std::numeric_limits<double>::quiet_NaN();

    while (1)
    {
        RCLCPP_INFO_ONCE(this->get_logger(), "SLAM running...");

        // --- 버퍼 상태 확인(락 최소화) ---
        bool hasLeft = false, hasRight = false, hasImu = false;
        {
            std::lock_guard<std::mutex> lkL(bufMutexLeft_);
            hasLeft = !imgLeftBuf_.empty();
        }
        {
            std::lock_guard<std::mutex> lkR(bufMutexRight_);
            hasRight = !imgRightBuf_.empty();
        }
        {
            std::lock_guard<std::mutex> lkI(bufMutex_);
            hasImu = !imuBuf_.empty();
        }

        if (!(hasLeft && hasRight && hasImu)) {
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
            continue;
        }

        // --- 좌/우 이미지 헤더 스탬프로 동기 ---
        ImageMsg::SharedPtr left_msg, right_msg;
        double tImLeft = 0.0, tImRight = 0.0;

        // 좌/우 각각 프런트만 '훑어보고' 시간차를 줄이기 위해 오래된 쪽을 pop
        {
            std::lock_guard<std::mutex> lkL(bufMutexLeft_);
            if (imgLeftBuf_.empty()) continue;
            left_msg = imgLeftBuf_.front();
        }
        {
            std::lock_guard<std::mutex> lkR(bufMutexRight_);
            if (imgRightBuf_.empty()) continue;
            right_msg = imgRightBuf_.front();
        }

        tImLeft  = Utility::StampToSec(left_msg->header.stamp);
        tImRight = Utility::StampToSec(right_msg->header.stamp);


        RCLCPP_INFO_ONCE(this->get_logger(), "Grab Image");

        // --- 최신 IMU 시간 확인: 이미지가 IMU보다 너무 앞서면 기다림 ---
        double latest_imu_ts = 0.0;
        {
            std::lock_guard<std::mutex> lk(bufMutex_);
            if (!imuBuf_.empty())
                latest_imu_ts = Utility::StampToSec(imuBuf_.back()->header.stamp);
        }
        // --- 이제 실제로 동일한 프레임을 소비 (ABA 문제 방지) ---
        cv::Mat imLeft, imRight;
        {
            std::lock_guard<std::mutex> lkL(bufMutexLeft_);
            if (imgLeftBuf_.empty() || imgLeftBuf_.front() != left_msg) continue;
            imLeft = GetImage(left_msg);
            imgLeftBuf_.pop();
        }
        {
            std::lock_guard<std::mutex> lkR(bufMutexRight_);
            if (imgRightBuf_.empty() || imgRightBuf_.front() != right_msg) continue;
            imRight = GetImage(right_msg);
            imgRightBuf_.pop();
        }

        // 입력 이미지 유효성 검사
        if (imLeft.empty() || imRight.empty()) {
            RCLCPP_WARN(this->get_logger(), "Empty image(s). Skip this frame.");
            continue;
        }

        // --- IMU 적분 구간: [last_img_ts, tImLeft] ---
        double tStart = std::isnan(last_img_ts) ? (tImLeft - warmupSec) : last_img_ts;
        if (tStart > tImLeft) tStart = tImLeft; // 안전

        std::vector<ORB_SLAM3::IMU::Point> vImuMeas;
        Eigen::Vector3f Wbb;
        {
            std::lock_guard<std::mutex> lk(bufMutex_);
            RCLCPP_INFO_ONCE(this->get_logger(), "Grab Imu");

            // tStart 이전 IMU는 폐기(버퍼 누적 방지)
            while (!imuBuf_.empty() &&
                   Utility::StampToSec(imuBuf_.front()->header.stamp) < tStart) {
                imuBuf_.pop();
            }

            // [tStart, tImLeft] 구간의 IMU 수집
            vImuMeas.clear();
            while (!imuBuf_.empty()) {
                double t = Utility::StampToSec(imuBuf_.front()->header.stamp);
                if (t > tImLeft) break;
                const auto &msg = imuBuf_.front();
                cv::Point3f acc(msg->linear_acceleration.x,
                                msg->linear_acceleration.y,
                                msg->linear_acceleration.z);
                cv::Point3f gyr(msg->angular_velocity.x,
                                msg->angular_velocity.y,
                                msg->angular_velocity.z);
                vImuMeas.emplace_back(acc, gyr, t);
                Wbb = Eigen::Vector3f(msg->angular_velocity.x,
                                      msg->angular_velocity.y,
                                      msg->angular_velocity.z);
                imuBuf_.pop();
            }
        }

        // 방어: 빈 IMU면 이번 프레임 스킵(세그폴트 방지)
        if (vImuMeas.size() < static_cast<size_t>(minImuCount)) {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                            "IMU vector too small (%zu < %d). Skip this frame.",
                            vImuMeas.size(), minImuCount);
            // 이미지 FPS가 너무 높거나, IMU QoS/지연 문제 가능성 → 상위 가이드 참조
            last_img_ts = tImLeft; // 그래도 진행 표시(다음 윈도우를 좁힘)
            continue;
        }

        // --- 이미지 전처리(유효성 통과 후) ---
        if (bClahe_) {
            if (!clahe_) clahe_ = cv::createCLAHE(3.0, cv::Size(8, 8));
            clahe_->apply(imLeft, imLeft);
            clahe_->apply(imRight, imRight);
        }
        if (doRectify_) {
            cv::remap(imLeft,  imLeft,  M1l_, M2l_, cv::INTER_LINEAR);
            cv::remap(imRight, imRight, M1r_, M2r_, cv::INTER_LINEAR);
        }

        // --- SLAM 추적 ---
        const rclcpp::Time stamp_left = left_msg->header.stamp;
        Sophus::SE3f Tcw = SLAM_->TrackStereo(imLeft, imRight, tImLeft, vImuMeas);
        Sophus::SE3f Twc = Tcw.inverse();

        // 좌표계 변환 (OpenCV->ROS FLU)
        Eigen::Matrix<float,3,3> cv_to_ros_rot;
        Eigen::Matrix<float,3,1> cv_to_ros_trans;
        cv_to_ros_rot << 0, 0, 1,
                        -1, 0, 0,
                         0,-1, 0;
        cv_to_ros_trans << 0, 0, 0;
        Sophus::SE3f cv_to_ros(cv_to_ros_rot, cv_to_ros_trans);
        Twc = Twc * cv_to_ros.inverse();

        // Publish
        const std::string world_frame = this->get_parameter("world_frame").as_string();
        const std::string odom_frame  = this->get_parameter("odom_frame").as_string();
        const std::string body_frame  = this->get_parameter("body_frame").as_string();

        publish_camera_tf   (tf_broadcaster_, stamp_left, Twc, world_frame, body_frame);
        publish_camera_pose (pubPose_,       stamp_left, Twc, world_frame);
        publish_tracking_img(pubTrackImage_, stamp_left, SLAM_->GetCurrentFrame(), world_frame);
        publish_tracked_points(pubTrackedPoints_, SLAM_->GetTrackedMapPoints(), stamp_left, world_frame);
        publish_all_points    (pubAllPoints_,    SLAM_->GetAllMapPoints(),       stamp_left, world_frame);
        publish_kf_markers    (pubKFMarkers_,    SLAM_->GetAllKeyframePoses(),   stamp_left, world_frame);

        // 다음 루프를 위한 기준 시각 갱신
        last_img_ts = tImLeft;
    }
}

