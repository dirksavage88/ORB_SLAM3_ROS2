#include "monocular-inertial-node.hpp"
#include <eigen3/Eigen/Dense>

#include <opencv2/core/core.hpp>

using std::placeholders::_1;
using ImuMsg = sensor_msgs::msg::Imu;
using ImageMsg = sensor_msgs::msg::Image;

MonocularInertialNode::MonocularInertialNode(ORB_SLAM3::System* pSLAM)
:   Node("ORB_SLAM3_ROS2")
{
    this->declare_parameter("image", "/image");
    this->declare_parameter("imu", "/imu");
    
    m_SLAM = pSLAM;

    // QoS 
    rmw_qos_profile_t qos_custom_profile = rmw_qos_profile_system_default;
    image_topic = this->get_parameter("image").as_string();
    imu_topic = this->get_parameter("imu").as_string();

    // if (do_rectify)
    // {
        // // Load settings related to stereo calibration
        // cv::FileStorage fsSettings(strSettingsFile, cv::FileStorage::READ);
        // if (!fsSettings.isOpened())
        // {
        //     cerr << "ERROR: Wrong path to settings" << endl;
        //     assert(0);
        // }
        //
        // cv::Mat K_l, K_r, P_l, P_r, R_l, R_r, D_l, D_r;
        // fsSettings["LEFT.K"] >> K_l;
        //
        // fsSettings["LEFT.P"] >> P_l;
        //
        // fsSettings["LEFT.R"] >> R_l;
        //
        // fsSettings["LEFT.D"] >> D_l;
        //
        // int rows = fsSettings["LEFT.height"];
        // int cols = fsSettings["LEFT.width"];
        //
        // if (K_l.empty() || K_r.empty() || P_l.empty() || P_r.empty() || R_l.empty() || R_r.empty() || D_l.empty() || D_r.empty() ||
        //     rows_l == 0 || rows_r == 0 || cols_l == 0 || cols_r == 0)
        // {
        //     cerr << "ERROR: Calibration parameters to rectify stereo are missing!" << endl;
        //     assert(0);
        // }
        // cv::initUndistortRectifyMap(K_l, D_l, R_l, P_l.rowRange(0, 3).colRange(0, 3), cv::Size(cols_l, rows_l), CV_32F, M1l_, M2l_);
    // }
    
    m_image_subscriber = this->create_subscription<ImageMsg>(image_topic.c_str(), 10, std::bind(&MonocularInertialNode::GrabImage, this, _1));
    std::cout << "slam changed" << std::endl;

    subImu_ = this->create_subscription<ImuMsg>(
        imu_topic.c_str(), 1000, std::bind(&MonocularInertialNode::GrabImu, this, _1));

    syncThread_ = new std::thread(&MonocularInertialNode::SyncWithImu, this);
    
    _odom_pub = this->create_publisher<nav_msgs::msg::Odometry>("/visual_slam/tracking/odometry", 10);
    
    std::cout << "System Initialization Complete" << std::endl;
}

MonocularInertialNode::~MonocularInertialNode()
{
//    if (syncThread_->joinable()) {
    syncThread_->join();
//    }
    delete syncThread_;

    m_SLAM->Shutdown();
    m_SLAM->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
}

void MonocularInertialNode::GrabImu(const ImuMsg::SharedPtr msg)
{
    if (!std::isnan(msg->linear_acceleration.x) && !std::isnan(msg->linear_acceleration.y) &&
        !std::isnan(msg->linear_acceleration.z) && !std::isnan(msg->angular_velocity.x) &&
        !std::isnan(msg->angular_velocity.y) && !std::isnan(msg->angular_velocity.z))
    {
        bufMutex_.lock();
        imuBuf_.push(msg);
        bufMutex_.unlock();
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "Invalid IMU data - Rxd NaN");
    }
}

void MonocularInertialNode::GrabImage(sensor_msgs::msg::Image::SharedPtr msg)
{
    bufMutexImg_.lock();

    if(!imgBuf_.empty())
    	imgBuf_.pop();

    imgBuf_.push(msg);

    bufMutexImg_.unlock();
    
}

cv::Mat MonocularInertialNode::GetImage(const ImageMsg::SharedPtr msg)
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

void MonocularInertialNode::SyncWithImu()
{
    while (rclcpp::ok()) {
        cv::Mat imageFrame;
        double tImage = 0.0;
        double tPrevFrameSnapshot = 0.0;
        std::vector<ORB_SLAM3::IMU::Point> vImuMeas;
        bool hasData = false;

        {
            std::unique_lock<std::mutex> img_lock(bufMutexImg_);
            std::unique_lock<std::mutex> imu_lock(bufMutex_);

            if (!imgBuf_.empty() && !imuBuf_.empty()) {
                auto imgPtr = imgBuf_.front();
                tImage = Utility::StampToSec(imgPtr->header.stamp);
                imageFrame = GetImage(imgPtr);
                imgBuf_.pop();

                if (tPrevFrame < 0.0)
                    tPrevFrame = tImage;

                tPrevFrameSnapshot = tPrevFrame;

                while (!imuBuf_.empty()) {
                    double tIMU = Utility::StampToSec(imuBuf_.front()->header.stamp);
                    if (tIMU > tImage)
                        break;
                    if (tIMU > tPrevFrame) {
                        auto& m = imuBuf_.front();
                        vImuMeas.emplace_back(
                            cv::Point3f(m->linear_acceleration.x,
                                        m->linear_acceleration.y,
                                        m->linear_acceleration.z),
                            cv::Point3f(m->angular_velocity.x,
                                        m->angular_velocity.y,
                                        m->angular_velocity.z),
                            tIMU);
                    }
                    imuBuf_.pop();
                }

                tPrevFrame = tImage;
                hasData = true;
            }
        } // locks released — GrabImu/GrabImage can run during TrackMonocular

        if (!hasData) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }

        if (vImuMeas.empty()) {
            RCLCPP_WARN(this->get_logger(),
                "No valid IMU data in window (%.6f -> %.6f).", tPrevFrameSnapshot, tImage);
        }

        try {
            Sophus::SE3f Tcw = m_SLAM->TrackMonocular(imageFrame, tImage, vImuMeas);
            if (!Tcw.translation().isZero()) {
                Eigen::Matrix3f R;
                R <<  0,  0,  1,
                     -1,  0,  0,
                      0, -1,  0;
                Eigen::Quaternionf q_opt_flu(R);
                Eigen::Vector3f t_flu = R * Tcw.translation();

                nav_msgs::msg::Odometry odom_msg;
                odom_msg.header.stamp = this->now();
                odom_msg.header.frame_id = "map";
                odom_msg.child_frame_id = "base_link";
                odom_msg.pose.pose.position.x = t_flu.x();
                odom_msg.pose.pose.position.y = t_flu.y();
                odom_msg.pose.pose.position.z = t_flu.z();

                Eigen::Quaternionf q_cam(Tcw.unit_quaternion());
                Eigen::Quaternionf q_world_flu = q_cam * q_opt_flu;
                odom_msg.pose.pose.orientation.x = q_world_flu.x();
                odom_msg.pose.pose.orientation.y = q_world_flu.y();
                odom_msg.pose.pose.orientation.z = q_world_flu.z();
                odom_msg.pose.pose.orientation.w = q_world_flu.w();

                _odom_pub->publish(odom_msg);
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "SLAM processing exception: %s", e.what());
        }
    }
}

