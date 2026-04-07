#include "monocular-inertial-node.hpp"
#include <eigen3/Eigen/Dense>

#include <opencv2/core/core.hpp>

using std::placeholders::_1;

MonocularInertialNode::MonocularInertialNode(ORB_SLAM3::System* pSLAM)
:   Node("ORB_SLAM3_ROS2")
{
    this->declare_parameter("image", "/image");
    
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
    
    m_image_subscriber = image_transport::create_camera_subscription(
	this,
        image_topic.c_str(),
        std::bind(&MonocularInertialNode::GrabImage, this, std::placeholders::_1), "raw", qos_custom_profile);
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

void MonocularInertialNode::GrabImage(sensor_msgs::ImageMsg::ConstSharedPtr msg)
{
    bufMutexImg_.lock();

    if (!imgBuf_.empty())
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
        std::unique_lock<std::mutex> img_lock(bufMutexImg_, std::defer_lock);
        std::unique_lock<std::mutex> imu_lock(bufMutex_, std::defer_lock);

        std::lock(img_lock, imu_lock);

        if (!imgBuf_.empty() && !imuBuf_.empty()) {
            auto imgPtr = imgBuf_.front();
            double tImage = Utility::StampToSec(imgPtr->header.stamp);
            // double tImageshort = fmod(tImage, 100);

            cv::Mat imageFrame = GetImage(imgPtr); // Process image before popping
            vector<ORB_SLAM3::IMU::Point> vImuMeas;
            std::stringstream imu_data_stream;

            while (!imuBuf_.empty() && Utility::StampToSec(imuBuf_.front()->header.stamp) <= tImage) {
                auto imuPtr = imuBuf_.front();
                double tIMU = Utility::StampToSec(imuPtr->header.stamp);
                // double tIMUshort = fmod(tIMU, 100);

                imuBuf_.pop();
                cv::Point3f acc(imuPtr->linear_acceleration.x, imuPtr->linear_acceleration.y, imuPtr->linear_acceleration.z);
                cv::Point3f gyr(imuPtr->angular_velocity.x, imuPtr->angular_velocity.y, imuPtr->angular_velocity.z);
                vImuMeas.push_back(ORB_SLAM3::IMU::Point(acc, gyr, tIMU));

                // Debug info
                // imu_data_stream << "IMU at " << std::fixed << std::setprecision(6) << tIMUshort << " - Acc: [" << acc << "], Gyr: [" << gyr << "]\n";
            }

            imgBuf_.pop(); // Safely pop the image from the buffer here

            if (vImuMeas.empty()) {
                RCLCPP_WARN(this->get_logger(), "No valid IMU data available for the current frame at time %.6f.", tImage);
                continue; // Skip processing this frame
            }

            try {

                cv::Mat img;
                if (do_rectify) {
                    cv::remap(cv_ptr->image,img,M1l,M2l,cv::INTER_LINEAR);

                }

                Sophus::SE3f Tcw = m_SLAM->TrackMonocular(imageFrame, tImage, vImuMeas);
                // Sophus::SE3f Tcw = m_SLAM->TrackStereo(imLeft, imRight, Utility::StampToSec(msgLeft->header.stamp));
                if(!Tcw.translation().isZero()) {
                    // Angles for rotation matrix (from optical frame to FLU)
                    Eigen::Matrix3f R;
                    R  <<
                        0,  0,  1,
                        -1,  0,  0,
                        0, -1,  0;
                    Eigen::Quaternionf q_opt_flu(R);
                    Eigen::Vector3f t_flu = R * Tcw.translation();
                    nav_msgs::msg::Odometry odom_msg;

                    odom_msg.header.stamp = this->now();
                    odom_msg.header.frame_id = "map";
                    odom_msg.child_frame_id = "base_link";

                    // Position vector converted to FLU from optical frame
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
                // RCLCPP_INFO(this->get_logger(), "Image at %.6f processed with IMU data: \n%s", tImageshort, imu_data_stream.str().c_str());
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "SLAM processing exception: %s", e.what());
            }
        }

        img_lock.unlock();
        imu_lock.unlock();

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

