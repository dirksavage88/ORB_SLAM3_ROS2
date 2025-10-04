#include "stereo-slam-node.hpp"
#include <eigen3/Eigen/Dense>
#include<opencv2/core/core.hpp>

using std::placeholders::_1;
using std::placeholders::_2;

StereoSlamNode::StereoSlamNode(ORB_SLAM3::System* pSLAM, const string &strSettingsFile, const string &strDoRectify)
:   Node("ORB_SLAM3_ROS2"),
    m_SLAM(pSLAM)
{
    stringstream ss(strDoRectify);
    ss >> boolalpha >> doRectify;
    _odom_pub = this->create_publisher<nav_msgs::msg::Odometry>("/visual_slam/tracking/odometry", 10);

    if (doRectify){

        cv::FileStorage fsSettings(strSettingsFile, cv::FileStorage::READ);
        if(!fsSettings.isOpened()){
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

        if(K_l.empty() || K_r.empty() || P_l.empty() || P_r.empty() || R_l.empty() || R_r.empty() || D_l.empty() || D_r.empty() ||
                rows_l==0 || rows_r==0 || cols_l==0 || cols_r==0){
            cerr << "ERROR: Calibration parameters to rectify stereo are missing!" << endl;
            assert(0);
        }

        cv::initUndistortRectifyMap(K_l,D_l,R_l,P_l.rowRange(0,3).colRange(0,3),cv::Size(cols_l,rows_l),CV_32F,M1l,M2l);
        cv::initUndistortRectifyMap(K_r,D_r,R_r,P_r.rowRange(0,3).colRange(0,3),cv::Size(cols_r,rows_r),CV_32F,M1r,M2r);
    }

    left_sub = std::make_shared<message_filters::Subscriber<ImageMsg> >(this, "/left/image_raw");
    right_sub = std::make_shared<message_filters::Subscriber<ImageMsg> >(this, "/right/image_raw");

    syncApproximate = std::make_shared<message_filters::Synchronizer<approximate_sync_policy> >(approximate_sync_policy(10), *left_sub, *right_sub);
    syncApproximate->registerCallback(&StereoSlamNode::GrabStereo, this);
}

StereoSlamNode::~StereoSlamNode()
{
    // Stop all threads
    m_SLAM->Shutdown();

    // Save camera trajectory
    m_SLAM->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
}

void StereoSlamNode::GrabStereo(const ImageMsg::SharedPtr msgLeft, const ImageMsg::SharedPtr msgRight)
{
    // Copy the ros rgb image message to cv::Mat.
    try
    {
        cv_ptrLeft = cv_bridge::toCvShare(msgLeft);
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    // Copy the ros depth image message to cv::Mat.
    try
    {
        cv_ptrRight = cv_bridge::toCvShare(msgRight);
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    if (doRectify){
        cv::Mat imLeft, imRight;
        cv::remap(cv_ptrLeft->image,imLeft,M1l,M2l,cv::INTER_LINEAR);
        cv::remap(cv_ptrRight->image,imRight,M1r,M2r,cv::INTER_LINEAR);
        Sophus::SE3f Tcw = m_SLAM->TrackStereo(imLeft, imRight, Utility::StampToSec(msgLeft->header.stamp));
        if(!Tcw.translation().isZero()) {
            Sophus::SE3f t_world_to_cam = Tcw.inverse();
            // Angles for rotation matrix (from optical frame to FLU)
            double ax, ay, az;
            ax = 3 * M_PI / 2;
            ay = M_PI / 2;
            az = 0;

            Eigen::Quaternionf q_f = Eigen::AngleAxisf(ax, Eigen::Vector3f::UnitX()) * Eigen::AngleAxisf(ay, Eigen::Vector3f::UnitY()) * Eigen::AngleAxisf(az, Eigen::Vector3f::UnitZ());
            Eigen::Matrix3f rot_max = q_f.toRotationMatrix();
            nav_msgs::msg::Odometry odom_msg;

            odom_msg.header.stamp = this->now();
            odom_msg.header.frame_id = "map";
            odom_msg.child_frame_id = "base_link";
           
            // Position vector converted to FLU from optical frame
            odom_msg.pose.pose.position.x = t_world_to_cam.translation().z();
            odom_msg.pose.pose.position.y = -t_world_to_cam.translation().x();
            odom_msg.pose.pose.position.z = -t_world_to_cam.translation().y();

            Eigen::Quaternionf q(t_world_to_cam.unit_quaternion());
            Eigen::Quaternionf q_rot(rot_max);
            q = q * q_rot;
            odom_msg.pose.pose.orientation.x = q.x();
            odom_msg.pose.pose.orientation.y = q.y();
            odom_msg.pose.pose.orientation.z = q.z();
            odom_msg.pose.pose.orientation.w = q.w();

            _odom_pub->publish(odom_msg);
        }
    }
    else
    {
        Sophus::SE3f Tcw = m_SLAM->TrackStereo(cv_ptrLeft->image, cv_ptrRight->image, Utility::StampToSec(msgLeft->header.stamp));
        if(!Tcw.translation().isZero()) {
            Sophus::SE3f t_world_to_cam = Tcw.inverse();
            // Angles for rotation matrix (from optical frame to FLU)
            double ax, ay, az;
            ax = 3 * M_PI / 2;
            ay = M_PI / 2;
            az = 0;
            
            Eigen::Quaternionf q_f = Eigen::AngleAxisf(ax, Eigen::Vector3f::UnitX()) * Eigen::AngleAxisf(ay, Eigen::Vector3f::UnitY()) * Eigen::AngleAxisf(az, Eigen::Vector3f::UnitZ());
            Eigen::Matrix3f rot_max = q_f.toRotationMatrix();
            nav_msgs::msg::Odometry odom_msg;

            odom_msg.header.stamp = this->now();
            odom_msg.header.frame_id = "map";
            odom_msg.child_frame_id = "base_link";
           
            // Position vector converted to FLU from optical frame
            odom_msg.pose.pose.position.x = t_world_to_cam.translation().z();
            odom_msg.pose.pose.position.y = -t_world_to_cam.translation().x();
            odom_msg.pose.pose.position.z = -t_world_to_cam.translation().y();

            Eigen::Quaternionf q(t_world_to_cam.unit_quaternion());
            Eigen::Quaternionf q_rot(rot_max);
            q = q * q_rot;
            odom_msg.pose.pose.orientation.x = q.x();
            odom_msg.pose.pose.orientation.y = q.y();
            odom_msg.pose.pose.orientation.z = q.z();
            odom_msg.pose.pose.orientation.w = q.w();

            _odom_pub->publish(odom_msg);
        }

    }
}
