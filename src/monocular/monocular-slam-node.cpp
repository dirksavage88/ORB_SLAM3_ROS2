#include "monocular-slam-node.hpp"

#include<opencv2/core/core.hpp>
#include <string>

using std::placeholders::_1;

MonocularSlamNode::MonocularSlamNode(ORB_SLAM3::System* pSLAM)
:   Node("ORB_SLAM3_ROS2")
{
    m_SLAM = pSLAM;

    // Input image topic. Define orb vocab and settings (yaml) path
    this->declare_parameter("image", "/image");
    
    //TODO
    //this->declare_parameter("orb_vocabulary", "");
    //this->declare_parameter("settings", "");

    // QoS 
    rmw_qos_profile_t qos_custom_profile = rmw_qos_profile_system_default;
    image_topic = this->get_parameter("image").as_string();

    //TODO
    //orb_vocab = this->get_parameter("orb_vocabulary").as_string();
    //settings_file = this->get_parameter("settings").as_string();

    
    m_image_subscriber = image_transport::create_camera_subscription(
	this,
        image_topic.c_str(),
        std::bind(&MonocularSlamNode::GrabImage, this, std::placeholders::_1), "raw", qos_custom_profile);
    std::cout << "slam changed" << std::endl;
}

MonocularSlamNode::~MonocularSlamNode()
{
    // Stop all threads
    m_SLAM->Shutdown();

    // Save camera trajectory
    m_SLAM->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
}

void MonocularSlamNode::GrabImage(sensor_msgs::msg::Image::ConstSharedPtr msg)
{
    // Copy the ros image message to cv::Mat.
    try
    {
        m_cvImPtr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    std::cout<<"one frame has been sent"<<std::endl;
    m_SLAM->TrackMonocular(m_cvImPtr->image, Utility::StampToSec(msg->header.stamp));
}
