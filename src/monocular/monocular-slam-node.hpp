#ifndef __MONOCULAR_SLAM_NODE_HPP__
#define __MONOCULAR_SLAM_NODE_HPP__

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

#include <cv_bridge/cv_bridge.h>
#include <string>
#include <image_transport/image_transport.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include "System.h"
#include "Frame.h"
#include "Map.h"
#include "Tracking.h"

#include "utility.hpp"

class MonocularSlamNode : public rclcpp::Node
{
public:
    MonocularSlamNode(ORB_SLAM3::System* pSLAM);
    
    ~MonocularSlamNode();

private:
    
    std::string image_topic;
    //std::string orb_vocabulary;
    //std::string settings_file;

    void GrabImage(sensor_msgs::msg::Image::ConstSharedPtr msg);

    ORB_SLAM3::System* m_SLAM;

    cv_bridge::CvImageConstPtr m_cvImPtr;

    image_transport::CameraSubscriber m_image_subscriber;
};

#endif
