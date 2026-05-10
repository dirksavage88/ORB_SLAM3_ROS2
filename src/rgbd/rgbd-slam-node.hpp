#ifndef __RGBD_SLAM_NODE_HPP__
#define __RGBD_SLAM_NODE_HPP__

#include <mutex>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include <opencv2/core/core.hpp>

#include <libobsensor/ObSensor.hpp>
#include <libobsensor/h/ObTypes.h>

#include "System.h"
#include "Frame.h"
#include "Map.h"
#include "Tracking.h"

class RgbdSlamNode : public rclcpp::Node
{
public:
    explicit RgbdSlamNode(ORB_SLAM3::System* pSLAM);
    ~RgbdSlamNode();

private:
    // Called by the ROS2 timer at 30fps to process the latest buffered frame
    void TimerCallback();

    // Called by the Orbbec SDK on a background thread each time a frameset arrives
    void FrameCallback(std::shared_ptr<ob::FrameSet> frameSet);

    ORB_SLAM3::System* m_SLAM;

    std::shared_ptr<ob::Pipeline> m_pipeline;

    // Shared frame buffer written by FrameCallback, read by TimerCallback
    std::mutex imu_mutex;
    cv::Mat    m_color_frame;
    cv::Mat    m_depth_frame;
    int count_im_buffer = 0;
    std::condition_variable cond_image_rec;
    double     m_frame_timestamp{-1.0};
    bool       m_new_frame_ready{false};

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr m_odom_pub;
    rclcpp::TimerBase::SharedPtr m_timer;
};

#endif
