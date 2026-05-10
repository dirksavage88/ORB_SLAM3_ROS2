#include "rgbd-slam-node.hpp"

#include <opencv2/imgproc/imgproc.hpp>

RgbdSlamNode::RgbdSlamNode(ORB_SLAM3::System* pSLAM)
:   Node("ORB_SLAM3_RGBD_ORBBEC"),
    m_SLAM(pSLAM)
{
    // ── Orbbec SDK setup ──────────────────────────────────────────────────
    m_pipeline = std::make_shared<ob::Pipeline>();

    auto cfg = std::make_shared<ob::Config>();
    // Color as RGB; we convert to BGR in FrameCallback to match OpenCV convention
    cfg->enableVideoStream(OB_STREAM_COLOR, OB_WIDTH_ANY, OB_HEIGHT_ANY, OB_FPS_ANY, OB_FORMAT_RGB);
    // Depth as 16-bit (millimetres); DepthMapFactor=1000 in the YAML handles the scaling
    cfg->enableVideoStream(OB_STREAM_DEPTH, OB_WIDTH_ANY, OB_HEIGHT_ANY, OB_FPS_ANY, OB_FORMAT_Y14);
    // Without this the SDK emits single-stream FrameSets and the RGBD pipeline starves.                                                             
    //cfg->setFrameAggregateOutputMode(OB_FRAME_AGGREGATE_OUTPUT_FULL_FRAME_REQUIRE);                                                                  
                                                                                                                                                     
    // Align color and depth timestamps so ORB-SLAM3 sees a coherent pair.                                                                           
    //m_pipeline->enableFrameSync();                                   
    m_pipeline->start(cfg, [this](std::shared_ptr<ob::FrameSet> fs) {
        this->FrameCallback(fs);
    });

    RCLCPP_INFO(this->get_logger(), "Orbbec pipeline started");

    // ── ROS2 odometry publisher ──────────────────────────────────────────
    m_odom_pub = this->create_publisher<nav_msgs::msg::Odometry>("orbslam3/odometry", 10);

    // ── 30 fps timer ─────────────────────────────────────────────────────
    // Interval matches the camera framerate so each tick consumes one frame.
    m_timer = this->create_wall_timer(
        std::chrono::milliseconds(1),
        std::bind(&RgbdSlamNode::TimerCallback, this));
}

RgbdSlamNode::~RgbdSlamNode()
{
    m_timer->cancel();

    if (m_pipeline) {
        m_pipeline->stop();
    }

    m_SLAM->Shutdown();
    m_SLAM->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
}

// ── FrameCallback ─────────────────────────────────────────────────────────
// Runs on the Orbbec SDK background thread. Extracts color + depth into the
// shared buffer so TimerCallback can consume them on the executor thread.
void RgbdSlamNode::FrameCallback(std::shared_ptr<ob::FrameSet> frameSet)
{
    std::unique_lock<std::mutex> lock(imu_mutex);
    if (!frameSet) return;
    count_im_buffer++;
    
    double new_timestamp_image = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count() * 1e-3;
    
    // Limit frame rate to 15 FPS to reduce processing load
    if (abs(m_frame_timestamp - new_timestamp_image) < 0.067) { // 1/15 = 0.067 seconds
        count_im_buffer--;
        return;
    }

    try {
        auto colorFrame = frameSet->getFrame(OB_FRAME_COLOR);
        if (colorFrame) {
            auto vf = colorFrame->as<ob::VideoFrame>();
            m_color_frame = cv::Mat(static_cast<int>(vf->getHeight()),
                        static_cast<int>(vf->getWidth()),
                        CV_8UC3,
                        static_cast<uint8_t*>(vf->getData()));
            // Orbbec delivers OB_FORMAT_RGB; convert to BGR for OpenCV / ORB-SLAM3
            cv::cvtColor(m_color_frame, m_color_frame, cv::COLOR_RGB2BGR);
            // Resize to the resolution expected by the YAML config (640×360)
            cv::resize(m_color_frame, m_color_frame, cv::Size(640, 360));
	    std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }

        auto depthFrame = frameSet->getFrame(OB_FRAME_DEPTH);
        if (depthFrame) {
            auto vf = depthFrame->as<ob::VideoFrame>();
            m_depth_frame = cv::Mat(static_cast<int>(vf->getHeight()),
                        static_cast<int>(vf->getWidth()),
                        CV_16U,
                        static_cast<uint8_t*>(vf->getData()));
            cv::resize(m_depth_frame, m_depth_frame
			    , cv::Size(640, 360));
        }
    } catch (const ob::Error& e) {
        RCLCPP_ERROR(this->get_logger(), "Orbbec frame error: %s", e.what());
        return;
    }

    //if(color.empty() || depth.empty()) return;

    //RCLCPP_INFO(this->get_logger(), "cb: color=%d depth=%d",
    //          (int)!color.empty(), (int)!depth.empty());



    //std::lock_guard<std::mutex> lock(m_frame_mutex);
    //m_color_frame     = color.clone();
    //m_depth_frame     = depth.clone();
    m_frame_timestamp = new_timestamp_image;
    m_new_frame_ready = true;
    lock.unlock();
    cond_image_rec.notify_all();
}

// ── TimerCallback ─────────────────────────────────────────────────────────
// Runs on the ROS2 executor thread at ~30 fps. Grabs the latest buffered
// frame, calls TrackRGBD, and publishes odometry when tracking is healthy.
void RgbdSlamNode::TimerCallback()
{
    cv::Mat color, depth;
    double  timestamp;

    {
        std::unique_lock<std::mutex> lk(imu_mutex);
        if (!m_new_frame_ready) cond_image_rec.wait(lk);
        if (count_im_buffer > 1) count_im_buffer = 0;
        color     = m_color_frame.clone();
        depth     = m_depth_frame.clone();
        timestamp = m_frame_timestamp;
        m_new_frame_ready = false;
    }

    // Apply any scale the YAML requests (Camera.imageScale)
    float scale = m_SLAM->GetImageScale();
    if (scale != 1.f) {
        cv::resize(color, color, cv::Size(
            static_cast<int>(color.cols * scale),
            static_cast<int>(color.rows * scale)));
        cv::resize(depth, depth, cv::Size(
            static_cast<int>(depth.cols * scale),
            static_cast<int>(depth.rows * scale)));
    }

    Sophus::SE3f pose = m_SLAM->TrackRGBD(color, depth, timestamp);

    // Tracking states: 0=NO_IMAGES_YET, 1=NOT_INITIALIZED, 2=OK,
    //                  3=RECENTLY_LOST, 4=LOST
    // Only publish a reliable pose when fully tracking.
    if (m_SLAM->GetTrackingState() != 2) return;

    auto odom = nav_msgs::msg::Odometry();
    odom.header.stamp    = this->now();
    odom.header.frame_id = "odom";
    odom.child_frame_id  = "camera_link";

    // TrackRGBD returns T_cw (camera-in-world); invert to get world-in-camera
    // then take the world position of the camera.
    Eigen::Vector3f    t = pose.translation();
    Eigen::Quaternionf q = pose.unit_quaternion();

    odom.pose.pose.position.x    = t.x();
    odom.pose.pose.position.y    = t.y();
    odom.pose.pose.position.z    = t.z();
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();

    m_odom_pub->publish(odom);
}
