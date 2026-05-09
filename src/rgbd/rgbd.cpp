#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "rgbd-slam-node.hpp"

#include "System.h"

int main(int argc, char **argv)
{
    if (argc < 3) {
        std::cerr << "\nUsage: ros2 run orbslam3 rgbd path_to_vocabulary path_to_settings\n";
        return 1;
    }

    rclcpp::init(argc, argv);

    // Visualization via Pangolin is enabled; set to false to run headless.
    bool visualization = true;
    ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::RGBD, visualization);

    auto node = std::make_shared<RgbdSlamNode>(&SLAM);

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
