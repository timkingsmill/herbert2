// ---------------------------------------------------------------------------

#include <rclcpp/rclcpp.hpp>
#include "rplidar_ros/laser_scan_subscriber.hpp"

// ---------------------------------------------------------------------------
// ---------------------------------------------------------------------------

int main(int argc, char ** argv)
{
    printf("RPLidar LaserScan message subscriber\n");
    rclcpp::init(argc, argv);
    auto subscriber = std::make_shared<LaserScanSubscriber>();
    rclcpp::spin(subscriber);
    rclcpp::shutdown();
    return 0;
}

// ---------------------------------------------------------------------------
// ---------------------------------------------------------------------------
