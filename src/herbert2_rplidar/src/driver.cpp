// ----------------------------------------------------------------
#include <rclcpp/rclcpp.hpp>
#include "rplidar_ros/laser_scan_publisher.hpp"

// ----------------------------------------------------------------
// ----------------------------------------------------------------

int main(int argc, char ** argv)
{
    printf("RPLidar ROS LaserScan message publisher package\n");
    rclcpp::init(argc, argv);
    
    auto publisher = std::make_shared<LaserScanPublisher>();
    if (publisher->connect())
    {
        rclcpp::spin(publisher);
        publisher->disconnect();
    }
    rclcpp::shutdown();
    return 0;
}

// ----------------------------------------------------------------
// ----------------------------------------------------------------

