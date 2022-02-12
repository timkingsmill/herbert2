#include <memory>
#include "rplidar_ros/laser_scan_subscriber.hpp"

using std::placeholders::_1;

// -------------------------------------------------------------------------------------------

#define RAD2DEG(x) ((x)*180./M_PI)

// -------------------------------------------------------------------------------------------
// -------------------------------------------------------------------------------------------

LaserScanSubscriber::LaserScanSubscriber():
    Node("laser_scan_subscriber")
{

    this->declare_parameter<std::string>("publisher_topic", "rplidar_scan");
    this->get_parameter("publisher_topic", _publisher_topic);

    _subscription = this->create_subscription<sensor_msgs::msg::LaserScan>(
                    _publisher_topic, 
                    rclcpp::SensorDataQoS(), 
                    std::bind(&LaserScanSubscriber::_scanCallback, this, _1));
}

// -------------------------------------------------------------------------------------------

LaserScanSubscriber::~LaserScanSubscriber()
{
}

// -------------------------------------------------------------------------------------------
// -------------------------------------------------------------------------------------------

void LaserScanSubscriber::_scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan) const
{
    RCLCPP_INFO(get_logger(), "---------------------------------------------------");
    RCLCPP_INFO(get_logger(), "Laser Scan Message Callback");

    int count = scan->scan_time / scan->time_increment;

    RCLCPP_INFO(get_logger(), "I heard a laser scan %s[%d]:", scan->header.frame_id.c_str(), count);
    RCLCPP_INFO(get_logger(), "angle_range, %f, %f", RAD2DEG(scan->angle_min), RAD2DEG(scan->angle_max));
 
    for(int i = 0; i < count; i++) 
    {
        float degree = RAD2DEG(scan->angle_min + scan->angle_increment * i);
        RCLCPP_INFO(get_logger(), ": [%f, %f]", degree, scan->ranges[i]);
    }
}


// -------------------------------------------------------------------------------------------
// -------------------------------------------------------------------------------------------
