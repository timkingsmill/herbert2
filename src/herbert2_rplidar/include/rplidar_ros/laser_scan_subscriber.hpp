#pragma once

// ----------------------------------------------------------------

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include "rplidar.h"

// ----------------------------------------------------------------

using namespace rp::standalone::rplidar;

// ----------------------------------------------------------------
// ----------------------------------------------------------------

class LaserScanSubscriber : public rclcpp::Node
{
public:
    LaserScanSubscriber();
    ~LaserScanSubscriber();
private:    
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr _subscription;
    void _scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan) const;

    std::string _publisher_topic = "rplidar_scan";
};

// ----------------------------------------------------------------
// ----------------------------------------------------------------

