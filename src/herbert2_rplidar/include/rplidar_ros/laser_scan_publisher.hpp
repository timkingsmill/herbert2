#pragma once

// ----------------------------------------------------------------

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include "rplidar.h"


// ----------------------------------------------------------------

#ifndef _countof
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif

#define DEG2RAD(x) ((x)*M_PI/180.)

using namespace rp::standalone::rplidar;

// ----------------------------------------------------------------
// ----------------------------------------------------------------

class LaserScanPublisher : public rclcpp::Node
{
public:
    LaserScanPublisher();
    ~LaserScanPublisher();

    bool connect();
    void disconnect() {};

private:
    bool _checkRPLIDARHealth();

    float _getAngle(const rplidar_response_measurement_node_hq_t& node);
    bool _getRPLIDARDeviceInfo();
    void _load_parameters();
    
    // node_count:    The number of scan nodes in the entire scan.
    // scan_duration: Time taken to collect an entire scan in seconds.
    bool _create_rplidar_scan_msg(rplidar_response_measurement_node_hq_t *nodes,
                                  size_t node_count, 
                                  rclcpp::Duration scan_duration,
                                  float angle_min, float angle_max);
                       
    void _updateRPLidarScan();
    bool _initRPLidarScanMode();
    void _timer_callback();

private:
    RPlidarDriver * _driver = nullptr;
    size_t count_;
    std::string _serial_port;
    int _serial_baudrate;
    bool _is_connected = false;
    bool _inverted = false;
    float _max_distance = 8.0;
    std::string _frame_id = "laser_frame";
    std::string _publisher_topic = "rplidar_scan";
    std::string _scan_mode;
    const uint32_t _request_timeout  = RPlidarDriver::DEFAULT_TIMEOUT * 2;
    bool _angle_compensate = true;
    // it stand of angle compensate at per 1 degree
    uint32_t _angle_compensate_multiple = 1; 

 
    rclcpp::TimerBase::SharedPtr _timer;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr _publisher;

};

// ----------------------------------------------------------------
// ----------------------------------------------------------------
