// ----------------------------------------------------------

#include <chrono>
#include <memory>
#include "rplidar_ros/laser_scan_publisher.hpp"

// ----------------------------------------------------------

using namespace std::chrono_literals;

// ----------------------------------------------------------

LaserScanPublisher::LaserScanPublisher(): 
    Node("laser_scan_publisher"), 
    count_(0)
{
    RCLCPP_INFO(get_logger(), "Creating the LaserScanPublisher ROS node.");
    _load_parameters();
}

// -------------------------------------------------------------------------------------------

LaserScanPublisher::~LaserScanPublisher()
{  
    RCLCPP_INFO(get_logger(), "Destroyed the LaserScanPublisher ROS node.");
    if (_driver)
    {
        _driver->stopMotor();
        _driver->stop();
        RPlidarDriver::DisposeDriver(_driver);
    }
    _driver = nullptr;
}

// -------------------------------------------------------------------------------------------
// -------------------------------------------------------------------------------------------
// Connect to the RPLidar hardware.
bool LaserScanPublisher::connect()
{
    _is_connected = false;   
    RCLCPP_INFO(get_logger(), "Connecting to the RPLidar. (Port: %s)", _serial_port.c_str());

    // Create the serial port driver.
    _driver = RPlidarDriver::CreateDriver(rp::standalone::rplidar::DRIVER_TYPE_SERIALPORT);

    // Connect to the RPLidar via the serial port.
    u_result op_result = _driver->connect(_serial_port.c_str(), (_u32)_serial_baudrate);
    if (IS_FAIL(op_result))
    {
        //RPlidarDriver::DisposeDriver(_driver);
        RCLCPP_FATAL(get_logger(), "Error, cannot bind to the specified serial port %s.", 
                                    _serial_port.c_str());
        return false;
    }

    RCLCPP_INFO(get_logger(), "Connected to RPLidar driver. Path: %s Baud: %d", 
                                _serial_port.c_str(), _serial_baudrate);

    _driver->startMotor();

    if (!_checkRPLIDARHealth())
    {
        //RPlidarDriver::DisposeDriver(_driver);
        RCLCPP_FATAL(get_logger(), "Error, Failed to check the RPLidars health.");
        return false;
    }
    RCLCPP_INFO(get_logger(), "RPLidar Health is OK");
    
    if (!_getRPLIDARDeviceInfo())
    {
        //RPlidarDriver::DisposeDriver(_driver);
        RCLCPP_FATAL(get_logger(), "Error, Failed to collect the RPLidar device information.");
        return false;
    }

    RCLCPP_INFO(get_logger(), "Connected to the RPLidar. No errors detected");

    /* TODO
    ros::ServiceServer stop_motor_service = nh.advertiseService("stop_motor", stop_motor);
    ros::ServiceServer start_motor_service = nh.advertiseService("start_motor", start_motor);
    */

    if (!_initRPLidarScanMode())
    {
        RCLCPP_FATAL(get_logger(), "Error, Failed to initialize the RPLidars scan mode.");
        return false;
    }

    _timer = this->create_wall_timer(100ms, std::bind(&LaserScanPublisher::_timer_callback, this));
    _publisher = this->create_publisher<sensor_msgs::msg::LaserScan>(_publisher_topic, rclcpp::SensorDataQoS());
   
    RCLCPP_INFO(get_logger(), "Ready to publish the RPLidar's scan data...");

    _is_connected = true;   

    return true;
}

// -------------------------------------------------------------------------------------------

bool LaserScanPublisher::_checkRPLIDARHealth()
{ 
    rplidar_response_device_health_t healthinfo;
    u_result op_result = _driver->getHealth(healthinfo, _request_timeout);
    if (IS_OK(op_result)) 
    { 
        RCLCPP_INFO(get_logger(), "RPLidar health status : %d", healthinfo.status);
        if (healthinfo.status == RPLIDAR_STATUS_ERROR) 
        {
            RCLCPP_FATAL(get_logger(), "Error, rplidar internal error detected. Please reboot the device to retry.");
            return false;
        } 
        else 
        {
            return true;
        }
    } 
    else 
    {
        RCLCPP_FATAL(get_logger(), "Error, cannot retrieve rplidar health code: %x", op_result);
        return false;
    }
    return false;
}

// -------------------------------------------------------------------------------------------

float LaserScanPublisher::_getAngle(const rplidar_response_measurement_node_hq_t& node)
{
    return node.angle_z_q14 * 90.f / 16384.f;
}

// -------------------------------------------------------------------------------------------

bool LaserScanPublisher::_getRPLIDARDeviceInfo()
{
    rplidar_response_device_info_t devinfo;
    
    u_result op_result = _driver->getDeviceInfo(devinfo, _request_timeout);
    if (IS_FAIL(op_result)) 
    {
        if (op_result == RESULT_OPERATION_TIMEOUT) 
        {
            RCLCPP_FATAL(get_logger(), "Error, operation time out. RESULT_OPERATION_TIMEOUT!");
        } 
        else 
        {
            RCLCPP_FATAL(get_logger(), "Error, unexpected error, code: %x", op_result);
        }
        return false;
    }

    // print out the device serial number, firmware and hardware version number..
    printf("RPLIDAR S/N: ");
    for (int pos = 0; pos < 16 ;++pos) 
    {
        printf("%02X", devinfo.serialnum[pos]);
    }
    printf("\n");

    RCLCPP_INFO(get_logger(), "Firmware Ver: %d.%02d", devinfo.firmware_version>>8, devinfo.firmware_version & 0xFF);
    RCLCPP_INFO(get_logger(), "Hardware Rev: %d", (int)devinfo.hardware_version);

    return true;
}

// -------------------------------------------------------------------------------------------

//  Load parameters
void LaserScanPublisher::_load_parameters()
{
    this->declare_parameter<std::string>("serial_port", "/dev/rplidar");
    this->get_parameter("serial_port", _serial_port);

    this->declare_parameter<int>("serial_baudrate", 115200);
    this->get_parameter("serial_baudrate", _serial_baudrate);

    this->declare_parameter<std::string>("publisher_topic", "rplidar_scan");
    this->get_parameter("publisher_topic", _publisher_topic);

    this->declare_parameter<std::string>("scan_mode", _scan_mode);
    this->get_parameter("scan_mode", _scan_mode);

    this->declare_parameter<std::string>("frame_id", _frame_id);
    this->get_parameter("frame_id", _frame_id);
}

// -------------------------------------------------------------------------------------------
// Init the RPLidar scan mode.
bool LaserScanPublisher::_initRPLidarScanMode()
{
    RCLCPP_INFO(get_logger(), "Initializing  lidar scanning");
    RplidarScanMode current_scan_mode;
    u_result op_result;

    if (_scan_mode.empty()) 
    {
        RCLCPP_INFO(get_logger(), "Scan Mode is not supplied, will use default"); 
        op_result = _driver->startScan(
                        false,          // not force scan 
                        true,           // use typical scan mode 
                        0, &current_scan_mode);
    }
    else
    {
        std::vector<RplidarScanMode> allSupportedScanModes;
        op_result = _driver->getAllSupportedScanModes(allSupportedScanModes, _request_timeout);
        if (IS_OK(op_result)) 
        {
            for (std::vector<RplidarScanMode>::iterator 
                    iter = allSupportedScanModes.begin(); 
                    iter != allSupportedScanModes.end(); iter++) 
            {
                RCLCPP_INFO(get_logger(), iter->scan_mode);
            }
        }
    }

    if (IS_OK(op_result))
    {
        // default frequent is 10 hz (by motor pwm value),  
        // current_scan_mode.us_per_sample is the number of scan point per us
        _angle_compensate_multiple = (int)(1000 * 1000 / current_scan_mode.us_per_sample / 10.0 /360.0);
        
        if (_angle_compensate_multiple < 1) 
            _angle_compensate_multiple = 1;

        _max_distance = current_scan_mode.max_distance;

        RCLCPP_INFO(get_logger(), 
                    "Current Scan Mode: %s, max_distance: %.1f m, Point number: %.1fK , angle_compensate: %d",  
                    current_scan_mode.scan_mode,
                    current_scan_mode.max_distance, 
                    (1000 / current_scan_mode.us_per_sample), 
                    _angle_compensate_multiple);

        return true;
    }
    else
    {
        RCLCPP_FATAL(get_logger(), "Can not start scan: %08x!", op_result);
        return false;
    }
    
    return true;
}

// -------------------------------------------------------------------------------------------

void LaserScanPublisher::_timer_callback()
{
    if (_is_connected)
        _updateRPLidarScan();       
}

// -------------------------------------------------------------------------------------------

void LaserScanPublisher::_updateRPLidarScan()
{
    rplidar_response_measurement_node_hq_t nodes[360 * 8];
    size_t count = _countof(nodes);

    // Collect the current scan data.
    rclcpp::Time start_scan_time = this->now(); //rclcpp::Clock().now();
    u_result op_result = _driver->grabScanDataHq(nodes, count, _request_timeout);
    rclcpp::Time end_scan_time = this->now(); //rclcpp::Clock().now();

    // Calculate the time required to collect the scan data in nanoseconds.
    rclcpp::Duration scan_duration(end_scan_time - start_scan_time); 
    //double scan_duration = (end_scan_time - start_scan_time).nanoseconds();

    if (op_result == RESULT_OK) 
    {
        float angle_min = DEG2RAD(0.0f);
        float angle_max = DEG2RAD(359.0f);

        op_result = _driver->ascendScanData(nodes, count);
        if (op_result == RESULT_OK) 
        {
            if (_angle_compensate) 
            {
                const int angle_compensate_nodes_count = 360 * _angle_compensate_multiple;
                int angle_compensate_offset = 0;
                rplidar_response_measurement_node_hq_t angle_compensate_nodes[angle_compensate_nodes_count];
                memset(angle_compensate_nodes, 0, angle_compensate_nodes_count*sizeof(rplidar_response_measurement_node_hq_t));

                size_t i = 0, 
                j = 0;
                for( ; i < count; i++ ) 
                {
                    if (nodes[i].dist_mm_q2 != 0) 
                    {
                        float angle = _getAngle(nodes[i]);
                        int angle_value = (int)(angle * _angle_compensate_multiple);
                        if ((angle_value - angle_compensate_offset) < 0) 
                        {
                            angle_compensate_offset = angle_value;
                        }
                        for (j = 0; j < _angle_compensate_multiple; j++) 
                        {
                            int angle_compensate_nodes_index = angle_value - angle_compensate_offset + j;
                            if (angle_compensate_nodes_index >= angle_compensate_nodes_count)
                            {
                                angle_compensate_nodes_index = angle_compensate_nodes_count -1;
                            }
                            angle_compensate_nodes[angle_compensate_nodes_index] = nodes[i];
                        }
                    }
                }

                _create_rplidar_scan_msg(angle_compensate_nodes,
                                         angle_compensate_nodes_count,
                                                        scan_duration,
                                                        angle_min,
                                                        angle_max);


            }
            else
            {
                /***
                int start_node = 0, end_node = 0;
                int i = 0;

                // find the first valid node and last valid node
                while (nodes[i++].dist_mm_q2 == 0);
                start_node = i-1;
                i = count -1;
                while (nodes[i--].dist_mm_q2 == 0);
                end_node = i+1;

                angle_min = DEG2RAD(_getAngle(nodes[start_node]));
                angle_max = DEG2RAD(_getAngle(nodes[end_node]));
                ***/

                RCLCPP_INFO(get_logger(), "PUBLISH SCAN MESSAGE HERE (1)"); 
            }
        }
        else if (op_result == RESULT_OPERATION_FAIL)
        {
            /****
            angle_min = DEG2RAD(0.0f);
            angle_max = DEG2RAD(359.0f);
            **/
            RCLCPP_INFO(get_logger(), "PUBLISH SCAN MESSAGE HERE (2)"); 
        }
    }
}

// -------------------------------------------------------------------------------------------

bool LaserScanPublisher::_create_rplidar_scan_msg(
                       rplidar_response_measurement_node_hq_t *nodes,
                       size_t node_count, 
                       rclcpp::Duration scan_duration,
                       float angle_min, float angle_max)
{
    static int scan_count = 0;
    sensor_msgs::msg::LaserScan scan_msg;

    scan_msg.header.frame_id = _frame_id;
    scan_count++;

    /**
    bool reversed = (angle_max > angle_min);
    if ( reversed ) 
    {
        scan_msg.angle_min =  M_PI - angle_max;
        scan_msg.angle_max =  M_PI - angle_min;
    } 
    else 
    {
        scan_msg.angle_min =  M_PI - angle_min;
        scan_msg.angle_max =  M_PI - angle_max;
    }
    **/

    scan_msg.angle_min = 0;
    scan_msg.angle_max =  M_PI * 2.0;

    scan_msg.angle_increment =
        (scan_msg.angle_max - scan_msg.angle_min) / (double)(node_count - 1);

    // Time taken to collect an entire scan.).
    scan_msg.scan_time = scan_duration.seconds();
    scan_msg.time_increment = scan_duration.seconds() / (double)(node_count - 1);

    scan_msg.range_min = 0.15;
    scan_msg.range_max = _max_distance;//8.0;

    scan_msg.intensities.resize(node_count);
    scan_msg.ranges.resize(node_count);

    /***
    bool reverse_data = (!_inverted && reversed) || (_inverted && !reversed);
    if (!reverse_data) 
    {
        for (size_t i = 0; i < node_count; i++) 
        {
            float read_value = (float)nodes[i].dist_mm_q2 / 4.0f / 1000;
            if (read_value == 0.0)
            {
                scan_msg.ranges[i] = std::numeric_limits<float>::infinity();
            }
            else
            {
                scan_msg.ranges[i] = read_value;
            }
            scan_msg.intensities[i] = (float) (nodes[i].quality >> 2);
        }
    } 
    else 
    {
        ****/
        for (size_t i = 0; i < node_count; i++) 
        {
            float read_value = (float)nodes[i].dist_mm_q2 / 4.0f / 1000;
            if (read_value == 0.0)
            {
                scan_msg.ranges[node_count-1-i] = std::numeric_limits<float>::infinity();
            }
            else
            {
                scan_msg.ranges[node_count-1-i] = read_value;
            }
            scan_msg.intensities[node_count-1-i] = (float) (nodes[i].quality >> 2);
        }
    //}
    // RCLCPP_INFO(get_logger(), "Publishing scan message."); 

    scan_msg.header.stamp = this->now();
    _publisher->publish(scan_msg);
    return true;
}

// -------------------------------------------------------------------------------------------
// -------------------------------------------------------------------------------------------

