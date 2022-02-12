cmake_minimum_required(VERSION 3.5)

project(herbert2_rplidar)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall 
                      -Wextra 
                      -Wpedantic 
                      -Wvla)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

# ----------------------------------------------------
# ----------------------------------------------------
# Setup the SDK
set(RPLIDAR_SDK_PATH "./sdk/")

FILE(GLOB RPLIDAR_SDK_SRC 
  "${RPLIDAR_SDK_PATH}/src/arch/linux/*.cpp"
  "${RPLIDAR_SDK_PATH}/src/hal/*.cpp"
  "${RPLIDAR_SDK_PATH}/src/*.cpp"
)

# ----------------------------------------------------
# ----------------------------------------------------

set(driver "rplidar_driver")

add_executable(${driver} 
    src/driver.cpp
    src/laser_scan_publisher.cpp
    ${RPLIDAR_SDK_SRC}
)

ament_target_dependencies(
    ${driver} 
        rclcpp std_msgs sensor_msgs
)

target_include_directories(${driver} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${RPLIDAR_SDK_PATH}/include
    ${RPLIDAR_SDK_PATH}/src
)

# ----------------------------------------------------
# ----------------------------------------------------

set(client "client")

add_executable(${client} 
    src/client.cpp
    src/laser_scan_subscriber.cpp
    ${RPLIDAR_SDK_SRC}
)

ament_target_dependencies(
    ${client} 
        rclcpp std_msgs sensor_msgs
)

target_include_directories(${client} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${RPLIDAR_SDK_PATH}/include
    ${RPLIDAR_SDK_PATH}/src
)
    
# ----------------------------------------------------
# ----------------------------------------------------

install(TARGETS ${driver} ${client}
  DESTINATION lib/${PROJECT_NAME})

install(
    DIRECTORY config launch rviz
    DESTINATION share/${PROJECT_NAME}
 )

# ----------------------------------------------------

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# ----------------------------------------------------

ament_package()

# ----------------------------------------------------
# ----------------------------------------------------
