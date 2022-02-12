################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.5)
project(herbert2_description)

################################################################################
# Find ament packages and libraries for ament and system dependencies
################################################################################

find_package(ament_cmake REQUIRED)
find_package(urdf REQUIRED)
find_package(xacro REQUIRED)

################################################################################
# Install
################################################################################

install(
    DIRECTORY 
        launch
        meshes 
        rviz 
        urdf
    DESTINATION 
        share/${PROJECT_NAME}
)

################################################################################
# Macro for ament package
################################################################################
ament_package()
