cmake_minimum_required(VERSION 3.5)
project(herbert2_cartographer)

# ----------------------------------------------------------------
find_package(ament_cmake REQUIRED)

# ----------------------------------------------------------------

install(
  DIRECTORY config launch rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

# ----------------------------------------------------------------
# ----------------------------------------------------------------