cmake_minimum_required(VERSION 3.0.2)
project(lane_detector)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  cv_bridge
)

catkin_package(
  CATKIN_DEPENDS
    rospy
    std_msgs
    geometry_msgs
    sensor_msgs
    nav_msgs
    cv_bridge
)

include_directories(${catkin_INCLUDE_DIRS})

# ---- Install Python nodes ----
catkin_install_python(PROGRAMS
  scripts/detect_lane.py
  scripts/control_lane.py
  scripts/control_blind.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# ---- Install launch, model, URDF, and world files ----
install(DIRECTORY launch models urdf worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
