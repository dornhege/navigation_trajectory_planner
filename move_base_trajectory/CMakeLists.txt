cmake_minimum_required(VERSION 2.8.3)
project(move_base_trajectory)
set(CMAKE_BUILD_TYPE RelWithDebInfo)

find_package(catkin REQUIRED
    COMPONENTS
        actionlib
        ais_rosparam_tools
        ais_ros_msg_conversions
        angles
        bonirob_navigation_msgs
        cmake_modules
        diagnostics_simple
        dynamic_reconfigure
        eigen_conversions
        geometry_msgs
        gps_reference
        message_generation
        moveit_msgs
#        nav_core
        pluginlib
        roscpp
        #tf
        tf2
        tf2_ros
        tf2_geometry_msgs
)
find_package(Eigen REQUIRED)
add_definitions(${EIGEN_DEFINITIONS})

# dynamic reconfigure
generate_dynamic_reconfigure_options(
    cfg/MoveBase.cfg
)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES move_base_interfaces
              move_base_trajectory
    CATKIN_DEPENDS
        actionlib
        ais_rosparam_tools
        ais_ros_msg_conversions
        angles
        dynamic_reconfigure
        eigen_conversions
        geometry_msgs
        pluginlib
        roscpp
        #tf
        tf2
        tf2_ros
        tf2_geometry_msgs
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN_INCLUDE_DIRS}
)

# move_base
add_library(move_base_interfaces
  src/base_local_planner.cpp
  src/base_global_planner.cpp
)

add_library(move_base_trajectory
  src/move_base_trajectory.cpp
  src/global_planner_thread.cpp
  src/local_planner.cpp
)
target_link_libraries(move_base_trajectory
    move_base_interfaces
    ${Boost_LIBRARIES}
    ${catkin_LIBRARIES})

add_executable(move_base_trajectory_node
  src/move_base_trajectory_node.cpp
)
target_link_libraries(move_base_trajectory_node move_base_trajectory ${catkin_LIBRARIES})
#target_link_libraries(move_base_trajectory_node move_base_trajectory)


install(DIRECTORY launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
    USE_SOURCE_PERMISSIONS
    )

install(
    PROGRAMS
       scripts/subtopic_forwarder.py
       scripts/subtopic_forwarder_node.py
       scripts/warner.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
    TARGETS
        move_base_trajectory
        move_base_interfaces
        move_base_trajectory_node
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)
