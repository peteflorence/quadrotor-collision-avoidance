cmake_minimum_required(VERSION 2.8.3)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3")
project(trajectory_selector)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  visualization_msgs
  nav_msgs
  roscpp
  std_msgs
  std_srvs
  tf
  tf2
  cv_bridge
  image_transport
  acl_fsw
  message_generation
  sensor_msgs
  PCL REQUIRED
  mavros_msgs
)

add_message_files( DIRECTORY msg FILES Coeff.msg State.msg )
generate_messages( DEPENDENCIES std_msgs nav_msgs)

find_package( Eigen3 REQUIRED )
find_package(OpenCV 2.4.8 REQUIRED)

include_directories ( src )
include_directories( ${EIGEN3_INCLUDE_DIR} )
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})

catkin_package(

)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

find_package(orocos_kdl REQUIRED)
find_library(OROCOS_KDL orocos-kdl)
set(orocos_kdl_LIBRARIES ${OROCOS_KDL})


add_library( trajectory_selector src/trajectory_selector.cpp src/trajectory_library.cpp src/trajectory_evaluator.cpp  src/trajectory.cpp src/attitude_generator.cpp src/trajectory_visualizer.cpp src/value_grid_evaluator.cpp src/value_grid.cpp src/trajectory_selector_utils.cpp src/laser_scan_collision_evaluator.cpp src/depth_image_collision_evaluator.cpp src/kd_tree.cpp)


add_executable( trajectory_selector_node src/trajectory_selector_node.cpp )
target_link_libraries( trajectory_selector_node trajectory_selector ${catkin_LIBRARIES} ${PCL_LIBRARIES} orocos-kdl)