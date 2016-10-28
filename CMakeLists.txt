cmake_minimum_required(VERSION 2.8.3)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3")
project(motion_primitives)

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
  mavros_msgs
  pcl_ros
)

add_message_files( DIRECTORY msg FILES Coeff.msg State.msg )
generate_messages( DEPENDENCIES std_msgs nav_msgs)

find_package( Eigen3 REQUIRED )
find_package(OpenCV 2.4.8 REQUIRED)
find_package(PCL REQUIRED)

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


add_library( motion_selector src/motion_selector.cpp src/motion_library.cpp src/motion.cpp src/attitude_generator.cpp src/motion_visualizer.cpp src/value_grid_evaluator.cpp src/value_grid.cpp src/motion_selector_utils.cpp src/depth_image_collision_evaluator.cpp)


add_executable( motion_selector_node src/motion_selector_node.cpp )
target_link_libraries( motion_selector_node motion_selector ${catkin_LIBRARIES} ${PCL_LIBRARIES} orocos-kdl)

add_executable( state_estimate_corruptor_node src/experimental/state_estimate_corruptor_node.cpp )
target_link_libraries( state_estimate_corruptor_node  ${catkin_LIBRARIES})

