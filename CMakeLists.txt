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

add_library( trajectory_selector src/trajectory_selector.cpp )
add_executable( test_trajectory_selector src/test_trajectory_selector.cpp )
target_link_libraries( test_trajectory_selector trajectory_selector )

add_library( trajectory_library src/trajectory_library.cpp )
target_link_libraries( trajectory_selector trajectory_library )

add_library( trajectory_evaluator src/trajectory_evaluator.cpp )
target_link_libraries( trajectory_selector trajectory_evaluator )

add_library( trajectory src/trajectory.cpp )
target_link_libraries( trajectory_library trajectory )

add_library( attitude_generator src/attitude_generator.cpp )
target_link_libraries( attitude_generator trajectory )


add_executable( trajectory_selector_node src/trajectory_selector_node.cpp )
target_link_libraries( trajectory_selector_node trajectory_selector )
target_link_libraries( trajectory_selector_node ${catkin_LIBRARIES} )
target_link_libraries( trajectory_selector_node ${PCL_LIBRARIES})
target_link_libraries( trajectory_selector_node attitude_generator )
target_link_libraries( trajectory_selector_node orocos-kdl )



##add_library( depth_image_processor src/devel/depth_image_processor.cpp)
##add_executable( test_depth_image_processor src/devel/test_depth_image_processor.cpp)
##add_executable( dip_node src/devel/depth_image_processor_node.cpp)
##target_link_libraries( test_depth_image_processor depth_image_processor)
##target_link_libraries( dip_node depth_image_processor)

target_link_libraries(trajectory_selector
  ${OpenCV_LIBS} ${catkin_LIBRARIES}
)
