cmake_minimum_required(VERSION 2.8.3)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3")
project(depth_image_processor)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  std_msgs
  tf
  cv_bridge
  image_transport
)

find_package( Eigen3 REQUIRED )
find_package(OpenCV 2.4.8 REQUIRED)

include_directories ( src )
include_directories( ${EIGEN3_INCLUDE_DIR} )

catkin_package(

)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_library( depth_image_processor src/depth_image_processor.cpp)
add_executable( test_depth_image_processor src/test_depth_image_processor.cpp)
add_executable( dip_node src/depth_image_processor_node.cpp)
target_link_libraries( test_depth_image_processor depth_image_processor)
target_link_libraries( dip_node depth_image_processor)

target_link_libraries(depth_image_processor
  ${OpenCV_LIBS} ${catkin_LIBRARIES}
)
