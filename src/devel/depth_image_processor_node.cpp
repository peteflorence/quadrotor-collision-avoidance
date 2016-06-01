#include "ros/ros.h"
#include <tf/transform_listener.h>
#include <iostream>
#include "depth_image_processor.h"
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>



int main(int argc, char **argv)
{

  ros::init(argc, argv, "tf_listener");
  
  
  tf::TransformListener listener;

  ros::Rate loop_rate(30);


  int count = 0;
  while (ros::ok())
  {
    
    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }


  return 0;
}



