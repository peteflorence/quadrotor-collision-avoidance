#include "ros/ros.h"
#include <tf/transform_listener.h>
#include <iostream>
#include "depth_image_processor.h"


int main(int argc, char **argv)
{

  ros::init(argc, argv, "tf_listener");
  tf::TransformListener listener;

  DepthImageProcessor dip;



  ros::NodeHandle n;

  //ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);

  ros::Rate loop_rate(10);


  int count = 0;
  while (ros::ok())
  {
    tf::StampedTransform transform;
    try {
      listener.lookupTransform("/xtion_depth_optical_frame", "/rdf_world", ros::Time(0), transform);
    }
    catch (tf::TransformException ex) {
      std::cout << "didn't work" << std::endl;
      std::cout << ex.what() << std::endl;
    }

    std::cout << transform.getRotation()[0] << " " << transform.getRotation()[1]  << " " << transform.getRotation()[2]<< " " << transform.getRotation()[3] << std::endl;

    auto t = transform.getRotation(); 
    Eigen::Quaternionf e = Eigen::Quaternionf(t[3],t[0],t[1],t[2]);

    dip.TestOpenCV(e);
    

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }


  return 0;
}

