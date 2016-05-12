#include "ros/ros.h"
#include <tf/transform_listener.h>
#include <iostream>


int main(int argc, char **argv)
{

  ros::init(argc, argv, "tf_listener");
  tf::TransformListener listener;



  ros::NodeHandle n;

  //ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);

  ros::Rate loop_rate(10);


  int count = 0;
  while (ros::ok())
  {
    tf::StampedTransform transform;
    try {
      listener.lookupTransform("/world", "/rdf_world", ros::Time(0), transform);
    }
    catch (tf::TransformException ex) {
      std::cout << "didn't worky" << std::endl;
    }

    std::cout << transform.getRotation().length()  << std::endl;

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }


  return 0;
}