#include "ros/ros.h"
#include <tf/transform_listener.h>
#include <iostream>
#include "depth_image_processor.h"
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

tf::TransformListener * listener_ptr;

void imageCb(const sensor_msgs::ImageConstPtr& msg)
{

    DepthImageProcessor dip;
    std::cout << "I'm inside the callback" << std::endl;
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      ROS_WARN("I am getting the callback");
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    tf::StampedTransform transform;
    try {
      listener_ptr->lookupTransform("/xtion_depth_optical_frame", "/rdf_world", ros::Time(0), transform);
    }
    catch (tf::TransformException ex) {
      std::cout << "didn't work" << std::endl;
      std::cout << ex.what() << std::endl;
    }

 
    std::cout << transform.getRotation()[0] << " " << transform.getRotation()[1]  << " " << transform.getRotation()[2]<< " " << transform.getRotation()[3] << std::endl;
    auto t = transform.getRotation(); 
    Eigen::Quaternionf e = Eigen::Quaternionf(t[3],t[0],t[1],t[2]);

    cv::Point corner1, corner2;
    dip.TestOpenCV(e, corner1, corner2);

    cv::namedWindow("cv_ptr", cv::WINDOW_NORMAL);
    cv::resizeWindow("cv_ptr", 1920, 1080);

    cv::clipLine(cv_ptr->image.size(), corner1, corner2);

    if (corner1.x > 5) {
      double m = ( corner2.y - corner1.y ) / (corner2.x - corner2.y);
      corner1.y = corner1.y - m * (corner1.x - 0.0) ;
      corner1.x = 0;
    }

    cv::line(cv_ptr->image, corner1, corner2, cv::Scalar(0xffff));
    //cv::line(cv_ptr->image, cv::Point(0,0), cv::Point(100,100), cv::Scalar(0xffff));
    cv::imshow("cv_ptr", cv_ptr->image);
    cv::waitKey(10);

}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "tf_listener");
  
  
  tf::TransformListener listener;
  listener_ptr = &listener;
  
  ros::NodeHandle n;

  image_transport::ImageTransport it_(n);
  image_transport::Subscriber image_sub_;

  image_sub_ = it_.subscribe("/flight/xtion_depth/image_raw", 1, imageCb);

  //ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);

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



