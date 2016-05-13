#include "depth_image_processor.h"


void DepthImageProcessor::TestOpenCV(Eigen::Quaternionf const& q_b, cv::Point & corner1, cv::Point & corner2) {

  cv::Mat img_a(480, 640, cv::DataType<float>::type);
  cv::Mat img_b(480, 640, cv::DataType<float>::type);
  cv::Mat debug(480, 640, cv::DataType<uint8_t>::type);

  // Need to choose size of img_a and img_b
  debug = cv::Scalar(0);

  Eigen::Quaternionf q_a;

  q_a.setIdentity();
  //X axis rotation is pitch
  //Y axis is yaw
  //Z axis is roll
  //q_b = Eigen::AngleAxis<float>(-10.0 * M_PI / 180.0, Eigen::Vector3f(1,0,0)) * Eigen::AngleAxis<float>(-10.0 * M_PI / 180.0, Eigen::Vector3f(0,0,1)) ;

  Eigen::Matrix3f K_b;
  K_b.setIdentity();
  K_b << 525,   0, 319.5,
           0, 525, 239.5,
           0,   0,     1;

  // Need parameters of xtion
  // Project corners of image b into rectified camera.
  std::vector<cv::Point2f> corners(2);
  corners[0] = cv::Point2f(0.0f, 239.0f);
  corners[1] = cv::Point2f(639.0f, 239.0f);

  std::vector<cv::Point2f> new_corners(2);
  Eigen::Quaternionf q_b_to_a(q_a.inverse() * q_b);
  Eigen::Matrix3f P(K_b * q_b_to_a.toRotationMatrix() * K_b.inverse());
  Eigen::Vector3f u_ii;
  for (int ii = 0; ii < corners.size(); ++ii) {
    u_ii << corners[ii].x, corners[ii].y, 1.0f;
    u_ii = P * u_ii;
    u_ii /= u_ii(2);

    new_corners[ii].x = u_ii(0);
    new_corners[ii].y = u_ii(1);
  }


  //cv::line(debug, )
  //cv::Point corner1(), corner2((int)new_corners[1].x, (int)new_corners[1].y);

  corner1 = cv::Point((int)new_corners[0].x, (int)new_corners[0].y);
  corner2 = cv::Point((int)new_corners[1].x, (int)new_corners[1].y);
  std::cout << "Here are my new_corners " << new_corners << std::endl;  

}