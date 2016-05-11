#include "depth_image_processor.h"

void DepthImageProcessor::TestOpenCV() {

  cv::Mat img_a(640, 480, cv::DataType<float>::type);
  cv::Mat img_b(640, 480, cv::DataType<float>::type);
  // Need to choose size of img_a and img_b

  Eigen::Quaternionf q_a, q_b;
  q_a.setIdentity();

  //q_b = Eigen::AngleAxis<float>(1.0 * M_PI / 180.0, Eigen::Vector3f::UnitX());

  q_b.setIdentity();
  // Need to make q_a the identity
  // Need to set q_b to be pitch and roll of the vehicle

  Eigen::Matrix3f K_b;
  K_b.setIdentity();
  K_b << 1, 0, 320, 0, 1, 240, 0, 0, 1; 
  // Need parameters of xtion

  // Project corners of image b into rectified camera.
  std::vector<cv::Point2f> corners(2);
  corners[0] = cv::Point2f(240.0f, 0.0f);
  corners[1] = cv::Point2f(240.0f, 639.0f);

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

  std::cout << "Here are my new_corners " << new_corners << std::endl;  

}