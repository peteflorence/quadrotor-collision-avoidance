#include "depth_image_processor.h"

void DepthImageProcessor::TestOpenCV() {

  cv::Mat img_a, img_b;
  Eigen::Quaternionf q_a, q_b;
  Eigen::Matrix3f K_b;

  // Project corners of image b into rectified camera.
  std::vector<cv::Point2f> corners(4);
  corners[0] = cv::Point2f(0.0f, 0.0f);
  corners[1] = cv::Point2f(img_b.cols, 0.0f);
  corners[2] = cv::Point2f(img_b.cols, img_b.rows);
  corners[3] = cv::Point2f(0.0f, img_b.rows);

  std::vector<cv::Point2f> new_corners(4);
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