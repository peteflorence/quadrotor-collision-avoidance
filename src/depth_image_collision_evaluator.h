#include <iostream>
#include <math.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "trajectory.h"

#include <chrono>

class DepthImageCollisionEvaluator {
public:
	DepthImageCollisionEvaluator() {
		K << 142.58555603027344, 0.0, 79.5, 0.0, 142.58555603027344, 59.5, 0.0, 0.0, 1.0;
		std::cout << "K is " << K << std::endl;
		std::cout << "sigma_depth_point is " << sigma_depth_point << std::endl;

	}
	
  void UpdatePointCloudPtr(pcl::PointCloud<pcl::PointXYZ>::Ptr const& xyz_cloud_new);

  double computeProbabilityOfCollisionOnePosition(Vector3 const& robot_position, Vector3 const& sigma_robot_position);
  double computeProbabilityOfCollisionOnePositionBlock(Vector3 const& robot_position, Vector3 const& sigma_robot_position, size_t const& block_increment);

  bool IsNoReturn(pcl::PointXYZ point);

  Eigen::Matrix<Scalar, 100, 3> DebugPointsToDraw();
  

private:
  pcl::PointCloud<pcl::PointXYZ>::Ptr xyz_cloud_ptr;

  Vector3 sigma_depth_point = Vector3(0.1, 0.1, 0.1);

  Eigen::Matrix<double, 3, 3> K;

  double probability_of_collision_in_unknown = 0.05;

};