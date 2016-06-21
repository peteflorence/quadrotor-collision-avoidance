#include <iostream>
#include <math.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "trajectory.h"

#include <chrono>

class LaserScanCollisionEvaluator {
public:
	
  void UpdatePointCloudPtr(pcl::PointCloud<pcl::PointXYZ>::Ptr const& xyz_cloud_new);
  double computeProbabilityOfCollisionOnePosition(Vector3 const& robot_position, Vector3 const& sigma_robot_position);
  Eigen::Matrix<Scalar, 100, 3> DebugPointsToDraw();
  

private:
  pcl::PointCloud<pcl::PointXYZ>::Ptr xyz_cloud_ptr;

  Vector3 sigma_depth_point = Vector3(0.2, 0.2, 0.2);

};