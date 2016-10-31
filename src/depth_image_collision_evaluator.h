#include "motion.h"
#include "kd_tree.h"

#include "nanoflann.hpp"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <math.h>
#include <chrono>
#include <algorithm> 

class DepthImageCollisionEvaluator {
public:
	DepthImageCollisionEvaluator() {
		//K << 304.8, 0.0, 160.06, 0.0, 304.8, 119.85, 0.0, 0.0, 1.0;
                K << 308.57684326171875, 0.0, 154.6868438720703, 0.0, 308.57684326171875, 120.21442413330078, 0.0, 0.0, 1.0;
                K/=4.0;
                K(2,2) = 1.0;
	}
	
  void UpdatePointCloudPtr(pcl::PointCloud<pcl::PointXYZ>::Ptr const& xyz_cloud_new);
  void UpdateLaserPointCloudPtr(pcl::PointCloud<pcl::PointXYZ>::Ptr const& xyz_cloud_new);
  void UpdateRotationMatrix(Matrix3 const R);

  bool computeDeterministicCollisionOnePositionKDTree(Vector3 const& robot_position);

  bool IsBehind(Vector3 robot_position);
  bool IsOutsideDeadBand(Vector3 robot_position);
  double IsOutsideFOV(Vector3 robot_position);
  double AddOutsideFOVPenalty(Vector3 robot_position, double probability_of_collision);
  
  double computeProbabilityOfCollisionNPositionsKDTree_DepthImage(Vector3 const& robot_position, Vector3 const& sigma_robot_position);
  double computeProbabilityOfCollisionNPositionsKDTree_Laser(Vector3 const& robot_position, Vector3 const& sigma_robot_position);
  double computeProbabilityOfCollisionNPositionsKDTree(Vector3 const& robot_position, Vector3 const& sigma_robot_position, std::vector<pcl::PointXYZ> const& closest_pts);

private:
  pcl::PointCloud<pcl::PointXYZ>::Ptr xyz_cloud_ptr;
  pcl::PointCloud<pcl::PointXYZ>::Ptr xyz_laser_cloud_ptr;

  Vector3 sigma_depth_point = Vector3(0.01, 0.01, 0.01);

  Matrix3 K;
  double num_x_pixels = 320/4.0;
  double num_y_pixels = 240/4.0;

  KDTree<double> my_kd_tree_depth_image;
  KDTree<double> my_kd_tree_laser;

  Matrix3 R; //rotation matrix from ortho_body frame into camera rdf frame

};
