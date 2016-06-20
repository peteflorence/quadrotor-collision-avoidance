#include "laser_scan_collision_evaluator.h"


void LaserScanCollisionEvaluator::UpdatePointCloudPtr(pcl::PointCloud<pcl::PointXYZ>::Ptr const& xyz_cloud_new) {
	//auto t1 = std::chrono::high_resolution_clock::now();

	xyz_cloud = xyz_cloud_new;

	//pcl::PointXYZ first_point = xyz_cloud->at(10,50);

	//std::cout << "I got the point cloud, converted it to x,y,z and for pixel 10,50 it is: " << first_point.x << ", " << first_point.y << std::endl;

	// auto t2 = std::chrono::high_resolution_clock::now();
	// std::cout << "Converting and saving the point cloud took "
 //      << std::chrono::duration_cast<std::chrono::microseconds>(t2-t1).count()
 //      << " microseconds\n"; 

}






// double DepthImageCollisionEvaluator::computeProbabilityOfCollisionOnePosition(Vector3 const& robot_position, Vector3 const& sigma_robot_position) {
// 	Vector3 projected = K * robot_position;
// 	int pi_x = projected(0)/projected(2); 
// 	int pi_y = projected(1)/projected(2);
// 	//std::cout << "pi_x and pi_y were " << pi_x << " " << pi_y << std::endl;

// 	if (pi_x < 0 || pi_x > 159) {
// 		//std::cout << "This position is outside of my FOV (to the sides)" << std::endl;
// 		return 0.0;
// 	}
// 	else if (pi_y < 0 || pi_y > 119) {
// 		//std::cout << "This position is outside of my FOV (above / below)" << std::endl;
// 		return 0.0;
// 	}

// 	pcl::PointXYZ first_point = xyz_cloud->at(pi_x,pi_y);
	
// 	if (isnan(first_point.x)) {
// 		return 0.0;
// 	}

// 	Vector3 depth_position = Vector3(first_point.x, first_point.y, first_point.z);

// 	Vector3 total_sigma = sigma_robot_position + sigma_depth_point;
// 	Vector3 inverse_total_sigma = Vector3(1/total_sigma(0), 1/total_sigma(1), 1/total_sigma(2));	
	
// 	double volume = 0.267*2; // 4/3*pi*r^3, with r=0.4 as first guess
//   double denominator = std::sqrt( 248.05021344239853*(total_sigma(0))*(total_sigma(1))*(total_sigma(2)) ); // coefficient is 2pi*2pi*2pi
//   double exponent = -0.5*(robot_position - depth_position).transpose() * inverse_total_sigma.cwiseProduct(robot_position - depth_position);

//   return volume / denominator * std::exp(exponent);

// }