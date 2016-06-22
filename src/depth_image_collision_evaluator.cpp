#include "depth_image_collision_evaluator.h"


void DepthImageCollisionEvaluator::UpdatePointCloudPtr(pcl::PointCloud<pcl::PointXYZ>::Ptr const& xyz_cloud_new) {
	//auto t1 = std::chrono::high_resolution_clock::now();
	xyz_cloud_ptr = xyz_cloud_new;
	// auto t2 = std::chrono::high_resolution_clock::now();
	// std::cout << "Converting and saving the point cloud took "
 //      << std::chrono::duration_cast<std::chrono::microseconds>(t2-t1).count()
 //      << " microseconds\n"; 
}



double DepthImageCollisionEvaluator::computeProbabilityOfCollisionOnePosition(Vector3 const& robot_position, Vector3 const& sigma_robot_position) {
  if (xyz_cloud_ptr != nullptr) {

    Vector3 projected = K * robot_position;
    int pi_x = projected(0)/projected(2); 
    int pi_y = projected(1)/projected(2);
    //std::cout << "pi_x and pi_y were " << pi_x << " " << pi_y << std::endl;

    if (pi_x < 0 || pi_x > 159) {
      //std::cout << "This position is outside of my FOV (to the sides)" << std::endl;
      return probability_of_collision_in_unknown;
    }
    else if (pi_y < 0 || pi_y > 119) {
      //std::cout << "This position is outside of my FOV (above / below)" << std::endl;
      return probability_of_collision_in_unknown;
    }

    pcl::PointXYZ first_point = xyz_cloud_ptr->at(pi_x,pi_y);
    
    if (IsNoReturn(first_point)) {
      return 0.0;
    }
    if (first_point.z < -0.2) { 
      return probability_of_collision_in_unknown;
    }

    Vector3 depth_position = Vector3(first_point.x, first_point.y, first_point.z);

    Vector3 total_sigma = sigma_robot_position + sigma_depth_point;
    Vector3 inverse_total_sigma = Vector3(1/total_sigma(0), 1/total_sigma(1), 1/total_sigma(2));  
    
    double volume = 0.267*2; // 4/3*pi*r^3, with r=0.4 as first guess
    double denominator = std::sqrt( 248.05021344239853*(total_sigma(0))*(total_sigma(1))*(total_sigma(2)) ); // coefficient is 2pi*2pi*2pi
    double exponent = -0.5*(robot_position - depth_position).transpose() * inverse_total_sigma.cwiseProduct(robot_position - depth_position);

    double val = volume / denominator * std::exp(exponent);
    if (isnan(val)) {
      std::cout << "PRINTING EVERYTHING " << std::endl;
      std::cout << "denom " << denominator << std::endl;
      std::cout << "exp " << exponent << std::endl;
      std::cout << std::endl;
      std::cout << "depth_position " << depth_position << std::endl;
      std::cout << "robot_position " << robot_position << std::endl;

    }
    return val;
  }
  std::cout << "PTR WAS NULL" << std::endl;
  // ptr was null
  return 0.0;

}

double DepthImageCollisionEvaluator::computeProbabilityOfCollisionOnePositionBlock(Vector3 const& robot_position, Vector3 const& sigma_robot_position, size_t const& block_increment) {
  if (xyz_cloud_ptr != nullptr) {
    pcl::PointXYZ first_point;

    // block_increment of 1 gives a 3x3
    // block_increment of 2 gives a 5x5

    double probability_no_collision = 1;

    Vector3 projected = K * robot_position;
    int pi_x = projected(0)/projected(2); 
    int pi_y = projected(1)/projected(2);
    //std::cout << "pi_x and pi_y were " << pi_x << " " << pi_y << std::endl;

    if (pi_x < 0 || pi_x > 159) {
      //std::cout << "This position is outside of my FOV (to the sides)" << std::endl;
      return probability_of_collision_in_unknown;
    }
    else if (pi_y < 0 || pi_y > 119) {
      //std::cout << "This position is outside of my FOV (above / below)" << std::endl;
      return probability_of_collision_in_unknown;
    }

    first_point = xyz_cloud_ptr->at(pi_x,pi_y);
    if (first_point.z < -0.2) { 
      return probability_of_collision_in_unknown;
    }


    Vector3 total_sigma = sigma_robot_position + sigma_depth_point;
    Vector3 inverse_total_sigma = Vector3(1/total_sigma(0), 1/total_sigma(1), 1/total_sigma(2));
    double volume = 0.267; // 4/3*pi*r^3, with r=0.4 as first guess
    double denominator = std::sqrt( 248.05021344239853*(total_sigma(0))*(total_sigma(1))*(total_sigma(2)) ); // coefficient is 2pi*2pi*2pi

    for (int i = pi_x - block_increment; i < pi_x + block_increment + 1; i++) {
      for (int j = pi_y - block_increment; j < pi_y + block_increment + 1; j++) {
        if ((i < 0 || i > 159) || (j < 0 || j > 119)) {
          continue;
        }
        first_point = xyz_cloud_ptr->at(i,j);
        if (IsNoReturn(first_point)) { 
          continue;
        }
        // if (first_point.y > 1.0) { // this makes it so the ground doesn't count // NEED TO DO THIS BETTER
        //   continue;
        // }
        

        Vector3 depth_position = Vector3(first_point.x, first_point.y, first_point.z);
        double exponent = -0.5*(robot_position - depth_position).transpose() * inverse_total_sigma.cwiseProduct(robot_position - depth_position);
        probability_no_collision = probability_no_collision* (1 - volume / denominator * std::exp(exponent));
      }
    }
    
    return 1 - probability_no_collision;
  }
  // ptr was null
  return 0.0;

}


// THIS IS ONLY NEEDED IN SIM VERSION. OTHERWISE JUST USE isnan()
bool DepthImageCollisionEvaluator::IsNoReturn(pcl::PointXYZ point) {
  if (isnan(point.x)) {
    return true;
  }
  if (point.x*point.x + point.y*point.y + point.z*point.z < 0.2) {
    return true;
  }
  return false;
}

Eigen::Matrix<Scalar, 100, 3> DepthImageCollisionEvaluator::DebugPointsToDraw() {

	Eigen::Matrix<Scalar, 100, 3> points_to_draw;
	points_to_draw.setZero();

	if (xyz_cloud_ptr != nullptr) {

		auto point_cloud_iterator_begin = xyz_cloud_ptr->begin();
  		auto point_cloud_iterator_end = xyz_cloud_ptr->end();
  		int i = 0;
  		for (auto point = point_cloud_iterator_begin; point != point_cloud_iterator_end; point++) {
  			Vector3 depth_position = Vector3(point->x, point->y, point->z);
  			points_to_draw.row(i) = depth_position;
  			i++;
        if (i == 99) {break;}
  		}
	}
	// Ptr was null
	return points_to_draw;

};