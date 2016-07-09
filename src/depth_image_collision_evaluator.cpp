#include "depth_image_collision_evaluator.h"


void DepthImageCollisionEvaluator::UpdatePointCloudPtr(pcl::PointCloud<pcl::PointXYZ>::Ptr const& xyz_cloud_new) {
	
	xyz_cloud_ptr = xyz_cloud_new;
  // uncomment for kd-tree version
  BuildKDTree();
  
}

void DepthImageCollisionEvaluator::BuildKDTree() {
  // auto t1 = std::chrono::high_resolution_clock::now();
  my_kd_tree.Initialize(xyz_cloud_ptr);
  // auto t2 = std::chrono::high_resolution_clock::now();
  // std::cout << "Building kd-tree took "
  //     << std::chrono::duration_cast<std::chrono::microseconds>(t2-t1).count()
  //     << " microseconds\n";  
}
 

bool DepthImageCollisionEvaluator::computeDeterministicCollisionOnePositionKDTree(Vector3 const& robot_position, Vector3 const& sigma_robot_position) {

  if (robot_position(2) < -1.0) {
    return true;
  }

  my_kd_tree.SearchForNearest<1>(robot_position[0], robot_position[1], robot_position[2], closest_pts, squared_distances);
  if (squared_distances.size() > 0) {
    if (squared_distances[0] < 1.0) {
      return true;
    }
  }
  return false;

}

double DepthImageCollisionEvaluator::computeProbabilityOfCollisionNPositionsKDTree(Vector3 const& robot_position, Vector3 const& sigma_robot_position) {
  size_t const n = 1;
  if (xyz_cloud_ptr != nullptr) {
    double probability_no_collision = 1.0;
    Vector3 projected = K * robot_position;
    int pi_x = projected(0)/projected(2); 
    int pi_y = projected(1)/projected(2);

    if (robot_position(2) < -1.0) {
      return 0.9;
    }

    my_kd_tree.SearchForNearest<n>(robot_position[0], robot_position[1], robot_position[2], closest_pts, squared_distances);
    if (closest_pts.size() > 0) {
      for (size_t i = 0; i < std::min(closest_pts.size(), n); i++) {


        pcl::PointXYZ first_point = closest_pts[i];
        Vector3 depth_position = Vector3(first_point.x, first_point.y, first_point.z);

        Vector3 total_sigma = sigma_robot_position + sigma_depth_point;
        Vector3 inverse_total_sigma = Vector3(1/total_sigma(0), 1/total_sigma(1), 1/total_sigma(2));  
      
        double volume = 0.267; // 4/3*pi*r^3, with r=0.4 as first guess
        double denominator = std::sqrt( 248.05021344239853*(total_sigma(0))*(total_sigma(1))*(total_sigma(2)) ); // coefficient is 2pi*2pi*2pi
        double exponent = -0.5*(robot_position - depth_position).transpose() * inverse_total_sigma.cwiseProduct(robot_position - depth_position);

        double probability_of_collision = volume / denominator * std::exp(exponent);

        if (robot_position.squaredNorm() > 0.5) {
      
          if (pi_x < 0 || pi_x > 159) {
            probability_of_collision += 0.5;
          }
          else if (pi_y < 0) { // ignore if it's under because this is preventing me from slowing down
            probability_of_collision += 0.5;
          }
        }
        probability_no_collision = probability_no_collision * (1 - probability_of_collision);
      }
      return 1 - probability_no_collision;
    }
    return 0.0; // if no points in closest_pts
  }
  return 0.0; // if ptr was null
}


double DepthImageCollisionEvaluator::computeProbabilityOfCollisionOnePosition(Vector3 const& robot_position, Vector3 const& sigma_robot_position) {
  if (xyz_cloud_ptr != nullptr) {

    Vector3 projected = K * robot_position;
    int pi_x = projected(0)/projected(2); 
    int pi_y = projected(1)/projected(2);

    if (pi_x < 0 || pi_x > 159) {
      return probability_of_collision_in_unknown;
    }
    else if (pi_y < 0 || pi_y > 119) {
      return probability_of_collision_in_unknown;
    }

    pcl::PointXYZ first_point = xyz_cloud_ptr->at(pi_x,pi_y);
    
    if (IsNoReturn(first_point)) {
      return 0.0;
    }

    Vector3 depth_position = Vector3(first_point.x, first_point.y, first_point.z);

    Vector3 total_sigma = sigma_robot_position + sigma_depth_point;
    Vector3 inverse_total_sigma = Vector3(1/total_sigma(0), 1/total_sigma(1), 1/total_sigma(2));  
    
    double volume = 0.267; // 4/3*pi*r^3, with r=0.4 as first guess
    double denominator = std::sqrt( 248.05021344239853*(total_sigma(0))*(total_sigma(1))*(total_sigma(2)) ); // coefficient is 2pi*2pi*2pi
    double exponent = -0.5*(robot_position - depth_position).transpose() * inverse_total_sigma.cwiseProduct(robot_position - depth_position);
 
    return volume / denominator * std::exp(exponent);
  }
  
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

    if (pi_x < 0 || pi_x > 159) {
      return probability_of_collision_in_unknown;
    }
    else if (pi_y < 0 || pi_y > 119) {
      return probability_of_collision_in_unknown;
    }

    first_point = xyz_cloud_ptr->at(pi_x,pi_y);

    Vector3 total_sigma = sigma_robot_position + sigma_depth_point;
    Vector3 inverse_total_sigma = Vector3(1/total_sigma(0), 1/total_sigma(1), 1/total_sigma(2));
    double volume = 0.267*2; // 4/3*pi*r^3, with r=0.4 as first guess
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

bool DepthImageCollisionEvaluator::IsNoReturn(pcl::PointXYZ point) {
  if (isnan(point.x)) {
    return true;
  }
  // THIS IS ONLY NEEDED IN SIM VERSION. OTHERWISE JUST USE isnan()
  // if (point.x*point.x + point.y*point.y + point.z*point.z < 0.2) {
  //   return true;
  // }
  return false;
}

double DepthImageCollisionEvaluator::computeProbabilityOfCollisionOnePositionBlockMarching(Vector3 const& robot_position, Vector3 const& sigma_robot_position, size_t const& block_increment) {
  if (xyz_cloud_ptr != nullptr) {
    pcl::PointXYZ first_point;

    // block_increment of 1 gives a 3x3
    // block_increment of 2 gives a 5x5

    double probability_no_collision = 1;

    Vector3 projected = K * robot_position;
    int pi_x = projected(0)/projected(2); 
    int pi_y = projected(1)/projected(2);

    if (pi_x < 0 || pi_x > 159) {
      return probability_of_collision_in_unknown;
    }
    else if (pi_y < 0 || pi_y > 119) {
      return probability_of_collision_in_unknown;
    }

    first_point = xyz_cloud_ptr->at(pi_x,pi_y);

    Vector3 total_sigma = sigma_robot_position + sigma_depth_point;
    Vector3 inverse_total_sigma = Vector3(1/total_sigma(0), 1/total_sigma(1), 1/total_sigma(2));
    double volume = 0.267; // 4/3*pi*r^3, with r=0.4 as first guess
    double denominator = std::sqrt( 248.05021344239853*(total_sigma(0))*(total_sigma(1))*(total_sigma(2)) ); // coefficient is 2pi*2pi*2pi

    size_t n = 0;
    size_t n_max = 10;

    // Check middle point
    if (!IsNoReturn(first_point)) { 
        Vector3 depth_position = Vector3(first_point.x, first_point.y, first_point.z);
        double exponent = -0.5*(robot_position - depth_position).transpose() * inverse_total_sigma.cwiseProduct(robot_position - depth_position);
        probability_no_collision = probability_no_collision* (1 - volume / denominator * std::exp(exponent));
    }

    int i = 0;
    int j = 0;

    for (size_t current_block = 1; current_block <= block_increment; current_block++) {
      // Start in upper left, go to upper right (but not upper right)
      j = pi_y - current_block;
      for (i = pi_x - current_block; i < pi_x + current_block; i++) {
        if ((i < 0 || i > 159) || (j < 0 || j > 119)) {
          continue;
        }
        first_point = xyz_cloud_ptr->at(i,j);
        if (IsNoReturn(first_point)) { 
          continue;
        }
        Vector3 depth_position = Vector3(first_point.x, first_point.y, first_point.z);
        double exponent = -0.5*(robot_position - depth_position).transpose() * inverse_total_sigma.cwiseProduct(robot_position - depth_position);
        probability_no_collision = probability_no_collision* (1 - volume / denominator * std::exp(exponent));
        n++;
      }
      if (n > n_max) {
        break;
      }

      // Start in upper right, go to bottom right (but not bottom right)
      i = pi_x + current_block;
      for (j = pi_y - current_block; j < pi_y + current_block; j++) {
        if ((i < 0 || i > 159) || (j < 0 || j > 119)) {
          continue;
        }
        first_point = xyz_cloud_ptr->at(i,j);
        if (IsNoReturn(first_point)) { 
          continue;
        }
        Vector3 depth_position = Vector3(first_point.x, first_point.y, first_point.z);
        double exponent = -0.5*(robot_position - depth_position).transpose() * inverse_total_sigma.cwiseProduct(robot_position - depth_position);
        probability_no_collision = probability_no_collision* (1 - volume / denominator * std::exp(exponent));
        n++;
      }
      if (n > n_max) {
        break;
      }

      // Start in bottom right, go to bottom left (but not bottom left)
      j = pi_y + current_block;
      for (i = pi_x + current_block; i > pi_x - current_block; i--) {
        if ((i < 0 || i > 159) || (j < 0 || j > 119)) {
          continue;
        }
        first_point = xyz_cloud_ptr->at(i,j);
        if (IsNoReturn(first_point)) { 
          continue;
        }
        Vector3 depth_position = Vector3(first_point.x, first_point.y, first_point.z);
        double exponent = -0.5*(robot_position - depth_position).transpose() * inverse_total_sigma.cwiseProduct(robot_position - depth_position);
        probability_no_collision = probability_no_collision* (1 - volume / denominator * std::exp(exponent));
        n++;
      }
      if (n > n_max) {
        break;
      }

      // Start in bottom left, go to upper left (but not top left)
      i = pi_x - current_block;
      for (j = pi_y + current_block; j > pi_y - current_block; j--) {
        if ((i < 0 || i > 159) || (j < 0 || j > 119)) {
          continue;
        }
        first_point = xyz_cloud_ptr->at(i,j);
        if (IsNoReturn(first_point)) { 
          continue;
        }
        Vector3 depth_position = Vector3(first_point.x, first_point.y, first_point.z);
        double exponent = -0.5*(robot_position - depth_position).transpose() * inverse_total_sigma.cwiseProduct(robot_position - depth_position);
        probability_no_collision = probability_no_collision* (1 - volume / denominator * std::exp(exponent));
        n++;
      }
      if (n > n_max) {
        break;
      }
          
    }
    
    return 1 - probability_no_collision;
  }
  // ptr was null
  return 0.0;

}


bool DepthImageCollisionEvaluator::computeDeterministicCollisionOnePositionBlock(Vector3 const& robot_position, Vector3 const& sigma_robot_position, size_t const& block_increment) {
  // returns 1.0 if collision detected
  // otherwise returns 0.0
  double buffer = 1.0;

  if (xyz_cloud_ptr != nullptr) {
    pcl::PointXYZ first_point;

    // block_increment of 1 gives a 3x3
    // block_increment of 2 gives a 5x5

    Vector3 projected = K * robot_position;
    int pi_x = projected(0)/projected(2); 
    int pi_y = projected(1)/projected(2);

    if (pi_x < 0 || pi_x > 159) {
      return false;
    }
    else if (pi_y < 0 || pi_y > 119) {
      return false;
    }

    first_point = xyz_cloud_ptr->at(pi_x,pi_y);

    for (int i = pi_x - block_increment; i < pi_x + block_increment + 1; i++) {
      for (int j = pi_y - block_increment; j < pi_y + block_increment + 1; j++) {
        if ((i < 0 || i > 159) || (j < 0 || j > 119)) {
          continue;
        }
        first_point = xyz_cloud_ptr->at(i,j);
        if (IsNoReturn(first_point)) { 
          continue;
        }

        // Check if in collision
        Vector3 depth_position = Vector3(first_point.x, first_point.y, first_point.z);
        if ( (depth_position-robot_position).squaredNorm() < buffer) {
          return true;
        }
      }
    }
    
    return false;
  }
  // ptr was null
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
