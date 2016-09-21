#include "depth_image_collision_evaluator.h"

#define num_nearest_neighbors 1

void DepthImageCollisionEvaluator::UpdatePointCloudPtr(pcl::PointCloud<pcl::PointXYZ>::Ptr const& xyz_cloud_new) {
	xyz_cloud_ptr = xyz_cloud_new;
  my_kd_tree_depth_image.Initialize(xyz_cloud_ptr);
}

void DepthImageCollisionEvaluator::UpdateLaserPointCloudPtr(pcl::PointCloud<pcl::PointXYZ>::Ptr const& xyz_cloud_new) {
  xyz_laser_cloud_ptr = xyz_cloud_new;
  my_kd_tree_laser.Initialize(xyz_laser_cloud_ptr);
}

bool DepthImageCollisionEvaluator::computeDeterministicCollisionOnePositionKDTree(Vector3 const& robot_position) {
  if (robot_position(2) < -1.0) {
    return true;
  }
  my_kd_tree_depth_image.SearchForNearest<1>(robot_position[0], robot_position[1], robot_position[2]);
  if (my_kd_tree_depth_image.squared_distances.size() > 0) {
    if (my_kd_tree_depth_image.squared_distances[0] < 2.0) {
      return true;
    }
  }
  return false;
}

double ThresholdSigmoid(double value) {
    double sigmoid_threshold = 0.99;
    if (value > sigmoid_threshold) { 
      double sigmoid = 1.0 / (1.0 + exp(-value));
      return sigmoid_threshold + (1 - sigmoid_threshold) * sigmoid;
    }
    if (value < 0.0) { 
      return 0.0;
    }
    return value;
}

double ThresholdHard(double value) {
    double hard_threshold = 0.9;
    if (value > hard_threshold) { 
      return hard_threshold;
    }
    if (value < 0.0) { 
      return 0.0;
    }
    return value;
}

bool DepthImageCollisionEvaluator::IsBehind(Vector3 robot_position) {
    return (robot_position(2) < -0.5);
}

bool DepthImageCollisionEvaluator::IsOutsideDeadBand(Vector3 robot_position) {
    return (robot_position.squaredNorm() > 0.5);
}

bool DepthImageCollisionEvaluator::IsOutsideFOV(Vector3 robot_position) {
    Vector3 projected = K * robot_position;
    int pi_x = projected(0)/projected(2); 
    int pi_y = projected(1)/projected(2);

    if ( (pi_x < 0) || (pi_x > (num_x_pixels - 1)) ) {
        return true;
    }
    return false;
    // else if (pi_y < 0) { // ignore if it's under because this is preventing me from slowing down
    //   return true;
    // }
    // else if (robot_position(2) > 10.0) {
    //   return true;
    // }
}

double DepthImageCollisionEvaluator::AddOutsideFOVPenalty(Vector3 robot_position, double probability_of_collision) {
    if (IsBehind(robot_position)) {
      return probability_of_collision += 0.5;
    }
    if (IsOutsideDeadBand(robot_position)) {
      if (IsOutsideFOV(robot_position)) {
          return probability_of_collision += 0.5;
      }
    }
    return probability_of_collision;
}

double DepthImageCollisionEvaluator::computeProbabilityOfCollisionNPositionsKDTree_DepthImage(Vector3 const& robot_position, Vector3 const& sigma_robot_position) {
  double probability_of_collision = 0.0;
  if (xyz_cloud_ptr != nullptr) {
    my_kd_tree_depth_image.SearchForNearest<num_nearest_neighbors>(robot_position[0], robot_position[1], robot_position[2]);
    probability_of_collision = computeProbabilityOfCollisionNPositionsKDTree(robot_position, sigma_robot_position, my_kd_tree_depth_image.closest_pts);
  }
  probability_of_collision = AddOutsideFOVPenalty(robot_position, probability_of_collision);
  return ThresholdHard(probability_of_collision);
}


double DepthImageCollisionEvaluator::computeProbabilityOfCollisionNPositionsKDTree_Laser(Vector3 const& robot_position, Vector3 const& sigma_robot_position) {
  if (xyz_laser_cloud_ptr != nullptr) {
    my_kd_tree_laser.SearchForNearest<num_nearest_neighbors>(robot_position[0], robot_position[1], robot_position[2]);
    double probability_of_collision = computeProbabilityOfCollisionNPositionsKDTree(robot_position, sigma_robot_position, my_kd_tree_laser.closest_pts);
    return ThresholdHard(probability_of_collision);
  }
  return 0.0;
}

double DepthImageCollisionEvaluator::computeProbabilityOfCollisionNPositionsKDTree(Vector3 const& robot_position, Vector3 const& sigma_robot_position, std::vector<pcl::PointXYZ> const& closest_pts) {
  double probability_no_collision = 1.0;
  
  if (closest_pts.size() > 0) {
    for (size_t i = 0; i < std::min((int)closest_pts.size(), num_nearest_neighbors); i++) {

      pcl::PointXYZ first_point = closest_pts[i];
      Vector3 depth_position = Vector3(first_point.x, first_point.y, first_point.z);

      Vector3 total_sigma = sigma_robot_position + sigma_depth_point;
      Vector3 inverse_total_sigma = Vector3(1/total_sigma(0), 1/total_sigma(1), 1/total_sigma(2));  
    
      double volume = 0.267; // 4/3*pi*r^3, with r=0.4 as first guess
      double denominator = std::sqrt( 248.05021344239853*(total_sigma(0))*(total_sigma(1))*(total_sigma(2)) ); // coefficient is 2pi*2pi*2pi
      double exponent = -0.5*(robot_position - depth_position).transpose() * inverse_total_sigma.cwiseProduct(robot_position - depth_position);

      double probability_of_collision = volume / denominator * std::exp(exponent);

      probability_no_collision = probability_no_collision * (1 - probability_of_collision);
    }
    return 1 - probability_no_collision;
  }
  return 0.0; // if no points in closest_pts
}