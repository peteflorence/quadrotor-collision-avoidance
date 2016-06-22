#include "depth_image_collision_evaluator.h"


void DepthImageCollisionEvaluator::UpdatePointCloudPtr(pcl::PointCloud<pcl::PointXYZ>::Ptr const& xyz_cloud_new) {
	//auto t1 = std::chrono::high_resolution_clock::now();
	xyz_cloud_ptr = xyz_cloud_new;
	// auto t2 = std::chrono::high_resolution_clock::now();
	// std::cout << "Converting and saving the point cloud took "
 //      << std::chrono::duration_cast<std::chrono::microseconds>(t2-t1).count()
 //      << " microseconds\n"; 
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