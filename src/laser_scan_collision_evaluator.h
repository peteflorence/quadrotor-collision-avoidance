#include <iostream>
#include <math.h>

#include <sensor_msgs/PointCloud2.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "trajectory.h"

#include <chrono>

class LaserScanCollisionEvaluator {
public:
	
  void updatePointCloud(const sensor_msgs::PointCloud2ConstPtr& point_cloud_msg);
  //double computeProbabilityOfCollisionOnePosition(Vector3 const& robot_position, Vector3 const& sigma_robot_position);
  

private:
  pcl::PointCloud<pcl::PointXYZ>::Ptr xyz_cloud;

  Vector3 sigma_depth_point = Vector3(0.2, 0.2, 0.2);

};