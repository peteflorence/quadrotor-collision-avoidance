#include "motion_selector_utils.h"

geometry_msgs::PoseStamped PoseFromVector3(Vector3 const& position, std::string const& frame) {
	geometry_msgs::PoseStamped pose;
	pose.pose.position.x = position(0);
	pose.pose.position.y = position(1);
	pose.pose.position.z = position(2);
	pose.header.frame_id = frame;
	pose.header.stamp = ros::Time::now();
	return pose;
}

Eigen::Vector3d VectorFromPose(geometry_msgs::PoseStamped const& pose) {
	return Vector3(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
}

Eigen::Vector3d VectorFromPoseUnstamped(geometry_msgs::Pose const& pose) {
	return Vector3(pose.position.x, pose.position.y, pose.position.z);
}