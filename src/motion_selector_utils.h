#ifndef MOTION_SELECTOR_UTILS_H
#define MOTION_SELECTOR_UTILS_H

#include "motion.h"
#include "geometry_msgs/PoseStamped.h"

geometry_msgs::PoseStamped PoseFromVector3(Vector3 const& position, std::string const& frame);
Eigen::Vector3d VectorFromPose(geometry_msgs::PoseStamped const& pose);
Eigen::Vector3d VectorFromPoseUnstamped(geometry_msgs::Pose const& pose);

#endif