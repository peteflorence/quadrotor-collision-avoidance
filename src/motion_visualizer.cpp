#include "motion_visualizer.h"

void MotionVisualizer::initializeDrawingPaths() {
	for (int i = 0; i < motion_selector->getNumMotions(); i++) {
		action_paths_pubs.push_back(nh.advertise<nav_msgs::Path>("/poly_samples"+std::to_string(i), 1));
	}
}

void MotionVisualizer::UpdateTimeHorizon(double final_time) {
	this->final_time = final_time;
	createSamplingTimeVector();
}

void MotionVisualizer::createSamplingTimeVector() {
	num_samples = 10;
	sampling_time_vector.resize(num_samples, 1);

	double sampling_time = 0;
	double sampling_interval = (final_time - start_time) / num_samples;
	for (size_t sample_index = 0; sample_index < num_samples; sample_index++) {
		sampling_time = start_time + sampling_interval*(sample_index+1);
		sampling_time_vector(sample_index) = sampling_time;
	}
}

void MotionVisualizer::drawGaussianPropagation(int id, Vector3 position, Vector3 sigma) {
	visualization_msgs::Marker marker;
	marker.header.frame_id = drawing_frame;
	marker.header.stamp = ros::Time::now();
	marker.ns = "gaussian_namespace";
	marker.id = id;
	marker.type = visualization_msgs::Marker::SPHERE;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.position.x = position(0);
	marker.pose.position.y = position(1);
	marker.pose.position.z = position(2);
	marker.scale.x = sigma(0)*2;
	marker.scale.y = sigma(1)*2;
	marker.scale.z = sigma(2)*2;
	marker.color.a = 0.30; // Don't forget to set the alpha!
	marker.color.r = 0.9;
	marker.color.g = 0.1;
	marker.color.b = 0.9;
	gaussian_pub.publish( marker );
}

void MotionVisualizer::drawCollisionIndicator(int const& id, Vector3 const& position, double const& collision_prob) {
	visualization_msgs::Marker marker;
	marker.header.frame_id = drawing_frame;
	marker.header.stamp = ros::Time::now();
	marker.ns = "collision_namespace";
	marker.id = id;
	marker.type = visualization_msgs::Marker::SPHERE;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.position.x = position(0);
	marker.pose.position.y = position(1);
	marker.pose.position.z = position(2);
	marker.scale.x = 0.8;
	marker.scale.y = 0.8;
	marker.scale.z = 0.8;
	marker.color.a = 0.15; // Don't forget to set the alpha!
	marker.color.r = collision_prob;
	marker.color.g = 1.0 - collision_prob;
	marker.color.b = 0.0;
	collision_pub.publish( marker );
}

void MotionVisualizer::drawAll() {
	size_t num_motions = collision_probabilities.size();
	MotionLibrary* motion_library_ptr = motion_selector->GetMotionLibraryPtr();
	for (size_t motion_index = 0; motion_index < num_motions; motion_index++) {

		Eigen::Matrix<Scalar, Eigen::Dynamic, 3> sample_points_xyz_over_time =  motion_selector->sampleMotionForDrawing(motion_index, sampling_time_vector, num_samples);

		nav_msgs::Path action_samples_msg;
		action_samples_msg.header.frame_id = drawing_frame;
		action_samples_msg.header.stamp = ros::Time::now();
		Vector3 sigma;
		for (size_t sample = 0; sample < num_samples; sample++) {
			action_samples_msg.poses.push_back(PoseFromVector3(sample_points_xyz_over_time.row(sample), drawing_frame));
			sigma = motion_library_ptr->getSigmaAtTime(sampling_time_vector(sample));
			if (motion_index == *best_traj_index) {
				drawGaussianPropagation(sample, sample_points_xyz_over_time.row(sample), sigma);
			}
		}
		drawCollisionIndicator(motion_index, sample_points_xyz_over_time.row(num_samples-1), collision_probabilities.at(motion_index));
		action_paths_pubs.at(motion_index).publish(action_samples_msg);
	}
}