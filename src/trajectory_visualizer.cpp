#include "trajectory_visualizer.h"


void TrajectoryVisualizer::TestVisualizer() {

  std::cout << "Printing from inside TrajectoryVisualizer " << std::endl;  

}


void TrajectoryVisualizer::initializeDrawingPaths() {
	for (int i = 0; i < trajectory_selector->getNumTrajectories(); i++) {
		action_paths_pubs.push_back(nh.advertise<nav_msgs::Path>("/poly_samples"+std::to_string(i), 1));
	}
}

void TrajectoryVisualizer::createSamplingTimeVector() {
	num_samples = 10;
	sampling_time_vector.resize(num_samples, 1);

	double sampling_time = 0;
	double sampling_interval = (final_time - start_time) / num_samples;
	for (size_t sample_index = 0; sample_index < num_samples; sample_index++) {
		sampling_time = start_time + sampling_interval*(sample_index+1);
		sampling_time_vector(sample_index) = sampling_time;
	}
}


void TrajectoryVisualizer::drawGaussianPropagation(int id, Vector3 position, Vector3 sigma) {
	visualization_msgs::Marker marker;
	marker.header.frame_id = "ortho_body";
	marker.header.stamp = ros::Time::now();
	marker.ns = "my_namespace";
	marker.id = id;
	marker.type = visualization_msgs::Marker::SPHERE;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.position.x = position(0);
	marker.pose.position.y = position(1);
	marker.pose.position.z = position(2);
	marker.scale.x = sigma(0);
	marker.scale.y = sigma(1);
	marker.scale.z = sigma(2);
	marker.color.a = 0.15; // Don't forget to set the alpha!
	marker.color.r = 0.9;
	marker.color.g = 0.1;
	marker.color.b = 0.9;
	gaussian_pub.publish( marker );
}

void TrajectoryVisualizer::drawFinalStoppingPosition(int id, Vector3 position) {
	visualization_msgs::Marker marker;
	marker.header.frame_id = "ortho_body";
	marker.header.stamp = ros::Time::now();
	marker.ns = "my_namespace";
	marker.id = id;
	marker.type = visualization_msgs::Marker::SPHERE;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.position.x = position(0);
	marker.pose.position.y = position(1);
	marker.pose.position.z = position(2);
	marker.scale.x = 1;
	marker.scale.y = 1;
	marker.scale.z = 1;
	marker.color.a = 0.15; // Don't forget to set the alpha!
	marker.color.r = 0.9;
	marker.color.g = 0.1;
	marker.color.b = 0.0;
	gaussian_pub.publish( marker );
}

void TrajectoryVisualizer::drawAll() {
	size_t num_trajectories = trajectory_selector->getNumTrajectories(); 

	for (size_t trajectory_index = 0; trajectory_index < num_trajectories; trajectory_index++) {

		Eigen::Matrix<Scalar, Eigen::Dynamic, 3> sample_points_xyz_over_time =  trajectory_selector->sampleTrajectoryForDrawing(trajectory_index, sampling_time_vector, num_samples);

		nav_msgs::Path action_samples_msg;
		action_samples_msg.header.frame_id = "ortho_body";
		action_samples_msg.header.stamp = ros::Time::now();
		Vector3 sigma;
		for (size_t sample = 0; sample < num_samples; sample++) {
			action_samples_msg.poses.push_back(PoseFromVector3(sample_points_xyz_over_time.row(sample), "ortho_body"));
			sigma = trajectory_selector->getSigmaAtTime(sampling_time_vector(sample));
			if (trajectory_index == *best_traj_index) {
				drawGaussianPropagation(sample, sample_points_xyz_over_time.row(sample), sigma);
			}
		}

		if (trajectory_index == *best_traj_index) {
			drawFinalStoppingPosition(num_samples-1, sample_points_xyz_over_time.row(num_samples-1));
		}

		action_paths_pubs.at(trajectory_index).publish(action_samples_msg);
	}
}