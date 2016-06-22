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
	marker.header.frame_id = drawing_frame;
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
	marker.color.a = 0.30; // Don't forget to set the alpha!
	marker.color.r = 0.9;
	marker.color.g = 0.1;
	marker.color.b = 0.9;
	gaussian_pub.publish( marker );
}

void TrajectoryVisualizer::NormalizeCollisions() {
	double max = collision_probabilities(0);
	double min = collision_probabilities(0);
	double current;
	for (int i = 1; i < 25; i++) {
		current = collision_probabilities(i);
		if (current > max) {
			max = current;
		}
		if (current < min) {
			min = current;
		}
	}

	if (max == min) {
		normalized_collision_probabilities = collision_probabilities;
		return;
	};

	for (int i = 0; i < 25; i++) {
		normalized_collision_probabilities(i) = (collision_probabilities(i) - min) / (max - min);
	}
}


void TrajectoryVisualizer::drawCollisionIndicator(int const& id, Vector3 const& position, double const& collision_prob) {
	NormalizeCollisions();

	visualization_msgs::Marker marker;
	marker.header.frame_id = drawing_frame;
	marker.header.stamp = ros::Time::now();
	marker.ns = "my_namespace";
	marker.id = 30 + id;
	marker.type = visualization_msgs::Marker::SPHERE;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.position.x = position(0);
	marker.pose.position.y = position(1);
	marker.pose.position.z = position(2);
	marker.scale.x = 0.2;
	marker.scale.y = 0.2;
	marker.scale.z = 0.2;
	marker.color.a = 0.15; // Don't forget to set the alpha!

	if (collision_prob > 0.5) {
		marker.color.r = collision_prob;
		marker.color.g = 1.0 - collision_prob;
	}
	else {
		marker.color.r = collision_prob;
		marker.color.g = 1.0 - collision_prob;
	}
	marker.color.b = 0.0;
	gaussian_pub.publish( marker );
}

void TrajectoryVisualizer::drawFinalStoppingPosition(int id, Vector3 position) {
	visualization_msgs::Marker marker;
	marker.header.frame_id = drawing_frame;
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

void TrajectoryVisualizer::drawDebugPoints() {
	LaserScanCollisionEvaluator* laser_scan_collision_ptr = trajectory_selector->GetLaserScanCollisionEvaluatorPtr();
	if (laser_scan_collision_ptr != nullptr) {
		Eigen::Matrix<Scalar, 100, 3> points = laser_scan_collision_ptr->DebugPointsToDraw();

		for (int i = 0; i<100; i++) {
			visualization_msgs::Marker marker;
			marker.header.frame_id = drawing_frame;
			marker.header.stamp = ros::Time::now();
			marker.ns = "my_namespace";
			marker.id = 65+i;
			marker.type = visualization_msgs::Marker::SPHERE;
			marker.action = visualization_msgs::Marker::ADD;
			marker.pose.position.x = points(i,0);
			marker.pose.position.y = points(i,1);
			marker.pose.position.z = points(i,2);
			marker.scale.x = 0.2;
			marker.scale.y = 0.2;
			marker.scale.z = 0.2;
			marker.color.a = 0.15; // Don't forget to set the alpha!
			marker.color.r = 0.0;
			marker.color.g = 0.0;
			marker.color.b = 1.0;
			gaussian_pub.publish( marker );

		}	
	}
}



void TrajectoryVisualizer::drawAll() {
	//drawDebugPoints();
	size_t num_trajectories = trajectory_selector->getNumTrajectories(); 
	TrajectoryLibrary* trajectory_library_ptr = trajectory_selector->GetTrajectoryLibraryPtr();


	for (size_t trajectory_index = 0; trajectory_index < num_trajectories; trajectory_index++) {

		Eigen::Matrix<Scalar, Eigen::Dynamic, 3> sample_points_xyz_over_time =  trajectory_selector->sampleTrajectoryForDrawing(trajectory_index, sampling_time_vector, num_samples);

		nav_msgs::Path action_samples_msg;
		action_samples_msg.header.frame_id = drawing_frame;
		action_samples_msg.header.stamp = ros::Time::now();
		Vector3 sigma;
		for (size_t sample = 0; sample < num_samples; sample++) {
			action_samples_msg.poses.push_back(PoseFromVector3(sample_points_xyz_over_time.row(sample), drawing_frame));
			sigma = trajectory_library_ptr->getRDFSigmaAtTime(sampling_time_vector(sample));
			if (trajectory_index == *best_traj_index) {
				drawGaussianPropagation(sample, sample_points_xyz_over_time.row(sample), sigma);
			}
		}

		// if (trajectory_index == *best_traj_index) {
		// 	drawFinalStoppingPosition(num_samples-1, sample_points_xyz_over_time.row(num_samples-1));
		// }
		drawCollisionIndicator(trajectory_index, sample_points_xyz_over_time.row(num_samples-1), normalized_collision_probabilities(trajectory_index));

		action_paths_pubs.at(trajectory_index).publish(action_samples_msg);
	}
}