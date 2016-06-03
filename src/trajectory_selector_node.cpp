#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/String.h>
#include "acl_fsw/QuadGoal.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "tf/tf.h"
#include <mutex>
#include <thread>
#include <std_srvs/Empty.h>
#include "trajectory_selector.h"
#include <cmath>


class TrajectorySelectorNode {
public:

	TrajectorySelectorNode(ros::NodeHandle & nh, std::string const& waypoint_topic, std::string const& pose_topic, std::string const& velocity_topic, std::string const& local_goal_topic, std::string const& samples_topic) {
		//nh.getParam("max_waypoints", max_waypoints);

		pose_sub = nh.subscribe(pose_topic, 1, &TrajectorySelectorNode::OnPose, this);
		velocity_sub = nh.subscribe(velocity_topic, 1, &TrajectorySelectorNode::OnVelocity, this);
		waypoints_sub = nh.subscribe(waypoint_topic, 1, &TrajectorySelectorNode::OnWaypoints, this);

		local_goal_pub = nh.advertise<acl_fsw::QuadGoal> (local_goal_topic, 1);
		poly_samples_pub = nh.advertise<nav_msgs::Path>(samples_topic, 1);
		vis_pub = nh.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );

		trajectory_selector.InitializeLibrary();
		createSamplingTimeVector();

		for (int i = 0; i < trajectory_selector.getNumTrajectories(); i++) {
			poly_samples_pubs.push_back(nh.advertise<nav_msgs::Path>(samples_topic+std::to_string(i), 1));
		}

		ROS_INFO("Finished constructing the trajectory selector node, waiting for waypoints");
	}

	void drawTrajectoryDebug() {
		size_t trajectory_index = 0;
		Eigen::Matrix<Scalar, Eigen::Dynamic, 3> sample_points_xyz_over_time =  trajectory_selector.sampleTrajectoryForDrawing(trajectory_index, sampling_time_vector, num_samples);

		nav_msgs::Path poly_samples_msg;
		poly_samples_msg.header.frame_id = "body";
		poly_samples_msg.header.stamp = ros::Time::now();
		mutex.lock();
		Vector3 sigma;
		for (size_t sample = 0; sample < num_samples; sample++) {
			poly_samples_msg.poses.push_back(PoseFromVector3(sample_points_xyz_over_time.row(sample)));
			sigma = trajectory_selector.getSigmaAtTime(sampling_time_vector(sample));
		 	drawGaussianPropagationDebug(sample, sample_points_xyz_over_time.row(sample), sigma);
		}
		mutex.unlock();
		poly_samples_pub.publish(poly_samples_msg);
	}

	void drawGaussianPropagationDebug(int id, Vector3 position, Vector3 sigma) {
		visualization_msgs::Marker marker;
		marker.header.frame_id = "body";
		marker.header.stamp = ros::Time();
		marker.ns = "my_namespace";
		marker.id = id;
		marker.type = visualization_msgs::Marker::SPHERE;
		marker.action = visualization_msgs::Marker::ADD;
		marker.pose.position.x = position(0);
		marker.pose.position.y = position(1);
		marker.pose.position.z = position(2);
		marker.scale.x = 0.1 + std::abs(position(0));
		marker.scale.y = 0.1 + std::abs(position(1));
		marker.scale.z = 0.1 + std::abs(position(2));
		marker.color.a = 0.15; // Don't forget to set the alpha!
		marker.color.r = 0.1;
		marker.color.g = 0.1;
		marker.color.b = 0.1;
		vis_pub.publish( marker );
	}

	void drawTrajectoriesDebug() {
		size_t num_trajectories = trajectory_selector.getNumTrajectories(); 
		
		for (size_t trajectory_index = 0; trajectory_index < num_trajectories; trajectory_index++) {

			Eigen::Matrix<Scalar, Eigen::Dynamic, 3> sample_points_xyz_over_time =  trajectory_selector.sampleTrajectoryForDrawing(trajectory_index, sampling_time_vector, num_samples);

			nav_msgs::Path poly_samples_msg;
			poly_samples_msg.header.frame_id = "body";
			poly_samples_msg.header.stamp = ros::Time::now();
			mutex.lock();
			Vector3 sigma;
			for (size_t sample = 0; sample < num_samples; sample++) {
				poly_samples_msg.poses.push_back(PoseFromVector3(sample_points_xyz_over_time.row(sample)));
				sigma = trajectory_selector.getSigmaAtTime(sampling_time_vector(sample));
				if (trajectory_index == 0) {
					drawGaussianPropagationDebug(sample, sample_points_xyz_over_time.row(sample), sigma);
				}
			}
			mutex.unlock();
			poly_samples_pubs.at(trajectory_index).publish(poly_samples_msg);
		}
	}

	// void drawTrajectoriesDebug() {
		
	// 	size_t num_trajectories = trajectory_selector.getNumTrajectories(); 

	// 	num_samples = 10;
	// 	size_t trajectory_index = 0;
	// 	double start_time = 0.0;
	// 	double final_time = 0.5;

	// 	nav_msgs::Path poly_samples_msg;
	// 	poly_samples_msg.header.frame_id = "world";
	// 	poly_samples_msg.header.stamp = ros::Time::now();

	// 	Eigen::Matrix<Scalar, Eigen::Dynamic, 3> sample_points_xyz_over_time;
	// 	for (size_t trajectory_index = 0; trajectory_index < num_trajectories; trajectory_index++) {
	// 		sample_points_xyz_over_time =  trajectory_selector.sampleTrajectoryForDrawing(trajectory_index, start_time, final_time, num_samples);

	// 		mutex.lock();
	// 		for (size_t sample = 0; sample < num_samples; sample++) {
	// 			poly_samples_msg.poses.push_back(PoseFromVector3(sample_points_xyz_over_time.row(sample)));
	// 		}
	// 		mutex.unlock();
	// 		poly_samples_pub.publish(poly_samples_msg);
	// 	}

	// }

private:

	void createSamplingTimeVector() {
		num_samples = 10;
		sampling_time_vector.resize(num_samples, 1);
		double start_time = 0.0;
		double final_time = 0.5;

		double sampling_time = 0;
		double sampling_interval = (final_time - start_time) / num_samples;
		for (size_t sample_index = 0; sample_index < num_samples; sample_index++) {
    		sampling_time = start_time + sampling_interval*(sample_index+1);
    		sampling_time_vector(sample_index) = sampling_time;
    	}
  	}

	geometry_msgs::PoseStamped PoseFromVector3(Vector3 const& position) {
		geometry_msgs::PoseStamped pose;
		pose.pose.position.x = position(0);
		pose.pose.position.y = position(1);
		pose.pose.position.z = position(2);
		pose.header.frame_id = "body";
		pose.header.stamp = ros::Time::now();
		return pose;
	}

	void OnPose( geometry_msgs::PoseStamped const& pose ) {
		//ROS_INFO("GOT POSE");
		mutex.lock();
		pose_x_y_z_yaw << pose.pose.position.x, pose.pose.position.y, pose.pose.position.z, tf::getYaw(pose.pose.orientation);
		mutex.unlock();
	}

	void OnVelocity( geometry_msgs::TwistStamped const& twist) {
		//ROS_INFO("GOT VELOCITY");
		
		// in world frame
		//twist.twist.linear.x, twist.twist.linear.y, twist.twist.linear.z

		// how to get into body frame?


		mutex.lock();
		trajectory_selector.setInitialVelocity(Vector3(twist.twist.linear.x, twist.twist.linear.y, twist.twist.linear.z));
		mutex.unlock();
	}

	Eigen::Vector3d VectorFromPose(geometry_msgs::PoseStamped const& pose) {
		return Vector3(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
	}

	void OnWaypoints(nav_msgs::Path const& waypoints) {

		//ROS_INFO("GOT WAYPOINTS");
		int waypoints_to_check = std::min((int) waypoints.poses.size(), max_waypoints);

		waypoints_matrix.resize(4, waypoints_to_check);
		waypoints_matrix.col(0) << VectorFromPose(waypoints.poses[0]), 0.0;  // yaw is currently hard set to be 0
		double distance_so_far = 0.0;
		double distance_to_add;
		double distance_left;
		Eigen::Vector3d truncated_waypoint;
		Eigen::Vector3d p1, p2;
		int i;
		for (i = 0; i < waypoints_to_check - 1; i++){
			p1 = VectorFromPose(waypoints.poses[i]);
			p2 = VectorFromPose(waypoints.poses[i+1]);
			distance_to_add = (p2-p1).norm();
			if ((distance_to_add + distance_so_far) < path_horizon_distance) {
				distance_so_far += distance_to_add;
				waypoints_matrix.col(i + 1) << p2, 0.0; // yaw is currently hard set to be 0
			}
			else {
				distance_left = path_horizon_distance - distance_so_far;
				truncated_waypoint = p1 + (p2-p1) / distance_to_add * distance_left;
				distance_so_far = distance_so_far + distance_left;
				waypoints_matrix.col(i + 1) << truncated_waypoint, 0.0; // yaw is currently hard set to be 0
				i++;
				break;

			}
		}

		waypoints_matrix.conservativeResize(4, i+1);

	}


	acl_fsw::QuadGoal local_goal_msg;

	ros::Subscriber waypoints_sub;
	ros::Subscriber pose_sub;
	ros::Subscriber velocity_sub;
	ros::Publisher local_goal_pub;
	ros::Publisher poly_samples_pub;
	ros::Publisher vis_pub;

	std::vector<ros::Publisher> poly_samples_pubs;

	nav_msgs::Path waypoints;
	nav_msgs::Path previous_waypoints;
	int max_waypoints = 10;
	double path_horizon_distance = 10.0;

	Eigen::Vector4d pose_x_y_z_yaw;
	Eigen::Matrix<double, 4, Eigen::Dynamic> waypoints_matrix;

	Eigen::Matrix<Scalar, Eigen::Dynamic, 1> sampling_time_vector;
	size_t num_samples;

	std::mutex mutex;

	TrajectorySelector trajectory_selector;

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};


int main(int argc, char* argv[]) {
	std::cout << "Initializing trajectory_selector_node" << std::endl;

	ros::init(argc, argv, "TrajectorySelectorNode");
	ros::NodeHandle nh;

	TrajectorySelectorNode trajectory_selector_node(nh, "/waypoint_list", "/FLA_ACL02/pose", "/FLA_ACL02/vel", "/goal_passthrough", "/poly_samples");

	std::cout << "Got through to here" << std::endl;


	while (ros::ok()) {
		trajectory_selector_node.drawTrajectoriesDebug();
		ros::spinOnce();
	}
}
