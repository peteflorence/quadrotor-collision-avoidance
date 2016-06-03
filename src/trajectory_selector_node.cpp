#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <std_msgs/String.h>
#include "acl_fsw/QuadGoal.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "tf/tf.h"
#include <mutex>
#include <thread>
#include <std_srvs/Empty.h>
#include "trajectory_selector.h"


class TrajectorySelectorNode {
public:

	TrajectorySelectorNode(ros::NodeHandle & nh, std::string const& waypoint_topic, std::string const& pose_topic, std::string const& velocity_topic, std::string const& local_goal_topic, std::string const& samples_topic) {
		//nh.getParam("max_waypoints", max_waypoints);

		pose_sub = nh.subscribe(pose_topic, 1, &TrajectorySelectorNode::OnPose, this);
		velocity_sub = nh.subscribe(velocity_topic, 1, &TrajectorySelectorNode::OnVelocity, this);
		waypoints_sub = nh.subscribe(waypoint_topic, 1, &TrajectorySelectorNode::OnWaypoints, this);

		local_goal_pub = nh.advertise<acl_fsw::QuadGoal> (local_goal_topic, 1);
		poly_samples_pub = nh.advertise<nav_msgs::Path>(samples_topic, 1);

		trajectory_selector.InitializeLibrary();

		ROS_INFO("Finished constructing the trajectory selector node, waiting for waypoints");
	}


	void drawTrajectoryDebug() {
		size_t num_samples = 100;
		size_t trajectory_index = 0;
		double start_time = 0.0;
		double final_time = 0.5;

		Eigen::Matrix<Scalar, Eigen::Dynamic, 3> sample_points_xyz_over_time =  trajectory_selector.sampleTrajectoryForDrawing(trajectory_index, start_time, final_time, num_samples);
		//std::cout << "sample_points " << sample_points_xyz_over_time << std::endl;

		nav_msgs::Path poly_samples_msg;
		poly_samples_msg.header.frame_id = "world";
		poly_samples_msg.header.stamp = ros::Time::now();
		mutex.lock();
		for (size_t sample = 0; sample < num_samples; sample++) {
			poly_samples_msg.poses.push_back(PoseFromVector3(sample_points_xyz_over_time.row(sample)));
		}
		mutex.unlock();
		poly_samples_pub.publish(poly_samples_msg);
	}

private:

	geometry_msgs::PoseStamped PoseFromVector3(Vector3 const& position) {
		geometry_msgs::PoseStamped pose;
		pose.pose.position.x = position(0);
		pose.pose.position.y = position(1);
		pose.pose.position.z = position(2);
		pose.header.frame_id = "world";
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

	nav_msgs::Path waypoints;
	nav_msgs::Path previous_waypoints;
	int max_waypoints = 10;
	double path_horizon_distance = 10.0;

	Eigen::Vector4d pose_x_y_z_yaw;
	Eigen::Matrix<double, 4, Eigen::Dynamic> waypoints_matrix;

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
		trajectory_selector_node.drawTrajectoryDebug();
		ros::spinOnce();
	}
}
