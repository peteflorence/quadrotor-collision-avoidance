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
#include <tf2_ros/transform_listener.h>
#include <sensor_msgs/PointCloud2.h>

  
std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
tf2_ros::Buffer tf_buffer_;


class TrajectorySelectorNode {
public:

	TrajectorySelectorNode(ros::NodeHandle & nh, std::string const& waypoint_topic, std::string const& pose_topic, std::string const& velocity_topic, std::string const& local_goal_topic, std::string const& samples_topic) {
		//nh.getParam("max_waypoints", max_waypoints);

		pose_sub = nh.subscribe(pose_topic, 1, &TrajectorySelectorNode::OnPose, this);
		velocity_sub = nh.subscribe(velocity_topic, 1, &TrajectorySelectorNode::OnVelocity, this);
		waypoints_sub = nh.subscribe(waypoint_topic, 1, &TrajectorySelectorNode::OnWaypoints, this);
  	    point_cloud_sub = nh.subscribe("/flight/xtion_depth/points", 1, &TrajectorySelectorNode::OnPointCloud, this);

		local_goal_pub = nh.advertise<acl_fsw::QuadGoal> (local_goal_topic, 1);
		poly_samples_pub = nh.advertise<nav_msgs::Path>(samples_topic, 1);
		vis_pub = nh.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );

		trajectory_selector.InitializeLibrary();
		createSamplingTimeVector();

		for (int i = 0; i < trajectory_selector.getNumTrajectories(); i++) {
			poly_samples_pubs.push_back(nh.advertise<nav_msgs::Path>(samples_topic+std::to_string(i), 1));
		}

		tf_listener_ = std::make_shared<tf2_ros::TransformListener>(tf_buffer_);

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
		marker.color.r = 0.9;
		marker.color.g = 0.1;
		marker.color.b = 0.9;
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
		
		geometry_msgs::TransformStamped tf;
	    try {
	      // Need to remove leading "/" if it exists.
	      std::string twist_frame_id = twist.header.frame_id;
	      if (twist_frame_id[0] == '/') {
	        twist_frame_id = twist_frame_id.substr(1, twist_frame_id.size()-1);
	      }

	      tf = tf_buffer_.lookupTransform("body", twist_frame_id, 
	                                    ros::Time(twist.header.stamp),
	                                    ros::Duration(1.0/30));
	    } catch (tf2::TransformException &ex) {
	      ROS_ERROR("%s", ex.what());
	      return;
	    }

	    Eigen::Quaternion<Scalar> quat(tf.transform.rotation.w, tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z);
	    Matrix3 R = quat.toRotationMatrix();

		mutex.lock();
		trajectory_selector.setInitialVelocity(R*Vector3(twist.twist.linear.x, twist.twist.linear.y, twist.twist.linear.z));
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



	void OnPointCloud(const sensor_msgs::PointCloud2ConstPtr& point_cloud) {
		ROS_INFO("GOT POINT CLOUD");
		//*point_cloud to access the point cloud (deref the ptr)
	}


	acl_fsw::QuadGoal local_goal_msg;

	ros::Subscriber waypoints_sub;
	ros::Subscriber pose_sub;
	ros::Subscriber velocity_sub;
	ros::Subscriber point_cloud_sub;

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
