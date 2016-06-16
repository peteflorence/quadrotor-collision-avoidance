#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/PointCloud2.h>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include <mavros_msgs/AttitudeTarget.h>
#include <nav_msgs/OccupancyGrid.h>

#include "tf/tf.h"
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <mutex>
#include <cmath>
#include <time.h>
#include <stdlib.h>

#include "trajectory_selector.h"
#include "attitude_generator.h"
#include "trajectory_visualizer.h"
//#include "trajectory_selector_utils.h"

// debug only
#include <chrono>

  

class TrajectorySelectorNode {
public:

	TrajectorySelectorNode() {

		// Subscribers
		pose_sub = nh.subscribe("/samros/pose", 1, &TrajectorySelectorNode::OnPose, this);
		velocity_sub = nh.subscribe("/samros/twist", 1, &TrajectorySelectorNode::OnVelocity, this);
		//waypoints_sub = nh.subscribe("/waypoint_list", 1, &TrajectorySelectorNode::OnWaypoints, this);
  	    //point_cloud_sub = nh.subscribe("/flight/xtion_depth/points", 1, &TrajectorySelectorNode::OnPointCloud, this);
  	    global_goal_sub = nh.subscribe("/move_base_simple/goal", 1, &TrajectorySelectorNode::OnGlobalGoal, this);
  	    value_grid_sub = nh.subscribe("/value_grid", 1, &TrajectorySelectorNode::OnValueGrid, this);

  	    // Publishers
		carrot_pub = nh.advertise<visualization_msgs::Marker>( "carrot_marker", 0 );
		gaussian_pub = nh.advertise<visualization_msgs::Marker>( "gaussian_visualization", 0 );
		attitude_thrust_pub = nh.advertise<mavros_msgs::AttitudeTarget>("/mavros/setpoint_raw/attitude", 1);
		attitude_setpoint_visualization_pub = nh.advertise<geometry_msgs::PoseStamped>("attitude_setpoint", 1);

		// Initialization
		trajectory_selector.InitializeLibrary(final_time);

		trajectory_visualizer.initialize(&trajectory_selector, nh, &best_traj_index, final_time);
		tf_listener_ = std::make_shared<tf2_ros::TransformListener>(tf_buffer_);
		srand ( time(NULL) ); //initialize the random seed

		ROS_INFO("Finished constructing the trajectory selector node, waiting for waypoints");
	}

	void SetThrustForLibrary(double thrust) {
		TrajectoryLibrary* trajectory_library_ptr = trajectory_selector.GetTrajectoryLibraryPtr();
		if (trajectory_library_ptr != nullptr) {
			trajectory_library_ptr->setThrust(thrust);
		}
	}

	void ReactToSampledPointCloud() {
		Vector3 desired_acceleration;
		trajectory_selector.computeBestTrajectory(point_cloud_xyz_samples_ortho_body, carrot_ortho_body_frame, best_traj_index, desired_acceleration);

		Vector3 attitude_thrust_desired = attitude_generator.generateDesiredAttitudeThrust(desired_acceleration);

		SetThrustForLibrary(attitude_thrust_desired(2));

		PublishAttitudeSetpoint(attitude_thrust_desired);
	}

private:

	void UpdateTrajectoryLibraryRollPitch(double roll, double pitch) {
		TrajectoryLibrary* trajectory_library_ptr = trajectory_selector.GetTrajectoryLibraryPtr();
		if (trajectory_library_ptr != nullptr) {
			trajectory_library_ptr->setRollPitch(roll, pitch);
		}
	}

	void PublishOrthoBodyTransform(double roll, double pitch) {
		static tf2_ros::TransformBroadcaster br;
  		geometry_msgs::TransformStamped transformStamped;
  
	    transformStamped.header.stamp = ros::Time::now();
	    transformStamped.header.frame_id = "body";
	    transformStamped.child_frame_id = "ortho_body";
	    transformStamped.transform.translation.x = 0.0;
	    transformStamped.transform.translation.y = 0.0;
	    transformStamped.transform.translation.z = 0.0;
	    tf2::Quaternion q_ortho;
	    q_ortho.setRPY(-roll, -pitch, 0);
	    transformStamped.transform.rotation.x = q_ortho.x();
	    transformStamped.transform.rotation.y = q_ortho.y();
	    transformStamped.transform.rotation.z = q_ortho.z();
	    transformStamped.transform.rotation.w = q_ortho.w();

	    br.sendTransform(transformStamped);
	}

	void UpdateCarrotOrthoBodyFrame() {
		geometry_msgs::TransformStamped tf;
	    try {

	      tf = tf_buffer_.lookupTransform("ortho_body", "world", 
	                                    ros::Time(0), ros::Duration(1.0/30.0));
	    } catch (tf2::TransformException &ex) {
	      ROS_ERROR("%s", ex.what());
	      return;
	    }

	    geometry_msgs::PoseStamped pose_global_goal_world_frame = PoseFromVector3(carrot_world_frame, "world");
	    geometry_msgs::PoseStamped pose_global_goal_ortho_body_frame = PoseFromVector3(Vector3(0,0,0), "ortho_body");
	   
	    tf2::doTransform(pose_global_goal_world_frame, pose_global_goal_ortho_body_frame, tf);
	    carrot_ortho_body_frame = VectorFromPose(pose_global_goal_ortho_body_frame);
	}


	void OnPose( geometry_msgs::PoseStamped const& pose ) {
		//ROS_INFO("GOT POSE");
		attitude_generator.setZ(pose.pose.position.z);
		
		tf::Quaternion q(pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w);
		double roll, pitch, yaw;
		tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

		UpdateTrajectoryLibraryRollPitch(roll, pitch);
		PublishOrthoBodyTransform(roll, pitch);
		UpdateCarrotOrthoBodyFrame();

	}

	Vector3 TransformWorldToOrthoBody(Vector3 const& world_frame) {
		geometry_msgs::TransformStamped tf;
	    try {
	      tf = tf_buffer_.lookupTransform("ortho_body", "world", 
	                                    ros::Time(0), ros::Duration(1/30.0));
	    } catch (tf2::TransformException &ex) {
	      ROS_ERROR("%s", ex.what());
	      return Vector3::Zero();
	    }

	    Eigen::Quaternion<Scalar> quat(tf.transform.rotation.w, tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z);
	    Matrix3 R = quat.toRotationMatrix();
	    return R*world_frame;
	}

	void UpdateTrajectoryLibraryVelocity(Vector3 const& velocity_ortho_body_frame) {
		TrajectoryLibrary* trajectory_library_ptr = trajectory_selector.GetTrajectoryLibraryPtr();
		if (trajectory_library_ptr != nullptr) {
			trajectory_library_ptr->setInitialVelocity(velocity_ortho_body_frame);
		}
	}

	void OnVelocity( geometry_msgs::TwistStamped const& twist) {
		//ROS_INFO("GOT VELOCITY");
		attitude_generator.setZvelocity(twist.twist.linear.z);
		Vector3 velocity_world_frame(twist.twist.linear.x, twist.twist.linear.y, twist.twist.linear.z);
		Vector3 velocity_ortho_body_frame = TransformWorldToOrthoBody(velocity_world_frame);
		UpdateTrajectoryLibraryVelocity(velocity_ortho_body_frame);
	}
	
	void OnGlobalGoal(geometry_msgs::PoseStamped const& global_goal) {
		//ROS_INFO("GOT GLOBAL GOAL");
		carrot_world_frame << global_goal.pose.position.x, global_goal.pose.position.y, global_goal.pose.position.z+1.0; 
		UpdateCarrotOrthoBodyFrame();
	}

	void OnValueGrid(nav_msgs::OccupancyGrid value_grid_msg) {
		ROS_INFO("GOT VALUE GRID");
		auto t1 = std::chrono::high_resolution_clock::now();

		ValueGridEvaluator* value_grid_evaluator_ptr = trajectory_selector.GetValueGridEvaluatorPtr();
		ValueGrid* value_grid_ptr = value_grid_evaluator_ptr->GetValueGridPtr();

		value_grid_ptr->SetResolution(value_grid_msg.info.resolution);
		value_grid_ptr->SetSize(value_grid_msg.info.width, value_grid_msg.info.height);
		value_grid_ptr->SetOrigin(value_grid_msg.info.origin.position.x, value_grid_msg.info.origin.position.y);
		value_grid_ptr->SetValues(value_grid_msg.data);

		//trajectory_selector.PassInUpdatedValueGrid(&value_grid);
		auto t2 = std::chrono::high_resolution_clock::now();
		std::cout << "Whole value grid construction took "
      		<< std::chrono::duration_cast<std::chrono::microseconds>(t2-t1).count()
      		<< " microseconds\n";

		std::cout << value_grid_ptr->GetValueOfPosition(carrot_world_frame) << " is value of goal" << std::endl;
		std::cout << value_grid_ptr->GetValueOfPosition(Vector3(0,0,0)) << " is value of world origin" << std::endl;
		std::cout << value_grid_ptr->GetValueOfPosition(Vector3(0,-2.0,0)) << " is value of 1.5 to my right" << std::endl;
	}


	void OnWaypoints(nav_msgs::Path const& waypoints) {
		//ROS_INFO("GOT WAYPOINTS");
		int waypoints_to_check = std::min((int) waypoints.poses.size(), max_waypoints);
		nh.param("carrot_distance", carrot_distance, 0.5);

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
			if ((distance_to_add + distance_so_far) < carrot_distance) {
				distance_so_far += distance_to_add;
				waypoints_matrix.col(i + 1) << p2, 0.0; // yaw is currently hard set to be 0
			}
			else {
				distance_left = carrot_distance - distance_so_far;
				truncated_waypoint = p1 + (p2-p1) / distance_to_add * distance_left;
				distance_so_far = distance_so_far + distance_left;
				waypoints_matrix.col(i + 1) << truncated_waypoint, 0.0; // yaw is currently hard set to be 0
				i++;
				break;

			}
		}
		carrot_world_frame << waypoints_matrix(0, i), waypoints_matrix(1, i), waypoints_matrix(2, i); 
		//attitude_generator.setZsetpoint(carrot_world_frame(2));
		


		geometry_msgs::TransformStamped tf;
	    try {
	      // Need to remove leading "/" if it exists.
	      std::string pose_frame_id = waypoints.poses[0].header.frame_id;
	      if (pose_frame_id[0] == '/') {
	        pose_frame_id = pose_frame_id.substr(1, pose_frame_id.size()-1);
	      }

	      tf = tf_buffer_.lookupTransform("ortho_body", pose_frame_id, 
	                                    ros::Time(0), ros::Duration(1/30.0));
	    } catch (tf2::TransformException &ex) {
	      ROS_ERROR("%s", ex.what());
	      return;
	    }

	    geometry_msgs::PoseStamped pose_carrot_world_frame = PoseFromVector3(carrot_world_frame, "world");
	    geometry_msgs::PoseStamped pose_carrot_ortho_body_frame = PoseFromVector3(carrot_ortho_body_frame, "ortho_body");
	   
	    tf2::doTransform(pose_carrot_world_frame, pose_carrot_ortho_body_frame, tf);

	    carrot_ortho_body_frame = VectorFromPose(pose_carrot_ortho_body_frame);


	    visualization_msgs::Marker marker;
		marker.header.frame_id = "ortho_body";
		marker.header.stamp = ros::Time::now();
		marker.ns = "my_namespace";
		marker.id = 0;
		marker.type = visualization_msgs::Marker::SPHERE;
		marker.action = visualization_msgs::Marker::ADD;
		marker.pose.position.x = carrot_ortho_body_frame(0);
		marker.pose.position.y = carrot_ortho_body_frame(1);
		marker.pose.position.z = carrot_ortho_body_frame(2);
		marker.scale.x = 0.5;
		marker.scale.y = 0.5;
		marker.scale.z = 0.5;
		marker.color.a = 0.5; // Don't forget to set the alpha!
		marker.color.r = 0.9;
		marker.color.g = 0.4;
		marker.color.b = 0.0;
		carrot_pub.publish( marker );

	}



	void OnPointCloud(const sensor_msgs::PointCloud2ConstPtr& point_cloud_msg) {
		ROS_INFO("GOT POINT CLOUD");

		geometry_msgs::TransformStamped tf;
	    try {
	      tf = tf_buffer_.lookupTransform("ortho_body", "xtion_depth_optical_frame", 
	                                    ros::Time(0), ros::Duration(1/30.0));
	    } catch (tf2::TransformException &ex) {
	      ROS_ERROR("%s", ex.what());
	      return;
	    }

		pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2; 
		pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
		
    	pcl_conversions::toPCL(*point_cloud_msg, *cloud);
    	pcl::PointCloud<pcl::PointXYZ>::Ptr xyz_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    	pcl::fromPCLPointCloud2(*cloud,*xyz_cloud);

    	// edges are:
    	// 0,0
    	// 159, 0
    	// 0, 119
    	// 159,119
    	Vector3 blank;
    	blank << 0,0,0;

  		for (size_t i = 0; i < 100; i++) {
  			int x_rand_index = rand() % 160;
  			int y_rand_index = rand() % 120;
  			pcl::PointXYZ first_point = xyz_cloud->at(x_rand_index,y_rand_index);
  			geometry_msgs::PoseStamped point_cloud_xyz_body = PoseFromVector3(Vector3(first_point.x, first_point.y, first_point.z), "xtion_depth_optical_frame");
	    	geometry_msgs::PoseStamped point_cloud_xyz_ortho_body = PoseFromVector3(blank, "ortho_body");
	    	tf2::doTransform(point_cloud_xyz_body, point_cloud_xyz_ortho_body, tf);
  			point_cloud_xyz_samples_ortho_body.row(i) = VectorFromPose(point_cloud_xyz_ortho_body);
  		}

  	// 	for (int i = 0; i < 100; i++) {
	  // 		if (isnan(point_cloud_xyz_samples_ortho_body(i,0))) {
	  // 			continue;
	  // 		}
  	// 		visualization_msgs::Marker marker;
			// marker.header.frame_id = "ortho_body";
			// marker.header.stamp = ros::Time();
			// marker.ns = "my_namespace";
			// marker.id = 0;
			// marker.type = visualization_msgs::Marker::SPHERE;
			// marker.action = visualization_msgs::Marker::ADD;
			// marker.pose.position.x = point_cloud_xyz_samples_ortho_body(0,0);
			// marker.pose.position.y = point_cloud_xyz_samples_ortho_body(0,1);
			// marker.pose.position.z = point_cloud_xyz_samples_ortho_body(0,2);
			// //std::cout << "Trying to plot this point " << point_cloud_xyz_samples_ortho_body << std::endl;
			// marker.scale.x = 0.3;
			// marker.scale.y = 0.3;
			// marker.scale.z = 0.3;
			// marker.color.a = 0.5; // Don't forget to set the alpha!
			// marker.color.r = 0.9;
			// marker.color.g = 0.1;
			// marker.color.b = 0.9;
			// vis_pub.publish( marker );
			// break;
  	// 	}
  		

  		ReactToSampledPointCloud();
	
	}

	

	void PublishAttitudeSetpoint(Vector3 const& roll_pitch_thrust) { 

		using namespace Eigen;

		Vector3 pid;
		double offset;
		nh.param("z_p", pid(0), 1.5);
		nh.param("z_i", pid(1), 0.6);
		nh.param("z_d", pid(2), 0.5);
		nh.param("z_offset", offset, 0.69);
		attitude_generator.setGains(pid, offset);

		mavros_msgs::AttitudeTarget setpoint_msg;
		setpoint_msg.header.stamp = ros::Time::now();
		setpoint_msg.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ROLL_RATE 
			| mavros_msgs::AttitudeTarget::IGNORE_PITCH_RATE
			| mavros_msgs::AttitudeTarget::IGNORE_YAW_RATE
			;

		Matrix3f m;
		m =AngleAxisf(0.0, Vector3f::UnitZ())
		* AngleAxisf(roll_pitch_thrust(1), Vector3f::UnitY())
		* AngleAxisf(-roll_pitch_thrust(0), Vector3f::UnitX());

		Quaternionf q(m);

		setpoint_msg.orientation.w = q.w();
		setpoint_msg.orientation.x = q.x();
		setpoint_msg.orientation.y = q.y();
		setpoint_msg.orientation.z = q.z();

		setpoint_msg.thrust = roll_pitch_thrust(2);

		attitude_thrust_pub.publish(setpoint_msg);

		// To visualize setpoint

		// geometry_msgs::PoseStamped attitude_setpoint;
		// attitude_setpoint.header.frame_id = "world";
		// attitude_setpoint.header.stamp = ros::Time::now();
		// Vector3 initial_acceleration = trajectory_selector.getInitialAcceleration();
		// attitude_setpoint.pose.position.x = initial_acceleration(0);
		// attitude_setpoint.pose.position.y = initial_acceleration(1);
		// attitude_setpoint.pose.position.z = initial_acceleration(2)+5;
		// attitude_setpoint.pose.orientation = setpoint_msg.orientation;
		// attitude_setpoint_visualization_pub.publish( attitude_setpoint );

	}


	ros::Subscriber waypoints_sub;
	ros::Subscriber pose_sub;
	ros::Subscriber velocity_sub;
	ros::Subscriber point_cloud_sub;
	ros::Subscriber global_goal_sub;
	ros::Subscriber value_grid_sub;

	ros::Publisher carrot_pub;
	ros::Publisher gaussian_pub;
	ros::Publisher attitude_thrust_pub;
	ros::Publisher attitude_setpoint_visualization_pub;

	std::vector<ros::Publisher> action_paths_pubs;

	std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
	tf2_ros::Buffer tf_buffer_;

	nav_msgs::Path waypoints;
	nav_msgs::Path previous_waypoints;
	int max_waypoints = 6;
	double carrot_distance;

	double start_time = 0.0;
	double final_time = 0.5;

	Eigen::Vector4d pose_x_y_z_yaw;
	Eigen::Matrix<double, 4, Eigen::Dynamic> waypoints_matrix;

	Eigen::Matrix<Scalar, Eigen::Dynamic, 1> sampling_time_vector;
	size_t num_samples;

	Eigen::Matrix<Scalar, 100, 3> point_cloud_xyz_samples_ortho_body;

	std::mutex mutex;

	Vector3 carrot_world_frame;
	Vector3 carrot_ortho_body_frame;

	size_t best_traj_index = 0;

	TrajectorySelector trajectory_selector;
	AttitudeGenerator attitude_generator;


	ros::NodeHandle nh;

public:
	TrajectoryVisualizer trajectory_visualizer;
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};


int main(int argc, char* argv[]) {
	std::cout << "Initializing trajectory_selector_node" << std::endl;

	ros::init(argc, argv, "TrajectorySelectorNode");

	TrajectorySelectorNode trajectory_selector_node;

	std::cout << "Got through to here" << std::endl;
	ros::Rate spin_rate(100);

	auto t1 = std::chrono::high_resolution_clock::now();
	auto t2 = std::chrono::high_resolution_clock::now();

	while (ros::ok()) {
		//t1 = std::chrono::high_resolution_clock::now();
		trajectory_selector_node.ReactToSampledPointCloud();
		//t2 = std::chrono::high_resolution_clock::now();
		// std::cout << "ReactToSampledPointCloud took "
  //     		<< std::chrono::duration_cast<std::chrono::microseconds>(t2-t1).count()
  //     		<< " microseconds\n";
      	

		trajectory_selector_node.trajectory_visualizer.drawAll();

		ros::spinOnce();
		spin_rate.sleep();
	}
}
