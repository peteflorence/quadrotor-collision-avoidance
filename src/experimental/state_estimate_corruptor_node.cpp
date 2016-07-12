#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include <mavros_msgs/AttitudeTarget.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/PointCloud2.h>

#include "tf/tf.h"
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <mutex>
#include <cmath>
#include <time.h>
#include <stdlib.h>
#include <chrono>


class StateEstimateCorruptorNode {
public:

	StateEstimateCorruptorNode() {

		// Subscribers
		pose_sub = nh.subscribe("/true_pose", 1, &StateEstimateCorruptorNode::OnPose, this);
		velocity_sub = nh.subscribe("/true_vel", 1, &StateEstimateCorruptorNode::OnVelocity, this);

  	    // Publishers
		corrupted_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/samros/pose", 1);
		corrupted_velocity_pub = nh.advertise<geometry_msgs::TwistStamped>("/samros/twist", 1);

		tf_listener_ = std::make_shared<tf2_ros::TransformListener>(tf_buffer_);
		srand ( time(NULL) ); //initialize the random seed

		ROS_INFO("Finished constructing the state estimate corruptor node");
	}

	
private:



	void OnPose( geometry_msgs::PoseStamped const& pose ) {
		ROS_INFO("GOT POSE");


		actual_pose_global_x = pose.pose.position.x;
		actual_pose_global_y = pose.pose.position.y;
		actual_pose_global_z = pose.pose.position.z;

		if (!virgin) {

			double diff_actual_pose_global_x = - previous_actual_pose_global_x + actual_pose_global_x;
			double diff_actual_pose_global_y = - previous_actual_pose_global_y + actual_pose_global_y;
			double diff_actual_pose_global_z = - previous_actual_pose_global_z + actual_pose_global_z;

			corrupted_pose_global_x = corrupted_pose_global_x + diff_actual_pose_global_x + randomNoise()*actual_velocity_global_x; // plus noise
			corrupted_pose_global_y = corrupted_pose_global_y + diff_actual_pose_global_y + randomNoise()*actual_velocity_global_y;
			corrupted_pose_global_z = actual_pose_global_z;
		}
		else {
			corrupted_pose_global_x = actual_pose_global_x;
			corrupted_pose_global_y = actual_pose_global_y;
			corrupted_pose_global_z = actual_pose_global_z;
		}

		previous_actual_pose_global_x = actual_pose_global_x;
		previous_actual_pose_global_y = actual_pose_global_y;
		previous_actual_pose_global_z = actual_pose_global_z;

		PublishCorruptedPose(pose);
		virgin = false;
	}

	double randomNoise() {
		std::random_device rd;
		std::mt19937 gen(rd());

		std::normal_distribution<> d(0.0,noise_scaling);
		return d(gen);
	}


	void OnVelocity( geometry_msgs::TwistStamped const& twist) {
		ROS_INFO("GOT VELOCITY");
		actual_velocity_global_x = twist.twist.linear.x;
		actual_velocity_global_y = twist.twist.linear.y;
		actual_velocity_global_z = twist.twist.linear.z;

		PublishCorruptedTwist(twist);
	}
	
	void PublishCorruptedPose(geometry_msgs::PoseStamped const& pose) { 

		geometry_msgs::PoseStamped corrupted_pose;
		corrupted_pose.header = pose.header;
		corrupted_pose.pose.position.x = corrupted_pose_global_x;
		corrupted_pose.pose.position.y = corrupted_pose_global_y;
		corrupted_pose.pose.position.z = corrupted_pose_global_z;
		corrupted_pose.pose.orientation = pose.pose.orientation;
		corrupted_pose_pub.publish( corrupted_pose );
		PublishCorruptedBodyTransform( corrupted_pose );

	}

	void PublishCorruptedTwist(geometry_msgs::TwistStamped const& twist) { 

		geometry_msgs::TwistStamped corrupted_twist;
		corrupted_twist = twist;
		corrupted_twist.twist.linear.x = twist.twist.linear.x * (1 + randomNoise());
		corrupted_twist.twist.linear.y = twist.twist.linear.y * (1 + randomNoise());
		corrupted_velocity_pub.publish( corrupted_twist );

	}

	void PublishCorruptedBodyTransform(geometry_msgs::PoseStamped const& pose) {
		static tf2_ros::TransformBroadcaster br;
  		geometry_msgs::TransformStamped transformStamped;
  
	    transformStamped.header.stamp = pose.header.stamp;
	    transformStamped.header.frame_id = "world";
	    transformStamped.child_frame_id = "body";
	    transformStamped.transform.translation.x = pose.pose.position.x;
	    transformStamped.transform.translation.y = pose.pose.position.y;
	    transformStamped.transform.translation.z = pose.pose.position.z;
	    transformStamped.transform.rotation.x = pose.pose.orientation.x;
	    transformStamped.transform.rotation.y = pose.pose.orientation.y;
	    transformStamped.transform.rotation.z = pose.pose.orientation.z;
	    transformStamped.transform.rotation.w = pose.pose.orientation.w;

	    br.sendTransform(transformStamped);
	}

	bool virgin = true;

	ros::Subscriber pose_sub;
	ros::Subscriber velocity_sub;

	ros::Publisher corrupted_pose_pub;
	ros::Publisher corrupted_velocity_pub;

	std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
	tf2_ros::Buffer tf_buffer_;

	double actual_pose_global_x = 0;
	double actual_pose_global_y = 0;
	double actual_pose_global_z = 0;

	double previous_actual_pose_global_x = 0;
	double previous_actual_pose_global_y = 0;
	double previous_actual_pose_global_z = 0;

	double corrupted_pose_global_x = 0;
	double corrupted_pose_global_y = 0;
	double corrupted_pose_global_z = 0;

	double actual_velocity_global_x = 0;
	double actual_velocity_global_y = 0;
	double actual_velocity_global_z = 0;

	double noise_scaling = 0.05;

	ros::NodeHandle nh;
};


int main(int argc, char* argv[]) {
	std::cout << "Initializing state_estimate_corruptor node" << std::endl;

	ros::init(argc, argv, "StateEstimateCorruptorNode");

	StateEstimateCorruptorNode state_estimate_corruptor_node;

	std::cout << "Got through to here" << std::endl;



	while (ros::ok()) {
		ros::spinOnce();
	}
}
