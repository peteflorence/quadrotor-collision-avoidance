#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/PointCloud2.h>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include <mavros_msgs/AttitudeTarget.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/PointCloud2.h>

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
#include <chrono>

#include "motion_selector.h"
#include "attitude_generator.h"
#include "motion_visualizer.h"


class MotionSelectorNode {
public:

	MotionSelectorNode() {

		// Subscribers

		pose_sub = nh.subscribe("/pose", 1, &MotionSelectorNode::OnPose, this);
		velocity_sub = nh.subscribe("/twist", 1, &MotionSelectorNode::OnVelocity, this);
  	    depth_image_sub = nh.subscribe("/flight/xtion_depth/points", 1, &MotionSelectorNode::OnDepthImage, this);
  	    //global_goal_sub = nh.subscribe("/move_base_simple/goal", 1, &MotionSelectorNode::OnGlobalGoal, this);
  	    local_goal_sub = nh.subscribe("/local_goal", 1, &MotionSelectorNode::OnLocalGoal, this);
  	    //value_grid_sub = nh.subscribe("/value_grid", 1, &MotionSelectorNode::OnValueGrid, this);
  	    // laser_scan_sub = nh.subscribe("/laserscan_to_pointcloud/cloud2_out", 1, &MotionSelectorNode::OnScan, this);


  	    // Publishers
  	    carrot_pub = nh.advertise<visualization_msgs::Marker>( "carrot_marker", 0 );
		gaussian_pub = nh.advertise<visualization_msgs::Marker>( "gaussian_visualization", 0 );
		attitude_thrust_pub = nh.advertise<mavros_msgs::AttitudeTarget>("/mux_input_1", 1);
		//attitude_setpoint_visualization_pub = nh.advertise<geometry_msgs::PoseStamped>("attitude_setpoint", 1);

		// Initialization
		double a_max_horizontal;
		double soft_top_speed;
        double min_speed_at_max_acceleration_total;
        double max_acceleration_total;

		nh.param("soft_top_speed", soft_top_speed, 2.0);
		nh.param("acceleration_interpolation_min", a_max_horizontal, 3.5);
		nh.param("yaw_on", yaw_on, false);
		nh.param("use_depth_image", use_depth_image, true);
        nh.param("speed_at_acceleration_max", min_speed_at_max_acceleration_total, 10.0);
        nh.param("acceleration_interpolation_max", max_acceleration_total, 4.0);
        nh.param("flight_altitude", flight_altitude, 1.2);

		this->soft_top_speed_max = soft_top_speed;

		motion_selector.InitializeLibrary(final_time, soft_top_speed, a_max_horizontal, min_speed_at_max_acceleration_total, max_acceleration_total);
		attitude_generator.setZsetpoint(flight_altitude);

		motion_visualizer.initialize(&motion_selector, nh, &best_traj_index, final_time);
		tf_listener_ = std::make_shared<tf2_ros::TransformListener>(tf_buffer_);
		srand ( time(NULL) ); //initialize the random seed

		ROS_INFO("Finished constructing the motion selector node");
	}

	void SetThrustForLibrary(double thrust) {
		MotionLibrary* motion_library_ptr = motion_selector.GetMotionLibraryPtr();
		if (motion_library_ptr != nullptr) {
			motion_library_ptr->setThrust(thrust);
		}
	}

	geometry_msgs::TransformStamped GetTransformToWorld() {
		geometry_msgs::TransformStamped tf;
	    try {

	      tf = tf_buffer_.lookupTransform("world", "ortho_body", 
	                                    ros::Time(0), ros::Duration(1.0/30.0));
	    } catch (tf2::TransformException &ex) {
	      ROS_ERROR("%s", ex.what());
	      return tf;
	    }
	    return tf;
	}

	void ReactToSampledPointCloud() {

		
		auto t1 = std::chrono::high_resolution_clock::now();
		
		mutex.lock();
		if (pose_global_z > 0.35) {
			motion_selector.computeBestEuclideanMotion(carrot_ortho_body_frame, best_traj_index, desired_acceleration);

			// geometry_msgs::TransformStamped tf = GetTransformToWorld();
			// motion_selector.computeBestDijkstraMotion(carrot_ortho_body_frame, carrot_world_frame, tf, best_traj_index, desired_acceleration);
	     }
	     else {
	     	motion_selector.computeTakeoffMotion(carrot_ortho_body_frame, best_traj_index, desired_acceleration);
	     }
	    mutex.unlock();


      	mutex.lock();
	    if (yaw_on) {
	    	SetYawFromMotion();
	    } 
	    mutex.unlock();
		
      	Eigen::Matrix<Scalar, 26, 1> collision_probabilities = motion_selector.getCollisionProbabilities();
		motion_visualizer.setCollisionProbabilities(collision_probabilities);

		PublishCurrentAttitudeSetpoint();
	}

	void ComputeBestAccelerationMotion() {
		mutex.lock();
		MotionLibrary* motion_library_ptr = motion_selector.GetMotionLibraryPtr();
		if (motion_library_ptr != nullptr) {

			Vector3 initial_velocity_ortho_body = motion_library_ptr->getMotionFromIndex(best_traj_index).getVelocity(0.0);
			Vector3 normalized_vector_towards_carrot_ortho_body = carrot_ortho_body_frame / (carrot_ortho_body_frame.norm());
			double initial_velocity_ortho_body_towards_carrot = initial_velocity_ortho_body.dot(normalized_vector_towards_carrot_ortho_body);

			double time_to_eval = 0.5;
			double best_acceleration_norm = (soft_top_speed_max - initial_velocity_ortho_body_towards_carrot) / time_to_eval;
			double current_max_acceleration = motion_library_ptr->getNewMaxAcceleration();

			if (best_acceleration_norm > current_max_acceleration) {
				best_acceleration_norm = current_max_acceleration;
			}
			if (best_acceleration_norm < -current_max_acceleration) {
				best_acceleration_norm = -current_max_acceleration;
			}
			Vector3 best_acceleration = normalized_vector_towards_carrot_ortho_body*best_acceleration_norm;

			motion_library_ptr->setBestAccelerationMotion(best_acceleration);
			Vector3 stop_position = motion_library_ptr->getMotionFromIndex(26-1).getTerminalStopPosition(0.5);
			double stop_distance = stop_position.dot(normalized_vector_towards_carrot_ortho_body);

			double distance_to_carrot = carrot_ortho_body_frame.norm();
			if (stop_distance > distance_to_carrot) {
				best_acceleration_norm = -0.5*(initial_velocity_ortho_body_towards_carrot*initial_velocity_ortho_body_towards_carrot)/distance_to_carrot;
				best_acceleration = normalized_vector_towards_carrot_ortho_body*best_acceleration_norm;
				motion_library_ptr->setBestAccelerationMotion(best_acceleration);
			} 

		}
		mutex.unlock();
	}

	void PublishCurrentAttitudeSetpoint() {
		attitude_thrust_desired = attitude_generator.generateDesiredAttitudeThrust(desired_acceleration);
		SetThrustForLibrary(attitude_thrust_desired(2));
		PublishAttitudeSetpoint(attitude_thrust_desired);
	}

	bool UseDepthImage() {
		return use_depth_image;
	}

private:


	void SetYawFromMotion() {
		MotionLibrary* motion_library_ptr = motion_selector.GetMotionLibraryPtr();
		if (motion_library_ptr != nullptr) {

			// get position at t=0
			Vector3 initial_position_ortho_body = motion_library_ptr->getMotionFromIndex(best_traj_index).getPosition(0.0);
			// get velocity at t=0
			Vector3 initial_velocity_ortho_body = motion_library_ptr->getMotionFromIndex(best_traj_index).getVelocity(0.5);
			Vector3 final_velocity_ortho_body = motion_library_ptr->getMotionFromIndex(best_traj_index).getVelocity(0.5);
			// normalize velocity
			double speed_initial = initial_velocity_ortho_body.norm();
			double speed_final = final_velocity_ortho_body.norm();
			if (speed_final != 0) {
				final_velocity_ortho_body = final_velocity_ortho_body / speed_final;
			}

			// add normalized velocity to position to get a future position
			Vector3 final_position_ortho_body = initial_position_ortho_body + initial_velocity_ortho_body;
			// yaw towards future position using below

			Vector3 final_position_world = TransformOrthoBodyToWorld(final_position_ortho_body);
			
			if (speed_initial < 2.0 && carrot_ortho_body_frame.norm() < 2.0) {
				motion_selector.SetSoftTopSpeed(soft_top_speed_max);
				return;
			}
			if ((final_position_world(0) - pose_global_x)!= 0) {
				double potential_bearing_azimuth_degrees = CalculateYawFromPosition(final_position_world);
				double actual_bearing_azimuth_degrees = -pose_global_yaw * 180.0/M_PI;
				double bearing_error = potential_bearing_azimuth_degrees - actual_bearing_azimuth_degrees;
				while(bearing_error > 180) { 
					bearing_error -= 360;
				}
				while(bearing_error < -180) { 
					bearing_error += 360;
				}
				//ROS_WARN("set_bearing_azimuth_degrees %f", set_bearing_azimuth_degrees);
				//ROS_WARN("bearing_azimuth_degrees %f", bearing_azimuth_degrees);
				//ROS_WARN("potential_bearing_azimuth_degrees %f", potential_bearing_azimuth_degrees);
				//ROS_WARN("bearing_error %f", bearing_error);
				//ROS_WARN("pose_global_yaw %f", pose_global_yaw);
				//ROS_WARN("actual_bearing_azimuth_degrees %f", actual_bearing_azimuth_degrees);

				if (abs(bearing_error) < 60.0)  {
					motion_selector.SetSoftTopSpeed(soft_top_speed_max);
					bearing_azimuth_degrees = potential_bearing_azimuth_degrees;
					return;
				}
				motion_selector.SetSoftTopSpeed(0.1);
				if (speed_initial < 0.5) {
					bearing_azimuth_degrees = CalculateYawFromPosition(carrot_world_frame);
				}
			}
		}
	}

	double CalculateYawFromPosition(Vector3 final_position) {
		return 180.0/M_PI*atan2(-(final_position(1) - pose_global_y), final_position(0) - pose_global_x);	
	}
	

	void SetGoalFromBearing() {
		bool go;
		nh.param("go", go, false);
		if (go) {
			carrot_ortho_body_frame << 100, 0, 0;
		}
		else if (carrot_ortho_body_frame(0) == 100) {
			carrot_ortho_body_frame << 5, 0, 0;
		}
	}

	void UpdateMotionLibraryRollPitch(double roll, double pitch) {
		MotionLibrary* motion_library_ptr = motion_selector.GetMotionLibraryPtr();
		if (motion_library_ptr != nullptr) {
			motion_library_ptr->setRollPitch(roll, pitch);
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
	    mutex.lock();
	    carrot_ortho_body_frame = VectorFromPose(pose_global_goal_ortho_body_frame);
	    mutex.unlock();
	}

	void UpdateAttitudeGeneratorRollPitch(double roll, double pitch) {
		attitude_generator.UpdateRollPitch(roll, pitch);
	}

	void UpdateLaserRDFFramesFromPose() {
		transformAccelerationsIntoLaserRDFFrames();
		MotionLibrary* motion_library_ptr = motion_selector.GetMotionLibraryPtr();
		Vector3 initial_acceleration = motion_library_ptr->getInitialAcceleration();
    	if (motion_library_ptr != nullptr) {
			motion_library_ptr->setInitialAccelerationLASER(transformOrthoBodyIntoLaserFrame(initial_acceleration));
			motion_library_ptr->setInitialAccelerationRDF(transformOrthoBodyIntoRDFFrame(initial_acceleration));
		}
	}


	void OnPose( geometry_msgs::PoseStamped const& pose ) {
		//ROS_INFO("GOT POSE");
		attitude_generator.setZ(pose.pose.position.z);
		
		tf::Quaternion q(pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w);
		double roll, pitch, yaw;
		tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

		UpdateMotionLibraryRollPitch(roll, pitch);
		UpdateAttitudeGeneratorRollPitch(roll, pitch);
		PublishOrthoBodyTransform(roll, pitch);
		UpdateCarrotOrthoBodyFrame();
		ComputeBestAccelerationMotion();
		UpdateLaserRDFFramesFromPose();

		mutex.lock();
		SetPose(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z, yaw);
		mutex.unlock();
	}

	void SetPose(double x, double y, double z, double yaw) {
		pose_global_x = x;
		pose_global_y = y;
		pose_global_z = z;
		pose_global_yaw = yaw;
	}

	void transformAccelerationsIntoLaserRDFFrames() {
		MotionLibrary* motion_library_ptr = motion_selector.GetMotionLibraryPtr();
		if (motion_library_ptr != nullptr) {

			std::vector<Motion>::iterator motion_iterator_begin = motion_library_ptr->GetMotionNonConstIteratorBegin();
	  		std::vector<Motion>::iterator motion_iterator_end = motion_library_ptr->GetMotionNonConstIteratorEnd();

	  		Vector3 acceleration_ortho_body;
			Vector3 acceleration_laser_frame;
			Vector3 acceleration_rdf_frame;


	  		for (auto motion = motion_iterator_begin; motion != motion_iterator_end; motion++) {
	  			acceleration_ortho_body = motion->getAcceleration();

	  			acceleration_laser_frame = transformOrthoBodyIntoLaserFrame(acceleration_ortho_body);
	  			motion->setAccelerationLASER(acceleration_laser_frame);

	  			acceleration_rdf_frame = transformOrthoBodyIntoRDFFrame(acceleration_ortho_body);
	  			motion->setAccelerationRDF(acceleration_rdf_frame);
	  		} 
	  	}
	}

	Vector3 transformOrthoBodyIntoLaserFrame(Vector3 const& ortho_body_vector) {
		geometry_msgs::TransformStamped tf;
    	try {
     		tf = tf_buffer_.lookupTransform("laser", "ortho_body", 
                                    ros::Time(0), ros::Duration(1/30.0));
   		} catch (tf2::TransformException &ex) {
     	 	ROS_ERROR("%s", ex.what());
      	return Vector3(0,0,0);
    	}
    	geometry_msgs::PoseStamped pose_ortho_body_vector = PoseFromVector3(ortho_body_vector, "ortho_body");
    	geometry_msgs::PoseStamped pose_vector_laser_frame = PoseFromVector3(Vector3(0,0,0), "laser");
    	tf2::doTransform(pose_ortho_body_vector, pose_vector_laser_frame, tf);
    	return VectorFromPose(pose_vector_laser_frame);
	}

	Vector3 transformOrthoBodyIntoRDFFrame(Vector3 const& ortho_body_vector) {
		geometry_msgs::TransformStamped tf;
    	try {
     		tf = tf_buffer_.lookupTransform("xtion_depth_optical_frame", "ortho_body", 
                                    ros::Time(0), ros::Duration(1/30.0));
   		} catch (tf2::TransformException &ex) {
     	 	ROS_ERROR("%s", ex.what());
      	return Vector3(0,0,0);
    	}
    	geometry_msgs::PoseStamped pose_ortho_body_vector = PoseFromVector3(ortho_body_vector, "ortho_body");
    	geometry_msgs::PoseStamped pose_vector_rdf_frame = PoseFromVector3(Vector3(0,0,0), "xtion_depth_optical_frame");
    	tf2::doTransform(pose_ortho_body_vector, pose_vector_rdf_frame, tf);
    	return VectorFromPose(pose_vector_rdf_frame);
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

	void UpdateMotionLibraryVelocity(Vector3 const& velocity_ortho_body_frame) {
		MotionLibrary* motion_library_ptr = motion_selector.GetMotionLibraryPtr();
		if (motion_library_ptr != nullptr) {
			motion_library_ptr->setInitialVelocity(velocity_ortho_body_frame);
			motion_library_ptr->setInitialVelocityLASER(transformOrthoBodyIntoLaserFrame(velocity_ortho_body_frame));
			motion_library_ptr->setInitialVelocityRDF(transformOrthoBodyIntoRDFFrame(velocity_ortho_body_frame));
		}
	}

	void OnVelocity( geometry_msgs::TwistStamped const& twist) {
		//ROS_INFO("GOT VELOCITY");
		attitude_generator.setZvelocity(twist.twist.linear.z);
		Vector3 velocity_world_frame(twist.twist.linear.x, twist.twist.linear.y, twist.twist.linear.z);
		Vector3 velocity_ortho_body_frame = TransformWorldToOrthoBody(velocity_world_frame);
		velocity_ortho_body_frame(2) = 0.0;  // WARNING for 2D only
		UpdateMotionLibraryVelocity(velocity_ortho_body_frame);
		double speed = velocity_ortho_body_frame.norm();
		UpdateTimeHorizon(speed);
		UpdateMaxAcceleration(speed);
	}

	void UpdateMaxAcceleration(double speed) {
		MotionLibrary* motion_library_ptr = motion_selector.GetMotionLibraryPtr();
			if (motion_library_ptr != nullptr) {
				motion_library_ptr->UpdateMaxAcceleration(speed);
			}
	}

	void UpdateTimeHorizon(double speed) { 
		if (speed < 10.0) {
			final_time = 1.0;
		}
		else { 
			final_time = 10.0 / speed;
		}
		if (final_time < 1.0) { final_time = 1.0;}
		motion_visualizer.UpdateTimeHorizon(final_time);
		motion_selector.UpdateTimeHorizon(final_time);
	}
	
	void OnGlobalGoal(geometry_msgs::PoseStamped const& global_goal) {
		//ROS_INFO("GOT GLOBAL GOAL");
		carrot_world_frame << global_goal.pose.position.x, global_goal.pose.position.y, flight_altitude; 
		UpdateCarrotOrthoBodyFrame();

		// if (yaw_on) {
		// 	if (carrot_world_frame.norm() > 5 && (global_goal.pose.position.x - pose_global_x) != 0) {
		// 		bearing_azimuth_degrees = 180.0/M_PI*atan2(-(global_goal.pose.position.y - pose_global_y), global_goal.pose.position.x - pose_global_x);
		// 	}
		// }
		std::cout << "bearing_azimuth_degrees is " << bearing_azimuth_degrees << std::endl;

	}

	Vector3 TransformOrthoBodyToWorld(Vector3 const& ortho_body_frame) {
		geometry_msgs::TransformStamped tf;
	    try {
	      tf = tf_buffer_.lookupTransform("world", "ortho_body",
	                                    ros::Time(0), ros::Duration(1/30.0));
	    } catch (tf2::TransformException &ex) {
	      ROS_ERROR("%s", ex.what());
	      return Vector3::Zero();
	    }

	    geometry_msgs::PoseStamped pose_ortho_body_vector = PoseFromVector3(ortho_body_frame, "ortho_body");
    	geometry_msgs::PoseStamped pose_vector_world_frame = PoseFromVector3(Vector3(0,0,0), "world");
    	tf2::doTransform(pose_ortho_body_vector, pose_vector_world_frame, tf);
    	return VectorFromPose(pose_vector_world_frame);
	}

	void OnScan(sensor_msgs::PointCloud2 const& laser_point_cloud_msg) {
		//ROS_INFO("GOT SCAN");
		DepthImageCollisionEvaluator* depth_image_collision_ptr = motion_selector.GetDepthImageCollisionEvaluatorPtr();

		if (depth_image_collision_ptr != nullptr) {
			
			pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2; 
			pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
			
			pcl_conversions::toPCL(laser_point_cloud_msg, *cloud);
			pcl::PointCloud<pcl::PointXYZ>::Ptr xyz_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
			pcl::fromPCLPointCloud2(*cloud,*xyz_cloud);

			depth_image_collision_ptr->UpdateLaserPointCloudPtr(xyz_cloud);
		}


	}

	void UpdateValueGrid(nav_msgs::OccupancyGrid value_grid_msg) {
		auto t1 = std::chrono::high_resolution_clock::now();

		ValueGridEvaluator* value_grid_evaluator_ptr = motion_selector.GetValueGridEvaluatorPtr();
		if (value_grid_evaluator_ptr != nullptr) {
			ValueGrid* value_grid_ptr = value_grid_evaluator_ptr->GetValueGridPtr();
			if (value_grid_ptr != nullptr) {

				value_grid_ptr->SetResolution(value_grid_msg.info.resolution);
				value_grid_ptr->SetSize(value_grid_msg.info.width, value_grid_msg.info.height);
				value_grid_ptr->SetOrigin(value_grid_msg.info.origin.position.x, value_grid_msg.info.origin.position.y);
				value_grid_ptr->SetValues(value_grid_msg.data);

				//motion_selector.PassInUpdatedValueGrid(&value_grid);
				auto t2 = std::chrono::high_resolution_clock::now();
				std::cout << "Whole value grid construction took "
		      		<< std::chrono::duration_cast<std::chrono::microseconds>(t2-t1).count()
		      		<< " microseconds\n";

				std::cout << value_grid_ptr->GetValueOfPosition(carrot_world_frame) << " is value of goal" << std::endl;
				std::cout << value_grid_ptr->GetValueOfPosition(Vector3(0,0,0)) << " is value of world origin" << std::endl;
				std::cout << value_grid_ptr->GetValueOfPosition(Vector3(0,-2.0,0)) << " is value of 1.5 to my right" << std::endl;
			}
		}
	}

	void OnValueGrid(nav_msgs::OccupancyGrid value_grid_msg) {
		ROS_INFO("GOT VALUE GRID");
		UpdateValueGrid(value_grid_msg);
	}


	void OnLocalGoal(geometry_msgs::PoseStamped const& local_goal) {
		//ROS_INFO("GOT LOCAL GOAL");
		carrot_world_frame << local_goal.pose.position.x, local_goal.pose.position.y, flight_altitude; 
		UpdateCarrotOrthoBodyFrame();

		visualization_msgs::Marker marker;
		marker.header.frame_id = "ortho_body";
		marker.header.stamp = ros::Time::now();
		marker.ns = "carrot_namespace";
		marker.id = 1;
		marker.type = visualization_msgs::Marker::SPHERE;
		marker.action = visualization_msgs::Marker::ADD;
		mutex.lock();
		marker.pose.position.x = carrot_ortho_body_frame(0);
		marker.pose.position.y = carrot_ortho_body_frame(1);
		marker.pose.position.z = carrot_ortho_body_frame(2);
		mutex.unlock();
		marker.scale.x = 0.5;
		marker.scale.y = 0.5;
		marker.scale.z = 0.5;
		marker.color.a = 0.5; // Don't forget to set the alpha!
		marker.color.r = 0.9;
		marker.color.g = 0.4;
		marker.color.b = 0.0;
		carrot_pub.publish( marker );
	}

	void OnDepthImage(const sensor_msgs::PointCloud2ConstPtr& point_cloud_msg) {
		// ROS_INFO("GOT POINT CLOUD");
		DepthImageCollisionEvaluator* depth_image_collision_ptr = motion_selector.GetDepthImageCollisionEvaluatorPtr();

		if (depth_image_collision_ptr != nullptr) {


			pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2; 
			pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
			
	    	pcl_conversions::toPCL(*point_cloud_msg, *cloud);
	    	pcl::PointCloud<pcl::PointXYZ>::Ptr xyz_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	    	pcl::fromPCLPointCloud2(*cloud,*xyz_cloud);

	    	mutex.lock();
			depth_image_collision_ptr->UpdatePointCloudPtr(xyz_cloud);
			mutex.unlock();
		}
		ReactToSampledPointCloud();

		
	
	}

	

	void PublishAttitudeSetpoint(Vector3 const& roll_pitch_thrust) { 

		using namespace Eigen;

		// Vector3 pid;
		// double offset;
		// nh.param("z_p", pid(0), 1.5);
		// nh.param("z_i", pid(1), 0.6);
		// nh.param("z_d", pid(2), 0.5);
		// nh.param("z_offset", offset, 0.69);
		// attitude_generator.setGains(pid, offset);

		mavros_msgs::AttitudeTarget setpoint_msg;
		setpoint_msg.header.stamp = ros::Time::now();
		setpoint_msg.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ROLL_RATE 
			| mavros_msgs::AttitudeTarget::IGNORE_PITCH_RATE
			| mavros_msgs::AttitudeTarget::IGNORE_YAW_RATE
			;
		
		// uncomment below for bearing control
		//nh.param("bearing_azimuth_degrees", bearing_azimuth_degrees, 0.0);
		double bearing_error = bearing_azimuth_degrees - set_bearing_azimuth_degrees;

		while(bearing_error > 180) { 
			bearing_error -= 360;
		}
		while(bearing_error < -180) { 
			bearing_error += 360;
		}

		if (abs(bearing_error) < 1.0) {
			set_bearing_azimuth_degrees = bearing_azimuth_degrees;
		}

		else if (bearing_error < 0)  {
			set_bearing_azimuth_degrees -= 1.0;
		}
		else {
			set_bearing_azimuth_degrees += 1.0;
		}
		if (set_bearing_azimuth_degrees > 180.0) {
			set_bearing_azimuth_degrees -= 360.0;
		}
		if (set_bearing_azimuth_degrees < -180.0) {
			set_bearing_azimuth_degrees += 360.0;
		}

		Matrix3f m;
		m =AngleAxisf(-set_bearing_azimuth_degrees*M_PI/180.0, Vector3f::UnitZ())
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
		// Vector3 initial_acceleration = motion_selector.getInitialAcceleration();
		// attitude_setpoint.pose.position.x = initial_acceleration(0);
		// attitude_setpoint.pose.position.y = initial_acceleration(1);
		// attitude_setpoint.pose.position.z = initial_acceleration(2)+5;
		// attitude_setpoint.pose.orientation = setpoint_msg.orientation;
		// attitude_setpoint_visualization_pub.publish( attitude_setpoint );

	}


	ros::Subscriber pose_sub;
	ros::Subscriber velocity_sub;
	ros::Subscriber depth_image_sub;
	ros::Subscriber global_goal_sub;
	ros::Subscriber local_goal_sub;
	ros::Subscriber value_grid_sub;
	ros::Subscriber laser_scan_sub;

	ros::Publisher carrot_pub;
	ros::Publisher gaussian_pub;
	ros::Publisher attitude_thrust_pub;
	ros::Publisher attitude_setpoint_visualization_pub;

	std::vector<ros::Publisher> action_paths_pubs;

	std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
	tf2_ros::Buffer tf_buffer_;

	double start_time = 0.0;
	double final_time = 1.0;

	double bearing_azimuth_degrees = 0.0;
	double set_bearing_azimuth_degrees = 0.0;

	Eigen::Vector4d pose_x_y_z_yaw;

	Eigen::Matrix<Scalar, Eigen::Dynamic, 1> sampling_time_vector;
	size_t num_samples;

	std::mutex mutex;

	Vector3 carrot_world_frame;
	Vector3 carrot_ortho_body_frame;

	size_t best_traj_index = 0;
	Vector3 desired_acceleration;
	Vector3 attitude_thrust_desired;

	MotionSelector motion_selector;
	AttitudeGenerator attitude_generator;

	double pose_global_x = 0;
	double pose_global_y = 0;
	double pose_global_z = 0;
	double pose_global_yaw = 0;

	bool yaw_on = false;
	double soft_top_speed_max = 0.0;
	bool use_depth_image = true;
	double flight_altitude;


	ros::NodeHandle nh;

public:
	MotionVisualizer motion_visualizer;
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};


int main(int argc, char* argv[]) {
	std::cout << "Initializing motion_selector_node" << std::endl;

	ros::init(argc, argv, "MotionSelectorNode");

	MotionSelectorNode motion_selector_node;

	std::cout << "Got through to here" << std::endl;
	ros::Rate spin_rate(100);

	auto t1 = std::chrono::high_resolution_clock::now();
	auto t2 = std::chrono::high_resolution_clock::now();

	size_t counter = 0;

	while (ros::ok()) {
		//t1 = std::chrono::high_resolution_clock::now();
		//motion_selector_node.ReactToSampledPointCloud();
		//t2 = std::chrono::high_resolution_clock::now();
		// std::cout << "ReactToSampledPointCloud took "
  //     		<< std::chrono::duration_cast<std::chrono::microseconds>(t2-t1).count()
  //     		<< " microseconds\n";
		motion_selector_node.PublishCurrentAttitudeSetpoint();
		//motion_selector_node.ReactToSampledPointCloud();

		counter++;
		if (counter > 3) {
			counter = 0;
			motion_selector_node.motion_visualizer.drawAll();
			if (!motion_selector_node.UseDepthImage()) {
				motion_selector_node.ReactToSampledPointCloud();
			}
		}
      	
		ros::spinOnce();
		spin_rate.sleep();
	}
}
