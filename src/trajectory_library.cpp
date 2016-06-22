#include "trajectory_library.h"
#include <chrono>
#include <math.h>

void TrajectoryLibrary::Initialize2DLibrary(double const& final_time) {
	//double a_max_horizontal = sqrt(a_max*a_max - 9.8*9.8);
	double a_max_horizontal = 9.8*0.5;
	Vector3 zero_initial_velocity = Vector3(0,0,0);

	// Make first trajectory be zero accelerations
	Vector3 acceleration = Vector3(0,0,0);
	trajectories.push_back(Trajectory( acceleration, zero_initial_velocity ));

	// Make next 8 trajectories sample around maximum horizontal acceleration
	for (double i = 1; i < 9; i++) {
		double theta = (i-1)*2*M_PI/8.0;
		acceleration << cos(theta)*a_max_horizontal, sin(theta)*a_max_horizontal, 0;
		trajectories.push_back(Trajectory( acceleration, zero_initial_velocity ));
	}

	// Make next 8 trajectories sample around 0.6 * maximum horizontal acceleration
	for (double i = 9; i < 17; i++) {
		double theta = (i-1)*2*M_PI/8.0;
		acceleration << cos(theta)*0.6*a_max_horizontal, sin(theta)*0.6*a_max_horizontal, 0;
		trajectories.push_back(Trajectory( acceleration, zero_initial_velocity ));
	}

	// Make next 8 trajectories sample around 0.3 * maximum horizontal acceleration
	for (double i = 17; i < 25; i++) {
		double theta = (i-1)*2*M_PI/8.0;
		acceleration << cos(theta)*0.15*a_max_horizontal, sin(theta)*0.15*a_max_horizontal, 0;
		trajectories.push_back(Trajectory( acceleration, zero_initial_velocity ));
	}

	this->final_time = final_time; 

	for (size_t index = 0; index < trajectories.size(); index++) {
		trajectories.at(index).setAccelerationMax(a_max_horizontal);
	}

};

void TrajectoryLibrary::updateInitialAcceleration() {
	double acceleration_from_thrust = thrust * 9.8/0.7;
	double a_x_initial = acceleration_from_thrust * sin(pitch);
	double a_y_initial = -acceleration_from_thrust * cos(pitch)*sin(roll);
	//double a_z_initial = acceleration_from_thrust * cos(pitch) * cos(roll)-9.8;
	double a_z_initial = 0;

	initial_acceleration = Vector3(a_x_initial, a_y_initial, a_z_initial);

	for (size_t index = 0; index < trajectories.size(); index++) {
		trajectories.at(index).setInitialAcceleration(initial_acceleration);
	}

	return;
};



void TrajectoryLibrary::setInitialVelocity(Vector3 const& velocity) {
	initial_velocity = velocity;
	initial_velocity(2) = 0; // WARNING MUST GET RID OF THIS FOR 3D FLIGHT
	for (size_t index = 0; index < trajectories.size(); index++) {
		trajectories.at(index).setInitialVelocity(initial_velocity);
	}
};


Trajectory TrajectoryLibrary::getTrajectoryFromIndex(size_t index) {
	return trajectories.at(index);
};

size_t TrajectoryLibrary::getNumTrajectories() {
	return trajectories.size();
};

Vector3 TrajectoryLibrary::getSigmaAtTime(double const& t) {
	return Vector3(0.01,0.01,0.01) + t*(Vector3(0.5,0.5,0.5) + 0.1*(initial_velocity.array().abs()).matrix());
};

Vector3 TrajectoryLibrary::getInverseSigmaAtTime(double const& t) {
	Vector3 sigma = getSigmaAtTime(t);
	return Vector3(1.0/sigma(0), 1.0/sigma(1), 1.0/sigma(2));
};

void TrajectoryLibrary::setInitialAccelerationLASER(Vector3 const& initial_acceleration_laser_frame) {
	this->initial_acceleration_laser_frame = initial_acceleration_laser_frame;
	for (size_t index = 0; index < trajectories.size(); index++) {
		trajectories.at(index).setInitialAccelerationLASER(initial_acceleration_laser_frame);
	}
	return;
};

void TrajectoryLibrary::setInitialVelocityLASER(Vector3 const& initial_velocity_laser_frame) {
	this->initial_velocity_laser_frame = initial_velocity_laser_frame;
	for (size_t index = 0; index < trajectories.size(); index++) {
		trajectories.at(index).setInitialVelocityLASER(initial_velocity_laser_frame);
	}
	return;
};

Vector3 TrajectoryLibrary::getLASERSigmaAtTime(double const& t) {
	return Vector3(0.01,0.01,0.01) + t*(Vector3(0.5,0.5,0.5) + 0.1*(initial_velocity_rdf_frame.array().abs()).matrix());
};

Vector3 TrajectoryLibrary::getLASERInverseSigmaAtTime(double const& t) {
	Vector3 RDFsigma = getLASERSigmaAtTime(t);
	return Vector3(1.0/RDFsigma(0), 1.0/RDFsigma(1), 1.0/RDFsigma(2));
};

void TrajectoryLibrary::setInitialAccelerationRDF(Vector3 const& initial_acceleration_rdf_frame) {
	this->initial_acceleration_laser_frame = initial_acceleration_laser_frame;
	for (size_t index = 0; index < trajectories.size(); index++) {
		trajectories.at(index).setInitialAccelerationLASER(initial_acceleration_laser_frame);
	}
	return;
};

void TrajectoryLibrary::setInitialVelocityRDF(Vector3 const& initial_velocity_rdf_frame) {
	this->initial_velocity_rdf_frame = initial_velocity_rdf_frame;
	for (size_t index = 0; index < trajectories.size(); index++) {
		trajectories.at(index).setInitialVelocityRDF(initial_velocity_rdf_frame);
	}
	return;
};

Vector3 TrajectoryLibrary::getRDFSigmaAtTime(double const& t) {
	return Vector3(0.01,0.01,0.01) + t*(Vector3(0.5,0.5,0.5) + 0.1*(initial_velocity_rdf_frame.array().abs()).matrix());
};

Vector3 TrajectoryLibrary::getRDFInverseSigmaAtTime(double const& t) {
	Vector3 RDFsigma = getRDFSigmaAtTime(t);
	return Vector3(1.0/RDFsigma(0), 1.0/RDFsigma(1), 1.0/RDFsigma(2));
};