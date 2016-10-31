#include "motion_library.h"

void MotionLibrary::Initialize2DLibrary(double acceleration_interpolation_min, double speed_at_acceleration_max, double max_acceleration_total) {

    this->speed_at_acceleration_max = speed_at_acceleration_max;
    this->max_acceleration_total = max_acceleration_total;
	this->initial_max_acceleration = acceleration_interpolation_min;
	
	Vector3 zero_initial_velocity = Vector3(0,0,0);

	// Optimal motion (ignoring obstacles) is 0th motion, initialize to 0 acceleration but this gets recalculated in motion_selector_node
	Vector3 acceleration = Vector3(0,0,0);
	motions.push_back(Motion( acceleration, zero_initial_velocity ));

	// Next motion is 0 acceleration
	acceleration = Vector3(0,0,0);
	motions.push_back(Motion( acceleration, zero_initial_velocity ));

	// Then build up more motions by sampling over accelerations
	BuildMotionsSamplingAroundHorizontalCircle(initial_max_acceleration, 8, 0.0);
	BuildMotionsSamplingAroundHorizontalCircle(0.6*initial_max_acceleration, 8, 0.0);
	BuildMotionsSamplingAroundHorizontalCircle(0.15*initial_max_acceleration, 8, 0.0);

	for (size_t index = 0; index < motions.size(); index++) {
		motions.at(index).setAccelerationMax(acceleration_interpolation_min);
	}

};

void MotionLibrary::BuildMotionsSamplingAroundHorizontalCircle(double horizontal_acceleration_radius, size_t num_samples_around_circle, double vertical_acceleration) {
	for (double i = 0; i < num_samples_around_circle; i++) {
		double theta = i*2*M_PI/num_samples_around_circle;
		Vector3 acceleration; cos << acceleration(theta)*horizontal_acceleration_radius, sin(theta)*horizontal_acceleration_radius, vertical_acceleration;
		Vector3 zero_initial_velocity = Vector3(0,0,0);
		motions.push_back(Motion( acceleration, zero_initial_velocity ));
	}
	return;
}

void MotionLibrary::UpdateMaxAcceleration(double speed) {

	new_max_acceleration = ComputeNewMaxAcceleration(speed);

	for (size_t index = 0; index < motions.size(); index++) {
		motions.at(index).setAccelerationMax(new_max_acceleration);
		if (index != 0) {
			motions.at(index).ScaleAcceleration(new_max_acceleration/initial_max_acceleration);
		}
	}
}

double MotionLibrary::ComputeNewMaxAcceleration(double speed) {
	if (speed > speed_at_acceleration_max) {
		return max_acceleration_total;
	}

	return speed * (max_acceleration_total - initial_max_acceleration) / speed_at_acceleration_max + initial_max_acceleration;

}

double MotionLibrary::getNewMaxAcceleration() const {
	return new_max_acceleration;
}

void MotionLibrary::updateInitialAcceleration() {
	double acceleration_from_thrust = thrust * 9.8/0.7;
	double a_x_initial = acceleration_from_thrust * sin(pitch);
	double a_y_initial = -acceleration_from_thrust * cos(pitch)*sin(roll);
	//double a_z_initial = acceleration_from_thrust * cos(pitch) * cos(roll)-9.8;
	double a_z_initial = 0;

	initial_acceleration = Vector3(a_x_initial, a_y_initial, a_z_initial);

	for (size_t index = 0; index < motions.size(); index++) {
		motions.at(index).setInitialAcceleration(initial_acceleration);
	}

	return;
};

void MotionLibrary::setBestAccelerationMotion(Vector3 best_acceleration) {
	motions.at(0).setAcceleration(best_acceleration);
}

void MotionLibrary::setInitialVelocity(Vector3 const& velocity) {
	initial_velocity = velocity;
	initial_velocity(2) = 0; // WARNING MUST GET RID OF THIS FOR 3D FLIGHT
	for (size_t index = 0; index < motions.size(); index++) {
		motions.at(index).setInitialVelocity(initial_velocity);
	}
};


Motion MotionLibrary::getMotionFromIndex(size_t index) {
	return motions.at(index);
};

size_t MotionLibrary::getNumMotions() {
	return motions.size();
};

Vector3 MotionLibrary::getSigmaAtTime(double const& t) {
	//return Vector3(0.01,0.01,0.01) + t*0.2*(Vector3(0.5,0.5,0.5) + 0.5*(initial_velocity.array().abs()).matrix());
	return Vector3(0.01,0.01,0.01) + t*(Vector3(0.5,0.5,0.2) + 0.1*Vector3(1.0,1.0,0.0)*(initial_velocity.norm()) );
};

Vector3 MotionLibrary::getLASERSigmaAtTime(double const& t) {
	//return Vector3(0.01,0.01,0.01) + t*0.2*(Vector3(0.5,0.5,0.5) + 0.5*(initial_velocity_laser_frame.array().abs()).matrix());
	return Vector3(0.01,0.01,0.01) + t*(Vector3(0.5,0.5,0.2) + 0.1*Vector3(1.0,1.0,0.0)*(initial_velocity.norm()) );
};

Vector3 MotionLibrary::getRDFSigmaAtTime(double const& t) const {
	//return Vector3(0.01,0.01,0.01) + t*0.2*(Vector3(0.5,0.5,0.5) + 0.5*(initial_velocity_rdf_frame.array().abs()).matrix());
	return Vector3(0.01,0.01,0.01) + t*(Vector3(0.5,0.2,0.5) + 0.1*Vector3(1.0,0.0,1.0)*(initial_velocity.norm()) );
};

Vector3 MotionLibrary::getInverseSigmaAtTime(double const& t) {
	Vector3 sigma = getSigmaAtTime(t);
	return Vector3(1.0/sigma(0), 1.0/sigma(1), 1.0/sigma(2));
};

void MotionLibrary::setInitialAccelerationLASER(Vector3 const& initial_acceleration_laser_frame) {
	this->initial_acceleration_laser_frame = initial_acceleration_laser_frame;
	for (size_t index = 0; index < motions.size(); index++) {
		motions.at(index).setInitialAccelerationLASER(initial_acceleration_laser_frame);
	}
	return;
};

void MotionLibrary::setInitialVelocityLASER(Vector3 const& initial_velocity_laser_frame) {
	this->initial_velocity_laser_frame = initial_velocity_laser_frame;
	for (size_t index = 0; index < motions.size(); index++) {
		motions.at(index).setInitialVelocityLASER(initial_velocity_laser_frame);
	}
	return;
};

Vector3 MotionLibrary::getLASERInverseSigmaAtTime(double const& t) {
	Vector3 LASERsigma = getLASERSigmaAtTime(t);
	return Vector3(1.0/LASERsigma(0), 1.0/LASERsigma(1), 1.0/LASERsigma(2));
};

void MotionLibrary::setInitialAccelerationRDF(Vector3 const& initial_acceleration_rdf_frame) {
	this->initial_acceleration_laser_frame = initial_acceleration_laser_frame;
	for (size_t index = 0; index < motions.size(); index++) {
		motions.at(index).setInitialAccelerationRDF(initial_acceleration_laser_frame);
	}
	return;
};

void MotionLibrary::setInitialVelocityRDF(Vector3 const& initial_velocity_rdf_frame) {
	this->initial_velocity_rdf_frame = initial_velocity_rdf_frame;
	for (size_t index = 0; index < motions.size(); index++) {
		motions.at(index).setInitialVelocityRDF(initial_velocity_rdf_frame);
	}
	return;
};

std::vector<Vector3> MotionLibrary::getRDFSampledInitialVelocity(size_t n) {
	std::random_device rd;
	std::mt19937 gen(rd());


	double x_velocity = initial_velocity_rdf_frame(0);
	double y_velocity = initial_velocity_rdf_frame(1);
	double z_velocity = initial_velocity_rdf_frame(2);

	sampled_velocities.clear();

	for (int i = 0; i < n; i++) {
		std::normal_distribution<> dx(x_velocity,0.05);
		std::normal_distribution<> dy(y_velocity,0.05);
		std::normal_distribution<> dz(z_velocity,0.05);
		sampled_velocities.push_back(Vector3(dx(gen),dy(gen),dz(gen)));
	}

	return sampled_velocities;
};

Vector3 MotionLibrary::getRDFInverseSigmaAtTime(double const& t) const {
	Vector3 RDFsigma = getRDFSigmaAtTime(t);
	return Vector3(1.0/RDFsigma(0), 1.0/RDFsigma(1), 1.0/RDFsigma(2));
};

void MotionLibrary::setMaxAccelerationTotal(double max_accel) {
  this->max_acceleration_total = max_accel;
}

void MotionLibrary::setMinSpeedAtMaxAccelerationTotal(double speed) {
  this->speed_at_acceleration_max = speed;
}
