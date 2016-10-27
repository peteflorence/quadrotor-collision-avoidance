#include "motion_library.h"

void MotionLibrary::Initialize2DLibrary(double a_max_horizontal, double min_speed_at_max_acceleration_total, double max_acceleration_total) {

    this->speed_at_acceleration_max = min_speed_at_max_acceleration_total;
    this->max_acceleration_total = max_acceleration_total;
	initial_max_acceleration = a_max_horizontal;
	
	Vector3 zero_initial_velocity = Vector3(0,0,0);

	// Make first motion be zero accelerations
	Vector3 acceleration = Vector3(0,0,0);
	motions.push_back(Motion( acceleration, zero_initial_velocity ));

	// Make next 8 motions sample around maximum horizontal acceleration
	for (double i = 1; i < 9; i++) {
		double theta = (i-1)*2*M_PI/8.0;
		acceleration << cos(theta)*a_max_horizontal, sin(theta)*a_max_horizontal, 0;
		motions.push_back(Motion( acceleration, zero_initial_velocity ));
	}

	// Make next 8 motions sample around 0.6 * maximum horizontal acceleration
	for (double i = 9; i < 17; i++) {
		double theta = (i-1)*2*M_PI/8.0;
		acceleration << cos(theta)*0.6*a_max_horizontal, sin(theta)*0.6*a_max_horizontal, 0;
		motions.push_back(Motion( acceleration, zero_initial_velocity ));
	}

	// Make next 8 motions sample around 0.3 * maximum horizontal acceleration
	for (double i = 17; i < 25; i++) {
		double theta = (i-1)*2*M_PI/8.0;
		acceleration << cos(theta)*0.15*a_max_horizontal, sin(theta)*0.15*a_max_horizontal, 0;
		motions.push_back(Motion( acceleration, zero_initial_velocity ));
	}

	for (size_t index = 0; index < motions.size(); index++) {
		motions.at(index).setAccelerationMax(a_max_horizontal);
	}

	// Gold star motion
	acceleration = Vector3(0,0,0);
	motions.push_back(Motion( acceleration, zero_initial_velocity ));

};

void MotionLibrary::UpdateMaxAcceleration(double speed) {

	new_max_acceleration = ComputeNewMaxAcceleration(speed);

	for (size_t index = 0; index < motions.size(); index++) {
		motions.at(index).setAccelerationMax(new_max_acceleration);
		if (index != 26-1) {
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
	motions.at(26-1).setAcceleration(best_acceleration);
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

size_t MotionLibrary::getNummotions() {
	return motions.size();
};

Vector3 MotionLibrary::getSigmaAtTime(double const& t) {
	//return Vector3(0.01,0.01,0.01) + t*0.2*(Vector3(0.5,0.5,0.5) + 0.5*(initial_velocity.array().abs()).matrix());
	return Vector3(0.01,0.01,0.01) + t*(Vector3(0.5,0.5,0.1) + 0.1*Vector3(1.0,1.0,0.0)*(initial_velocity.norm()) );
};

Vector3 MotionLibrary::getLASERSigmaAtTime(double const& t) {
	//return Vector3(0.01,0.01,0.01) + t*0.2*(Vector3(0.5,0.5,0.5) + 0.5*(initial_velocity_laser_frame.array().abs()).matrix());
	return Vector3(0.01,0.01,0.01) + t*(Vector3(0.5,0.5,0.1) + 0.1*Vector3(1.0,1.0,0.0)*(initial_velocity.norm()) );
};

Vector3 MotionLibrary::getRDFSigmaAtTime(double const& t) const {
	//return Vector3(0.01,0.01,0.01) + t*0.2*(Vector3(0.5,0.5,0.5) + 0.5*(initial_velocity_rdf_frame.array().abs()).matrix());
	return Vector3(0.01,0.01,0.01) + t*(Vector3(0.5,0.1,0.5) + 0.1*Vector3(1.0,0.0,1.0)*(initial_velocity.norm()) );
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
