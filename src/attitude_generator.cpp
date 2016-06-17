#include "attitude_generator.h"

void AttitudeGenerator::TestAttitudeGenerator() {

  std::cout << "Printing from inside TrajectoryEvaluator " << std::endl;  

}

void AttitudeGenerator::setZ(double z) {
	this->z = z;
};

void AttitudeGenerator::setZsetpoint(double z_setpoint) {
	this->z_setpoint = z_setpoint;
};

void AttitudeGenerator::setZvelocity(double const& z_velocity) {
	this->z_velocity = z_velocity;
};

Vector3 AttitudeGenerator::generateDesiredAttitudeThrust(Vector3 const& desired_acceleration) {
	double a_x = desired_acceleration(0);
	double a_y = desired_acceleration(1);
	double a_z = desired_acceleration(2) + 9.8;
	if (a_z == 0) {
		a_z = 1;
	}

	double roll = atan2(a_y , a_z);
	double pitch = atan2(a_x , a_z);
	if (roll > 70) {roll = 70;}; if (roll <-70) {roll = -70;};
	if (pitch > 70) {pitch = 70;}; if (pitch <-70) {pitch = -70;};


	double roll_degrees = roll * 180/M_PI;
	double pitch_degrees = pitch * 180/M_PI;
	if ((abs(roll_degrees) > 30) || (abs(pitch_degrees) > 30)) {
	//	std::cout << "I just tried to command a roll, pitch of: " << roll_degrees << " " << pitch_degrees << std::endl;
	}

	double thrust = zPID(roll, pitch);

	return Vector3(roll, pitch, thrust);
};

void AttitudeGenerator::setGains(Vector3 const& pid, double const& offset) {
    if( fabs(pid(1) - _Ki) > 1e-6 ) _integral = 0.0;
    _Kp = pid(0);
	_Ki = pid(1);
	_Kd = pid(2);
	_offset = offset;
}

double AttitudeGenerator::zPID(double roll, double pitch) {

	// Proportional term
	double error = z_setpoint - z;
	double Pout = _Kp * error;

    //std::cout << "dt is " << _dt << std::endl;
    //std::cout << "_integral is " << _integral << std::endl;
    //std::cout << "z setpoint is " << z_setpoint << std::endl;
    //std::cout << "z is          " << z << std::endl;

	// Integral term
	_integral += _Ki * error * _dt;
	if (_integral >_i_max) {
		_integral = _i_max;
	}
	if (_integral < -_i_max ) {
		_integral = -_i_max;
	}


    // Derivative term
    double velocity_error = z_velocity_setpoint - z_velocity;
    double Dout = _Kd * velocity_error;

    // Calculate total output
    double offset_tilted = _offset/cos(pitch)/cos(roll);
    double output = Pout + _integral + Dout + offset_tilted;

    // Restrict to max/min
    if( output > _max )
        output = _max;
    else if( output < _min )
        output = _min;

    // Save error to previous error
    _pre_error = error;

    return output;

};

