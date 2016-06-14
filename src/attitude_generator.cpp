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

Vector3 AttitudeGenerator::generateDesiredAttitudeThrust(Vector3 const& desired_acceleration) {
	double a_x = desired_acceleration(0);
	double a_y = desired_acceleration(1);
	double a_z = desired_acceleration(2) + 9.8;

	double roll = atan2(a_y , a_z);
	double pitch = atan2(a_x , std::sqrt(a_y*a_y + a_z*a_z) );
	
	//double thrust = a_x / sin(pitch);
	double roll_degrees = roll * 180/M_PI;
	double pitch_degrees = pitch * 180/M_PI;
	if ((abs(roll_degrees) > 30) || (abs(pitch_degrees) > 30)) {
	//	std::cout << "I just tried to command a roll, pitch of: " << roll_degrees << " " << pitch_degrees << std::endl;
	}

	double thrust = zPID();

	return Vector3(roll, pitch, thrust);
};

void AttitudeGenerator::setGains(Vector3 const& pid, double const& offset) {
    if( fabs(pid(1) - _Ki) > 1e-6 ) _integral = 0.0;
    _Kp = pid(0);
	_Ki = pid(1);
	_Kd = pid(2);
	_offset = offset;
}

double AttitudeGenerator::zPID() {

	// Proportional term
	double error = z_setpoint - z;
	double Pout = _Kp * error;

        std::cout << "dt is " << _dt << std::endl;
        std::cout << "_integral is " << _integral << std::endl;
        std::cout << "z setpoint is " << z_setpoint << std::endl;
        std::cout << "z is          " << z << std::endl;


	// Integral term
	_integral += _Ki * error * _dt;
	if (_integral >_i_max) {
		_integral = _i_max;
	}
	if (_integral < -_i_max ) {
		_integral = -_i_max;
	}


    // Derivative term
    double derivative = (error - _pre_error) / _dt;
    double Dout = _Kd * derivative;

    // Calculate total output
    double output = Pout + _integral + Dout + _offset;

    // Restrict to max/min
    if( output > _max )
        output = _max;
    else if( output < _min )
        output = _min;

    // Save error to previous error
    _pre_error = error;

    return output;

};

