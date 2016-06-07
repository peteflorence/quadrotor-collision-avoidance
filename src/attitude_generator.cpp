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

	double thrust = zPID();

	return Vector3(roll, pitch, thrust);
};

double AttitudeGenerator::zPID() {
	// Proportional term
	double error = z_setpoint - z;
	double Pout = _Kp * error;

	// Integral term
	_integral += error * _dt;
    double Iout = _Ki * _integral;

    // Derivative term
    double derivative = (error - _pre_error) / _dt;
    double Dout = _Kd * derivative;

    // Calculate total output
    double output = Pout + Iout + Dout;

    // Restrict to max/min
    if( output > _max )
        output = _max;
    else if( output < _min )
        output = _min;

    // Save error to previous error
    _pre_error = error;

    return output;

};

