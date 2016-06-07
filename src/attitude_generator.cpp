#include "attitude_generator.h"

void AttitudeGenerator::TestAttitudeGenerator() {

  std::cout << "Printing from inside TrajectoryEvaluator " << std::endl;  

}

Vector3 AttitudeGenerator::generateDesiredAttitude(Vector3 const& desired_acceleration) {
	return Vector3(0,0,0);
};