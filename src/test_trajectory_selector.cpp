#include "trajectory_selector.h"
#include <chrono>

int main(int argc, char* argv[]) {
  std::cout << "This has been run!" << std::endl;
  // TrajectorySelector my_trajectory_selector;
  // my_trajectory_selector.Test();
  // my_trajectory_selector.InitializeLibrary();

  // Vector3 initialVelocity = Vector3(13.0, 1.0, 0.3);

  // auto t1 = std::chrono::high_resolution_clock::now();
  // for (Scalar t = 0; t < 1000000; t++) {
  // 	my_trajectory_selector.setInitialVelocity(initialVelocity);
  // }
  // auto t2 = std::chrono::high_resolution_clock::now();
  // std::cout << "setting velocity 1 million times took "
  //             << std::chrono::duration_cast<std::chrono::milliseconds>(t2-t1).count()
  //             << " milliseconds\n";

  // t1 = std::chrono::high_resolution_clock::now();
  // //Eigen::Matrix<Scalar, Eigen::Dynamic, 3> my_matrx = my_trajectory_selector.sampleTrajectoryForDrawing(13, 0.0, 0.5, 10);
  // t2 = std::chrono::high_resolution_clock::now();
  // std::cout << "1000 samples along one trajectory took "
  //             << std::chrono::duration_cast<std::chrono::milliseconds>(t2-t1).count()
  //             << " milliseconds\n";  

}
