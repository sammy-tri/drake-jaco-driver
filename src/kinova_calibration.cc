#include <signal.h>
#include <sys/time.h>
#include <unistd.h>

#include <cassert>
#include <cmath>
#include <cstring>
#include <iostream>

#include <gflags/gflags.h>

#include "kinova_driver_common.h"

DEFINE_bool(gravity, false,
            "Run the gravity estimation sequence to determine the optimal Z "
            "parameters.  This can take some time to complete.");

int main(int argc, char** argv) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  if (InitializeApi() != NO_ERROR_KINOVA) {
    return 1;
  }

  AngularPosition current_position;
  GetAngularPosition(current_position);

  std::cout << "Moving to torque zero position.\n";

  TrajectoryPoint command;
  command.InitStruct();
  command.LimitationsActive = 0;
  command.SynchroType = 1;
  command.Position.Type = ANGULAR_POSITION;
  command.Position.HandMode = HAND_NOMOVEMENT;
  command.Position.Actuators.Actuator1 = current_position.Actuators.Actuator1;
  command.Position.Actuators.Actuator2 = 180;
  command.Position.Actuators.Actuator3 = 90;
  command.Position.Actuators.Actuator4 = 180;
  command.Position.Actuators.Actuator5 = 0;
  command.Position.Actuators.Actuator6 = 180;
  command.Position.Actuators.Actuator7 = 180;

  SendBasicTrajectory(command);

  double norm = 0;
  do {
    GetAngularPosition(current_position);
    norm += std::abs(current_position.Actuators.Actuator1 -
                     command.Position.Actuators.Actuator1);
    norm += std::abs(current_position.Actuators.Actuator2 -
                     command.Position.Actuators.Actuator2);
    norm += std::abs(current_position.Actuators.Actuator3 -
                     command.Position.Actuators.Actuator3);
    norm += std::abs(current_position.Actuators.Actuator4 -
                     command.Position.Actuators.Actuator4);
    norm += std::abs(current_position.Actuators.Actuator5 -
                     command.Position.Actuators.Actuator5);
    norm += std::abs(current_position.Actuators.Actuator6 -
                     command.Position.Actuators.Actuator6);
    norm += std::abs(current_position.Actuators.Actuator7 -
                     command.Position.Actuators.Actuator7);
    norm /= 7.;
  } while (norm > 1);

  std::cout << "Setting actuator torque zero.\n";
  for (int i = 0; i < 7; ++i) {
    int result = SetTorqueZero(i + 16);
    if (result != NO_ERROR_KINOVA) {
      std::cerr << "Setting torque zero failed on actuator "
                << i << ": " << result << "\n";
      break;
    }
    usleep(100000);
  }

  if (FLAGS_gravity) {
    std::cout << "Running gravity estimation\n";
    float optimal_z[OPTIMAL_Z_PARAM_SIZE_7DOF];
    memset(optimal_z, 0, sizeof(optimal_z));
    int result =
        RunGravityZEstimationSequence7DOF(SPHERICAL_7DOF_SERVICE, optimal_z);
    if (result != NO_ERROR_KINOVA) {
      std::cerr << "Gravity estimation failed: " << result << std::endl;
      return 1;
    }
  }

  CloseAPI();

  return 0;
}
