#include <signal.h>
#include <sys/time.h>
#include <unistd.h>

#include <cassert>
#include <cmath>
#include <cstring>
#include <fstream>
#include <iostream>

#include <gflags/gflags.h>

#include "kinova_driver_common.h"

DEFINE_string(optimal_z, "",
              "A file containing the optimal z parameters for this robot.");

int main(int argc, char** argv) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  if (InitializeApi() != NO_ERROR_KINOVA) {
    return 1;
  }

  if (!FLAGS_optimal_z.empty()) {
    std::ifstream z_file(FLAGS_optimal_z);
    if (!z_file.is_open()) {
      std::cerr << "Unable to open: " << FLAGS_optimal_z << "\n";
      return 1;
    }
    float optimal_z[GRAVITY_PARAM_SIZE];
    memset(optimal_z, 0, sizeof(optimal_z));

    std::cout << "optimal z: ";
    for (int i = 0; i < OPTIMAL_Z_PARAM_SIZE_7DOF; ++i) {
      std::string line;
      if (!std::getline(z_file, line)) {
        std::cerr << "Failed reading z parameters\n";
        return 1;
      }
      optimal_z[i] = std::atof(line.c_str());
      std::cout << optimal_z[i] << " ";
    }
    std::cout <<  "\n";

    // I'm not sure what to make of the arm returning JACO_NACK_NORMAL, but
    // the parameters still seem to improve gravity estimation even when we
    // get that result code.
    int result = SdkSetGravityOptimalZParam(optimal_z);
    if (result != NO_ERROR_KINOVA && result != JACO_NACK_NORMAL) {
      std::cerr << "Failed to set optimal z parameters: " << result << "\n";
      //return 1;
    }

    result = SdkSetGravityType(OPTIMAL);
    if (result != NO_ERROR_KINOVA) {
      std::cerr << "Failed to set gravity type: " << result << "\n";
      return 1;
    }
    std::cout << "Set optimal Z parameters.\n";
  } else {
    int result = SdkSetGravityType(MANUAL_INPUT);
    if (result != NO_ERROR_KINOVA) {
      std::cerr << "Failed to set gravity type: " << result << "\n";
      return 1;
    }
  }


#if 0
  AngularPosition current_position;
  GetAngularPosition(current_position);
  std::cout << "Current position: "
            << current_position.Actuators.Actuator1
            << " " << current_position.Actuators.Actuator2
            << " " << current_position.Actuators.Actuator3
            << " " << current_position.Actuators.Actuator4
            << " " << current_position.Actuators.Actuator5
            << " " << current_position.Actuators.Actuator6
            << " " << current_position.Actuators.Actuator7
            << "\n";
#endif

  float torque_command[COMMAND_SIZE];
  for (int i = 0; i < COMMAND_SIZE; ++i) {
    torque_command[COMMAND_SIZE] = 0;
  }

#if 0
  int i = SwitchTrajectoryTorque(POSITION);
  int mode = -1;
  GetTrajectoryTorqueMode(mode);
  std::cout << "Switched: " << i
            <<  " mode: " << mode
            << "\n";

  MoveHome();
#endif

  AngularPosition current_torque;
  GetAngularForceGravityFree(current_torque);
  //GetAngularForce(current_torque);

  torque_command[0] = current_torque.Actuators.Actuator1;
  torque_command[1] = current_torque.Actuators.Actuator2;
  torque_command[2] = current_torque.Actuators.Actuator3;
  torque_command[3] = current_torque.Actuators.Actuator4;
  torque_command[4] = current_torque.Actuators.Actuator5;
  torque_command[5] = current_torque.Actuators.Actuator6;
  torque_command[6] = current_torque.Actuators.Actuator7;

  std::cout << "Initial torque: "
                << current_torque.Actuators.Actuator1
                << " " << current_torque.Actuators.Actuator2
                << " " << current_torque.Actuators.Actuator3
                << " " << current_torque.Actuators.Actuator4
                << " " << current_torque.Actuators.Actuator5
                << " " << current_torque.Actuators.Actuator6
                << " " << current_torque.Actuators.Actuator7
                << "\n";

  std::cout << "Starting Torque Control.\n";

  int j = SetTorqueControlType(DIRECTTORQUE);
  std::cout << "Set torque control type: " << j << "\n";

  //int cmd = SendAngularTorqueCommand(torque_command);
  //std::cout << "Initial command: " << cmd << "\n";

  int i = SwitchTrajectoryTorque(TORQUE);
  int mode = -1;
  GetTrajectoryTorqueMode(mode);
  std::cout << "Switched: " << i
            <<  " mode: " << mode
            << "\n";
  while (mode == 0) {
    usleep(5000);
    GetTrajectoryTorqueMode(mode);

    //    std::cout << "Switch failed.";
    //    return 1;
  }
  std::cout << "Entered torque mode: " << mode << "\n";
  //usleep(1000000);

  //std::cout << "Sending command\n";

  // This is as a fraction of velocity.
  SetTorqueSafetyFactor(0.9);


  //SetTorqueVibrationController(0.5);

  torque_command[5] += 1;
  for (int i = 0; i < 200; ++i) {
    //torque_command[5] = -1.;
    GetTrajectoryTorqueMode(mode);
    if (mode == 0) {
      std::cout << "Lost torque mode on pass: " << i << "\n";
      return 1;
    }

    SendAngularTorqueCommand(torque_command);
    if (i % 10 == 0) {
      GetAngularForceGravityFree(current_torque);
      std::cout << "Current torque: "
                << current_torque.Actuators.Actuator1
                << " " << current_torque.Actuators.Actuator2
                << " " << current_torque.Actuators.Actuator3
                << " " << current_torque.Actuators.Actuator4
                << " " << current_torque.Actuators.Actuator5
                << " " << current_torque.Actuators.Actuator6
                << " " << current_torque.Actuators.Actuator7
                << "\n";
    }
    usleep(10000);
    //usleep(6000);
  }

  std::cout << "Ending Torque Control\n";
  SwitchTrajectoryTorque(POSITION);
  CloseAPI();

  return 0;
}
