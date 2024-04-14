#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "Robot.h"

class RobotController {
  private:
    Robot* robot;

  public:
    RobotController(Robot* robot);
    void yawRotation(bool yawRotationDirection);
    void pitchRotation(bool pitchRotationDirection, int servoNum);
    void resetPositions();

    // Methods for end effector actions
    void openEndEffector();
    void closeEndEffector();
};

#endif
