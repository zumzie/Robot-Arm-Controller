#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "Robot.h" // Include directly if using functionalities specific to RobotEndEffector

class RobotController {
  private:
    Robot* robot; // Use pointer for dynamic allocation or direct object for static

  public:
    RobotController(Robot* robot); // Constructor
    void rotateLeft();
    void rotateRight();

    // Methods for end effector actions
    void openEndEffector();
    void closeEndEffector();
};

#endif
