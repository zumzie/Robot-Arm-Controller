#include "../include/Controller.h"
#include <Adafruit_PWMServoDriver.h>


extern Adafruit_PWMServoDriver pwm;

RobotController::RobotController(Robot* robot) : robot(robot) {}

void RobotController::rotateLeft() {
    int baseServoAngle = robot->getServoAngle(robot->getBaseServo());
    int newAngle = baseServoAngle + 1;
    robot->moveServoToAngle(robot->getBaseServo(), newAngle);
}

void RobotController::rotateRight() {
    int baseServoAngle = robot->getServoAngle(robot->getBaseServo());
    int newAngle = baseServoAngle - 1;
    robot->moveServoToAngle(robot->getBaseServo(), newAngle);
}

void RobotController::openEndEffector() {
    robot->moveServoToAngle(robot->getEndEffectorServo(), 60);
}

void RobotController::closeEndEffector() {
    robot->moveServoToAngle(robot->getEndEffectorServo(), 0);
}
