#include "../include/Controller.h"
#include <Adafruit_PWMServoDriver.h>


extern Adafruit_PWMServoDriver pwm;

RobotController::RobotController(Robot* robot) : robot(robot) {}

void RobotController::yawRotation(bool yawRotationDirection) {
    int baseJoint = robot->getServoAngle(robot->getBaseServo());
    Serial.print("Current angle: "); Serial.println(baseJoint);

    int newAngle = baseJoint + (yawRotationDirection ? 1 : -1); // Increment or decrement
    Serial.print("Attempted new angle: "); Serial.println(newAngle);

    // Ensure the new angle is within the valid range
    newAngle = max(min(newAngle, SERVO_MAX_ANGLE), SERVO_MIN_ANGLE);
    Serial.print("Constrained new angle: "); Serial.println(newAngle);

    robot->moveServoToAngle(robot->getBaseServo(), newAngle);
}

void RobotController::pitchRotation(bool pitchRotationDirection, int servo) {

    int jointAngle = robot->getServoAngle(servo);
    Serial.print("Current angle: "); Serial.println(jointAngle);

    int newAngle = jointAngle + (pitchRotationDirection ? 1 : -1); // Increment or decrement
    Serial.print("Attempted new angle: "); Serial.println(newAngle);

    // Ensure the new angle is within the valid range
    newAngle = max(min(newAngle, SERVO_MAX_ANGLE), SERVO_MIN_ANGLE);
    Serial.print("Constrained new angle: "); Serial.println(newAngle);

    robot->moveServoToAngle(servo, newAngle);
}

void RobotController::openEndEffector() {
    robot->moveServoToAngle(robot->getEndEffectorServo(), 60);
}

void RobotController::closeEndEffector() {
    robot->moveServoToAngle(robot->getEndEffectorServo(), 0);
}

void RobotController::resetPositions() {
    robot->moveServoToAngle(robot->getBaseServo(), 0);
    robot->moveServoToAngle(robot->getServoLinkOne(), 0);
    robot->moveServoToAngle(robot->getServoLinkTwo(), 0);
}
