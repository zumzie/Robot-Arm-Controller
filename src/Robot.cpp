#include "../include/Robot.h"
#include "../include/RobotConfig.h"
#include <Adafruit_PWMServoDriver.h>


extern Adafruit_PWMServoDriver pwm;

Robot::Robot()
    : servoCount(SERVO_COUNT), baseServo(BASE_SERVO_INDEX), endEffectorServo(END_EFFECTOR_SERVO_INDEX),
      servoLinkOne(SERVO_ONE_INDEX), servoLinkTwo(SERVO_TWO_INDEX) {}

void Robot::moveServoToAngle(int servoNum, double angle) {
    // Make sure the servoNum is within bounds
    if(servoNum >= 0 && servoNum < servoCount) {
        int pulseLength = map(angle, 0, 180, SERVOMIN, SERVOMAX);
        pwm.setPWM(servoNum, 0, pulseLength);
        currentAngles[servoNum] = angle; // Update the stored angle
    }
}

int Robot::getServoAngle(int servoNum) {
    if(servoNum >= 0 && servoNum < servoCount) {
        return currentAngles[servoNum];
    }
    return -1; // Return an invalid value if servoNum is out of bounds
}