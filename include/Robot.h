#ifndef ROBOT_H
#define ROBOT_H

#include "RobotConfig.h"

class Robot {
public:
    explicit Robot();
    virtual ~Robot() = default;

    // Function to move a servo to a specified angle
    void moveServoToAngle(int servoNum, double angle);

    // Getters
    virtual int getServoCount() const { return servoCount; }
    virtual int getBaseServo() const { return baseServo; }
    virtual int getEndEffectorServo() const { return endEffectorServo; }
    virtual int getServoLinkOne() const { return servoLinkOne; }
    virtual int getServoLinkTwo() const { return servoLinkTwo; }
    int getServoAngle(int servoNum);

    // Setters
    virtual void setServoCount(int count) { servoCount = count; }
    virtual void setBaseServo(int index) { baseServo = index; }
    virtual void setEndEffectorServo(int index) { endEffectorServo = index; }
    virtual void setServoLinkOne(int index) { servoLinkOne = index; }
    virtual void setServoLinkTwo(int index) { servoLinkTwo = index; }

protected:
    int servoCount;
    int baseServo;
    int endEffectorServo;
    int servoLinkOne;
    int servoLinkTwo;
    int currentAngles[SERVO_COUNT];
};

#endif // ROBOT_H
