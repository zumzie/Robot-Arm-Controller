#ifndef ROBOTCONFIG_H
#define ROBOTCONFIG_H

#define SERVOMIN  150
#define SERVOMAX  600
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates
#define USMIN  600 // This is the rounded 'minimum' microsecond length based on the minimum pulse of 150
#define USMAX  2400 // This is the rounded 'maximum' microsecond length based on the maximum pulse of 600
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updat



// Servo Configuration

// Declare the indices for specific servos
#define BASE_SERVO_INDEX 0
#define END_EFFECTOR_SERVO_INDEX 3
#define SERVO_ONE_INDEX 1
#define SERVO_TWO_INDEX 2
#define SERVO_COUNT 4

// Declare the size of servoJoints for external visibility
#define SERVO_JOINTS_SIZE 2

// Declare an external array for servo joints
extern int servoJoints[SERVO_JOINTS_SIZE];

#endif
