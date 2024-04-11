#include "./include/Robot.h" // Include if your classes are in separate files
#include "./include/Controller.h"

#include <Adafruit_PWMServoDriver.h>

extern Adafruit_PWMServoDriver pwm; // Use the external pwm object defined in main.ino
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

using namespace std;

RobotController* robotController = nullptr;
Robot* robot = nullptr;

void setup() {
  Serial.begin(115200);
  pwm.begin();

  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);

  robot = new Robot(); // Now actually creating a Robot instance
  robotController = new RobotController(robot);

  delay(10);
}

bool isMoving = false;
bool rotateLeft = false;
bool rotateRight = false;

void loop() {
    // Check for new serial input
    if (Serial.available() > 0) {
        String command = Serial.readStringUntil('\n');
        command.trim();
        command.toLowerCase();
        char cmd = command.charAt(0);
        Serial.println(cmd);
        
        // Update state based on command
        switch(cmd) {
            case 'a': // Start rotating left
                rotateLeft = true;
                rotateRight = false;
                break;
            case 'd': // Start rotating right
                rotateRight = true;
                rotateLeft = false;
                break;
            case 'z': // Stop any rotation
                rotateLeft = false;
                rotateRight = false;
                break;
            case 'c':
                robotController->openEndEffector();
                break;
            case 'n':
                robotController->closeEndEffector();
                break;
        }
    }
    
    // Continuously act on the current state
    if (rotateLeft) {
        // Continuously rotate left
        robotController->rotateLeft(); // Ensure this method supports continuous action
    } else if (rotateRight) {
        // Continuously rotate right
        robotController->rotateRight(); // Ensure this method supports continuous action
    }

    delay(100); // Small delay to prevent overwhelming the processor
}
