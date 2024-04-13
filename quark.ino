#include "./include/Robot.h" // Include if your classes are in separate files
#include "./include/Controller.h"

#include <Adafruit_PWMServoDriver.h>

extern Adafruit_PWMServoDriver pwm; // Use the external pwm object defined in main.ino
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

using namespace std;

RobotController* robotController = nullptr;
Robot* robot = nullptr;

bool isMoving = false;
bool yawLeft = false;
bool yawRight = false;
bool pitchRotationDirection = false;
bool pitchUp = false;
bool pitchDown = false;
bool pitchUpTwo = false;
bool pitchDownTwo = false;
char cmd;
bool menuActive = true;
char servoNum;

void setup() {
    Serial.begin(115200);
    pwm.begin();
    pwm.setOscillatorFrequency(27000000);
    pwm.setPWMFreq(SERVO_FREQ);

    robot = new Robot(); 
    robotController = new RobotController(robot);

    delay(10);
    displayMenu();
    Serial.println("Resetting Servo Positions to 0");
    robotController->resetPositions();
}

void displayMenu() {
    Serial.println("Welcome, please select an option from the menu:");
    Serial.println("1: User Controls");
    Serial.println("2: Advanced User Controls");
    Serial.println("0: Exit");
}

char getSerialCommand() {
    while (!Serial.available()) {
        delay(10);  // Wait for user input
    }
    String command = Serial.readStringUntil('\n');
    command.trim();
    command.toLowerCase();
    if (command.length() > 0) {
        return command.charAt(0);
    }
    return '\0';
}

void loop() {
    if (!menuActive && Serial.available()) {
        cmd = getSerialCommand();
        switch (cmd) {
            case '1':
                Serial.println("User Controls selected...loading");
                menuActive = true;
                break;
            case '2':
                Serial.println("Advanced User Controls selected...loading");
                // Additional controls logic here
                break;
            case '0':
                Serial.println("Quitting Program...");
                menuActive = false;
                break;
            default:
                displayMenu();
        }
    }

    if (menuActive) {
        manageControls();
    }
}

void manageControls() {
    if (Serial.available()) {
        cmd = getSerialCommand();
        Serial.println(cmd);  // Debug print
        handleUserInput(cmd);
    }

    manageContinuousControls();  // Always check for and manage continuous controls
    delay(100);
}

void handleUserInput(char command) {
    switch (command) {
        case 'a':
            yawLeft = true;
            Serial.println('yawLeft');
            break;
        case 'd':
            yawRight = true;
            Serial.println('yawRight');
            break;
        case 'w':
            pitchUp = true;
            Serial.println('pitchUp');
            break;
        case 's':
            pitchDown = true;
            Serial.println('pitchDown');
            break;
        case 'r':
            pitchUpTwo = true;
            Serial.println('pitchUpTwo');
            break;
        case 'f':
            pitchDownTwo = true;
            Serial.println('pitchDownTwo');
            break;
        case 'z':
            yawLeft = yawRight = pitchUp = pitchDown = pitchUpTwo = pitchDownTwo = false;
            Serial.println('Cancel');
            break;
        case 'c':
            if (robotController) robotController->openEndEffector();
            Serial.println('Open Claw');
            break;
        case 'n':
            if (robotController) robotController->closeEndEffector();
            Serial.println('Close Claw');
            break;
        case '0':
            Serial.println('Back to Main Menu');
            menuActive = false;
            displayMenu();
            break;
    }
}

void manageContinuousControls() {
    // Pitch
    if (pitchUp) robotController->pitchRotation(true, robot->getServoLinkOne());
    if (pitchDown) robotController->pitchRotation(false, robot->getServoLinkOne());

    // Pitch Servo Two
    if (pitchUpTwo) robotController->pitchRotation(true, robot->getServoLinkTwo());
    if (pitchDownTwo) robotController->pitchRotation(false, robot->getServoLinkTwo());
    
    // Yaw
    if (yawLeft) robotController->yawRotation(true);
    if (yawRight) robotController->yawRotation(false);
}

void specificRobotUserControls(RobotController* robotController) {

}
