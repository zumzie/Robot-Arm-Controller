#include "./include/Robot.h"
#include "./include/Controller.h"

#include <Adafruit_PWMServoDriver.h>

extern Adafruit_PWMServoDriver pwm;
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
String cmd;
bool menuActive = true;
char servoNum;
int servoIndexValues[3] = {0};


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

void splitStringToInt(const String& input, int* output, int maxElements) {
    int idx = 0;
    int startPos = 0;
    int endPos = 0;

    while (idx < maxElements && endPos >= 0) {
        endPos = input.indexOf(' ', startPos);
        if (endPos == -1) {
            output[idx++] = input.substring(startPos).toInt();
        } else {
            output[idx++] = input.substring(startPos, endPos).toInt();
            startPos = endPos + 1;
        }
    }
}

String readLineFromSerial() {
    String input = "";
    while (Serial.available()) {
        char c = Serial.read();
        if (c == '\n') break;
        input += c;
    }
    input.trim();
    input.toLowerCase();
    return input;
}



void displayMenu() {
    Serial.println("Welcome, please select an option from the menu:");
    Serial.println("1: User Controls");
    Serial.println("2: Advanced User Controls");
    Serial.println("0: Exit");
    Serial.flush();
}

String getSerialCommand() {
    while (!Serial.available()) {
        delay(10);
    }
    String command = "";
    while (Serial.available()) {
        char c = Serial.read();
        if (c == '\n' || c == '\r') {
            break;
        }
        command += c;
    }
    command.trim();
    command.toLowerCase();
    return command;
}

void loop() {
    if (Serial.available()) {
        cmd = getSerialCommand();
        if (cmd == "1") {
            Serial.println("User Controls selected...loading");
            menuActive = true;
            manageControls();
        } else if (cmd == "2") {
            Serial.println("Advanced2 User Controls selected...loading");
            menuActive = true;
            manageControlsAdvanced();
        } else if (cmd == "0") {
            Serial.println("Quitting Program...");
            menuActive = false;
            return;
        } else {
            displayMenu();
        }
    }

    if (menuActive) {
        manageContinuousControls();
    }
    delay(100);
}


void manageControls() {
    Serial.println("Enter '0' to go back.");
    Serial.flush();
    String command;
    while (true) {
        if (Serial.available()) {
            command = getSerialCommand();
            handleUserInput(command);
        }
        if (command == "z") {
            handleUserInput("z");
        } else if (command == "0") {
            break;
        }
        delay(100);
        manageContinuousControls();
    }
}


void manageControlsAdvanced() {
    Serial.println("Enter 'z' to go back. Enter servo angles as '30 40 70'.");
    while (menuActive) {
        if (Serial.available()) {
            String command = getSerialCommand();
            if (command == "z") {
                menuActive = false;
                displayMenu();
                break;
            } else {
                handleUserInputAdvanced(command);
            }
        }
        delay(100);
    }
}

void handleUserInput(const String& command) {
    if (command == "a") {
        yawLeft = true;
        Serial.println("yawLeft");
    } else if (command == "d") {
        yawRight = true;
        Serial.println("yawRight");
    } else if (command == "w") {
        pitchUp = true;
        Serial.println("pitchUp");
    } else if (command == "s") {
        pitchDown = true;
        Serial.println("pitchDown");
    } else if (command == "r") {
        pitchUpTwo = true;
        Serial.println("pitchUpTwo");
    } else if (command == "f") {
        pitchDownTwo = true;
        Serial.println("pitchDownTwo");
    } else if (command == "z") {
        yawLeft = yawRight = pitchUp = pitchDown = pitchUpTwo = pitchDownTwo = false;
    } else if (command == "c") {
        if (robotController) robotController->openEndEffector();
        Serial.println("Open Claw");
    } else if (command == "n") {
        if (robotController) robotController->closeEndEffector();
        Serial.println("Close Claw");
    } else if (command == "0") {
        Serial.println("Back to Main Menu");
        menuActive = false;
        displayMenu();
    }
}



void manageContinuousControls() {
    
    // Yaw
    if (yawLeft) robotController->yawRotation(true);
    if (yawRight) robotController->yawRotation(false);

    // Pitch One
    if (pitchUp) robotController->pitchRotation(true, robot->getServoLinkOne());
    if (pitchDown) robotController->pitchRotation(false, robot->getServoLinkOne());


    // Pitch Two
    if (pitchUpTwo) robotController->pitchRotation(true, robot->getServoLinkTwo());
    if (pitchDownTwo) robotController->pitchRotation(false, robot->getServoLinkTwo());
}

void handleUserInputAdvanced(const String& command) {
    if (command.length() > 1) {
        Serial.println("Processing angles: " + command);
        splitStringToInt(command, servoIndexValues, 3);
        for (int i = 0; i < 3; i++) {
            robot->moveServoToAngle(i, servoIndexValues[i]);
        }
    } else {
        switch (command.charAt(0)) {
            case 'z':
                menuActive = false;
                break;
            case 'c':
                robotController->openEndEffector();
                break;
            case 'n':
                robotController->closeEndEffector();
                break;
            default:
                Serial.println("Unknown command.");
        }
    }
}

void manageContinuousControlsAdvanced() {
    String command = getSerialCommand();
}
