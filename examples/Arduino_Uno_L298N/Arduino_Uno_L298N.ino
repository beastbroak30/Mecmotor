/**
 * @file Arduino_Uno_L298N.ino
 * @brief Arduino Uno/Mega Mecanum Robot Example with L298N
 * 
 * This example demonstrates using Mecmotor on Arduino Uno or Mega
 * with standard analogWrite() PWM (no ESP32 LEDC).
 * 
 * Hardware:
 * - Arduino Uno or Mega
 * - 2x L298N Motor Drivers
 * - 4x Mecanum wheel motors
 * - External power supply for motors (7-12V)
 * 
 * Wiring (Arduino Uno - limited PWM pins!):
 *   Motor FL: D2 (IN1), D4 (IN2), D3 (ENA - PWM)
 *   Motor FR: D7 (IN3), D8 (IN4), D5 (ENB - PWM)
 *   Motor BL: D12 (IN1), D13 (IN2), D6 (ENA - PWM)
 *   Motor BR: A0 (IN3), A1 (IN4), D9 (ENB - PWM)
 * 
 * Note: Arduino Uno has only 6 PWM pins (3, 5, 6, 9, 10, 11)
 *       We use 4 of them for motor speed control.
 * 
 * @author Beastbroak30
 * @version 1.2.0
 */

#include <mecmotor.h>

// Define pins for Arduino Uno/Mega
// PWM pins on Uno: 3, 5, 6, 9, 10, 11
// We need 4 PWM pins for EN/speed control

// Motor pin definitions
#define FL_IN1  2
#define FL_IN2  4
#define FL_EN   3   // PWM

#define FR_IN1  7
#define FR_IN2  8
#define FR_EN   5   // PWM

#define BL_IN1  12
#define BL_IN2  13
#define BL_EN   6   // PWM

#define BR_IN1  A0  // Using analog pins as digital
#define BR_IN2  A1
#define BR_EN   9   // PWM

// Create Mecmotor with custom pins
Mecmotor robot(
    FL_IN1, FL_IN2, FL_EN,   // Front Left
    FR_IN1, FR_IN2, FR_EN,   // Front Right
    BL_IN1, BL_IN2, BL_EN,   // Back Left
    BR_IN1, BR_IN2, BR_EN    // Back Right
);

// Demo state
int demoStep = 0;
unsigned long lastStepTime = 0;
const unsigned long STEP_DURATION = 2000;  // 2 seconds per step

void setup() {
    Serial.begin(9600);  // Uno uses 9600 typically
    
    Serial.println(F("==========================="));
    Serial.println(F("Mecmotor Arduino Uno Demo"));
    Serial.println(F("==========================="));
    Serial.println();
    
    // Initialize the robot
    robot.begin();
    
    // Configuration
    robot.setMaxSpeed(200);   // Limit speed (good for testing)
    robot.setDeadzone(15);    // Ignore very low PWM
    
    // If motors spin wrong direction, uncomment and adjust:
    // robot.setInversion(false, true, false, true);
    
    Serial.println(F("Initialization complete!"));
    Serial.println(F("Running demo pattern..."));
    Serial.println(F("Send 's' to stop, 'r' to restart demo"));
    Serial.println();
    
    delay(1000);
}

void loop() {
    // Handle serial input
    if (Serial.available()) {
        char cmd = Serial.read();
        handleCommand(cmd);
    }
    
    // Run demo pattern
    if (millis() - lastStepTime >= STEP_DURATION) {
        lastStepTime = millis();
        runDemoStep();
    }
}

void handleCommand(char cmd) {
    switch (cmd) {
        case 's':
        case 'S':
            robot.stop();
            demoStep = -1;  // Pause demo
            Serial.println(F("STOPPED"));
            break;
            
        case 'r':
        case 'R':
            demoStep = 0;
            Serial.println(F("Restarting demo..."));
            break;
            
        case 'f':
            robot.forward(180);
            Serial.println(F("Forward"));
            demoStep = -1;
            break;
            
        case 'b':
            robot.backward(180);
            Serial.println(F("Backward"));
            demoStep = -1;
            break;
            
        case 'l':
            robot.strafeLeft(180);
            Serial.println(F("Strafe Left"));
            demoStep = -1;
            break;
            
        case 'p':  // Print motor speeds
            printMotorSpeeds();
            break;
    }
}

void runDemoStep() {
    if (demoStep < 0) return;  // Demo paused
    
    switch (demoStep) {
        case 0:
            Serial.println(F("Step 1: Forward"));
            robot.forward(180);
            break;
            
        case 1:
            Serial.println(F("Step 2: Backward"));
            robot.backward(180);
            break;
            
        case 2:
            Serial.println(F("Step 3: Strafe Right"));
            robot.strafeRight(180);
            break;
            
        case 3:
            Serial.println(F("Step 4: Strafe Left"));
            robot.strafeLeft(180);
            break;
            
        case 4:
            Serial.println(F("Step 5: Rotate Right (CW)"));
            robot.right(150);
            break;
            
        case 5:
            Serial.println(F("Step 6: Rotate Left (CCW)"));
            robot.left(150);
            break;
            
        case 6:
            Serial.println(F("Step 7: Diagonal Front-Right"));
            robot.diagonalFR(180);
            break;
            
        case 7:
            Serial.println(F("Step 8: Diagonal Back-Left"));
            robot.diagonalBL(180);
            break;
            
        case 8:
            Serial.println(F("Step 9: Vector Drive (forward + strafe + rotate)"));
            // Use vector drive: 50% forward, 30% right strafe, 20% rotation
            robot.drive(0.5, 0.3, 0.2);
            break;
            
        case 9:
            Serial.println(F("Step 10: Polar Drive (45 degrees)"));
            // Move at 45 degree angle (forward-right diagonal)
            robot.driveAngle(45, 0.7, 0);
            break;
            
        case 10:
            Serial.println(F("Step 11: Stop with BRAKE"));
            robot.brake();
            break;
            
        case 11:
            Serial.println(F("Step 12: Pivot Front-Right"));
            robot.pivotfr(150);
            break;
            
        default:
            Serial.println(F("--- Demo Complete, Restarting ---"));
            Serial.println();
            robot.stop();
            demoStep = -1;  // Will reset to 0 next iteration
            delay(2000);
            demoStep = 0;
            return;
    }
    
    printMotorSpeeds();
    demoStep++;
}

void printMotorSpeeds() {
    Serial.print(F("  Motors: FL="));
    Serial.print(robot.getMotorSpeed(Motor::FL));
    Serial.print(F(" FR="));
    Serial.print(robot.getMotorSpeed(Motor::FR));
    Serial.print(F(" BL="));
    Serial.print(robot.getMotorSpeed(Motor::BL));
    Serial.print(F(" BR="));
    Serial.println(robot.getMotorSpeed(Motor::BR));
}
