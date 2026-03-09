/**
 * @file ESP32_FieldCentric.ino
 * @brief ESP32 Field-Centric Mecanum Drive Example
 * 
 * This example demonstrates:
 * - Field-centric (joystick-style) control
 * - Smooth acceleration ramping
 * - Vector-based mecanum drive
 * - Simulated joystick input (can be replaced with real analog inputs)
 * 
 * Hardware:
 * - ESP32 DevKit
 * - 2x L298N Motor Drivers (default pins)
 * - 4x Mecanum wheel motors
 * - Optional: 2x Analog joysticks on A0, A1, A2
 * - Optional: IMU/Gyroscope for heading (simulated here)
 * 
 * @author Beastbroak30
 * @version 1.2.0
 */

#include <mecmotor.h>

// Create robot with default ESP32 L298N pins
Mecmotor robot;

// Joystick pins (if using real joysticks)
const int JOY_X_PIN = 34;   // Strafe left/right
const int JOY_Y_PIN = 35;   // Forward/backward  
const int JOY_ROT_PIN = 32; // Rotation

// Simulated robot heading (replace with IMU reading in real application)
float robotHeading = 0.0;

// Demo mode toggle
bool useFieldCentric = true;
bool useSimulatedInput = true;  // Set false if using real joysticks

// Timing
unsigned long lastUpdate = 0;
const unsigned long UPDATE_INTERVAL = 20;  // 50Hz update rate

void setup() {
    Serial.begin(115200);
    delay(1000);
    
    Serial.println("=================================");
    Serial.println("Mecmotor Field-Centric Demo v1.2.0");
    Serial.println("=================================");
    Serial.println();
    
    // Initialize the robot
    robot.begin();
    
    // Configure settings
    robot.setMaxSpeed(200);       // Cap speed at 200 (out of 255)
    robot.setDeadzone(20);        // Ignore small joystick movements
    robot.setRamping(true, 8);    // Enable smooth ramping
    robot.setStopMode(StopMode::BRAKE);  // Use active braking
    
    // If any motors spin wrong direction, fix here:
    // robot.setInversion(false, true, false, true);
    
    Serial.println("Robot initialized!");
    Serial.println("Commands via Serial:");
    Serial.println("  'f' - Toggle field-centric mode");
    Serial.println("  's' - Stop motors");
    Serial.println("  'r' - Reset heading to 0");
    Serial.println();
    
    // Setup joystick pins if using real hardware
    if (!useSimulatedInput) {
        pinMode(JOY_X_PIN, INPUT);
        pinMode(JOY_Y_PIN, INPUT);
        pinMode(JOY_ROT_PIN, INPUT);
    }
}

void loop() {
    // Handle serial commands
    handleSerialCommands();
    
    // Update at fixed interval
    if (millis() - lastUpdate >= UPDATE_INTERVAL) {
        lastUpdate = millis();
        
        // Get joystick input (normalized -1.0 to 1.0)
        float vx, vy, omega;
        
        if (useSimulatedInput) {
            // Simulated joystick pattern for demo
            getSimulatedInput(vx, vy, omega);
        } else {
            // Real joystick reading
            getRealJoystickInput(vx, vy, omega);
        }
        
        // Simulate heading change from rotation (in real app, read from IMU)
        robotHeading += omega * 2.0;  // Simulate rotation
        if (robotHeading > 360) robotHeading -= 360;
        if (robotHeading < 0) robotHeading += 360;
        
        // Apply drive mode
        if (useFieldCentric) {
            // Field-centric: joystick directions stay consistent with field
            robot.driveFieldCentric(vx, vy, omega, robotHeading);
        } else {
            // Robot-centric: joystick directions relative to robot front
            robot.drive(vx, vy, omega);
        }
        
        // Apply ramping (smooth acceleration)
        robot.update();
        
        // Debug output every 500ms
        static unsigned long lastPrint = 0;
        if (millis() - lastPrint >= 500) {
            lastPrint = millis();
            printStatus(vx, vy, omega);
        }
    }
}

/**
 * @brief Simulate joystick input with a demo pattern
 */
void getSimulatedInput(float& vx, float& vy, float& omega) {
    // Demo pattern: cycle through different movements
    unsigned long t = millis() / 2000;  // Change every 2 seconds
    int phase = t % 8;
    
    vx = 0;
    vy = 0;
    omega = 0;
    
    switch (phase) {
        case 0:  // Forward
            vx = 0.7;
            Serial.println("[Demo] Forward");
            break;
        case 1:  // Strafe right
            vy = 0.7;
            Serial.println("[Demo] Strafe Right");
            break;
        case 2:  // Backward
            vx = -0.7;
            Serial.println("[Demo] Backward");
            break;
        case 3:  // Strafe left
            vy = -0.7;
            Serial.println("[Demo] Strafe Left");
            break;
        case 4:  // Diagonal forward-right
            vx = 0.5;
            vy = 0.5;
            Serial.println("[Demo] Diagonal FR");
            break;
        case 5:  // Rotate CW
            omega = 0.6;
            Serial.println("[Demo] Rotate CW");
            break;
        case 6:  // Forward while rotating
            vx = 0.5;
            omega = 0.3;
            Serial.println("[Demo] Forward + Rotate");
            break;
        case 7:  // Stop
            Serial.println("[Demo] Stop");
            break;
    }
}

/**
 * @brief Read real analog joystick values
 */
void getRealJoystickInput(float& vx, float& vy, float& omega) {
    // Read analog values (0-4095 on ESP32)
    int rawX = analogRead(JOY_X_PIN);
    int rawY = analogRead(JOY_Y_PIN);
    int rawRot = analogRead(JOY_ROT_PIN);
    
    // Convert to -1.0 to 1.0 range
    // Center is ~2048, range is 0-4095
    vx = (rawY - 2048) / 2048.0;
    vy = (rawX - 2048) / 2048.0;
    omega = (rawRot - 2048) / 2048.0;
    
    // Apply deadzone (small movements ignored by library, but 
    // we can also apply here for cleaner input)
    if (abs(vx) < 0.1) vx = 0;
    if (abs(vy) < 0.1) vy = 0;
    if (abs(omega) < 0.1) omega = 0;
}

/**
 * @brief Handle serial commands
 */
void handleSerialCommands() {
    if (Serial.available()) {
        char cmd = Serial.read();
        
        switch (cmd) {
            case 'f':
            case 'F':
                useFieldCentric = !useFieldCentric;
                Serial.print("Field-centric mode: ");
                Serial.println(useFieldCentric ? "ON" : "OFF");
                break;
                
            case 's':
            case 'S':
                robot.stop();
                Serial.println("Motors stopped");
                break;
                
            case 'r':
            case 'R':
                robotHeading = 0;
                Serial.println("Heading reset to 0");
                break;
                
            case 'b':
            case 'B':
                robot.brake();
                Serial.println("Brake applied");
                break;
        }
    }
}

/**
 * @brief Print current status
 */
void printStatus(float vx, float vy, float omega) {
    Serial.print("Mode: ");
    Serial.print(useFieldCentric ? "Field-Centric" : "Robot-Centric");
    Serial.print(" | Heading: ");
    Serial.print(robotHeading, 1);
    Serial.print("° | Input: (");
    Serial.print(vx, 2);
    Serial.print(", ");
    Serial.print(vy, 2);
    Serial.print(", ");
    Serial.print(omega, 2);
    Serial.print(") | Motors: FL=");
    Serial.print(robot.getMotorSpeed(Motor::FL));
    Serial.print(" FR=");
    Serial.print(robot.getMotorSpeed(Motor::FR));
    Serial.print(" BL=");
    Serial.print(robot.getMotorSpeed(Motor::BL));
    Serial.print(" BR=");
    Serial.println(robot.getMotorSpeed(Motor::BR));
}
