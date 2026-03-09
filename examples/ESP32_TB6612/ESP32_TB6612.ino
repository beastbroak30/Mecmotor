/**
 * @file ESP32_TB6612.ino
 * @brief ESP32 Mecanum Robot with TB6612FNG Motor Drivers
 * 
 * This example demonstrates using Mecmotor with TB6612FNG motor drivers,
 * which are more efficient than L298N and better suited for battery-powered robots.
 * 
 * TB6612FNG Advantages over L298N:
 * - Higher efficiency (MOSFET vs BJT)
 * - Lower voltage drop (~0.5V vs ~2V)
 * - Smaller form factor
 * - Built-in standby mode for power saving
 * - Better for 3.3V logic (ESP32 compatible)
 * 
 * Hardware:
 * - ESP32 DevKit
 * - 2x TB6612FNG Motor Drivers (or 4x single-channel)
 * - 4x Mecanum wheel motors (6V or 12V)
 * - External power supply (battery)
 * 
 * TB6612FNG Pinout per driver:
 *   - VM: Motor voltage (up to 15V)
 *   - VCC: Logic voltage (3.3V from ESP32)
 *   - GND: Ground
 *   - STBY: Standby (HIGH to enable)
 *   - AIN1, AIN2: Motor A direction
 *   - PWMA: Motor A speed (PWM)
 *   - BIN1, BIN2: Motor B direction
 *   - PWMB: Motor B speed (PWM)
 * 
 * @author Beastbroak30
 * @version 1.2.0
 */

#include <mecmotor.h>

// ============================================================================
// Pin Definitions for TB6612FNG
// ============================================================================

// Driver 1 - Front motors (FL and FR)
#define STBY_1      25      // Standby pin for driver 1

#define FL_AIN1     18      // Front Left IN1
#define FL_AIN2     19      // Front Left IN2
#define FL_PWMA     21      // Front Left PWM

#define FR_BIN1     5       // Front Right IN1
#define FR_BIN2     23      // Front Right IN2
#define FR_PWMB     22      // Front Right PWM

// Driver 2 - Back motors (BL and BR)
#define STBY_2      26      // Standby pin for driver 2 (can connect to STBY_1)

#define BL_AIN1     13      // Back Left IN1
#define BL_AIN2     12      // Back Left IN2
#define BL_PWMA     14      // Back Left PWM

#define BR_BIN1     27      // Back Right IN1
#define BR_BIN2     33      // Back Right IN2
#define BR_PWMB     32      // Back Right PWM

// ============================================================================
// Robot Instance
// ============================================================================

// Create Mecmotor with TB6612 driver type
// Note: We'll manage STBY pins separately for flexibility
Mecmotor robot(
    DriverType::TB6612,
    FL_AIN1, FL_AIN2, FL_PWMA,   // Front Left
    FR_BIN1, FR_BIN2, FR_PWMB,   // Front Right
    BL_AIN1, BL_AIN2, BL_PWMA,   // Back Left
    BR_BIN1, BR_BIN2, BR_PWMB,   // Back Right
    STBY_1                        // Standby pin (will be set HIGH)
);

// ============================================================================
// Configuration
// ============================================================================

// Joystick simulation (set to actual analog pins if using real joystick)
bool useRealJoystick = false;
const int JOY_X_PIN = 34;
const int JOY_Y_PIN = 35;
const int JOY_ROT_PIN = 36;

// Timing
unsigned long lastUpdate = 0;
const int UPDATE_RATE_MS = 20;

void setup() {
    Serial.begin(115200);
    delay(500);
    
    Serial.println(F("====================================="));
    Serial.println(F("Mecmotor TB6612FNG Example v1.2.0"));
    Serial.println(F("====================================="));
    Serial.println();
    
    // Setup second standby pin if using two drivers
    pinMode(STBY_2, OUTPUT);
    digitalWrite(STBY_2, HIGH);  // Enable driver 2
    
    // Initialize robot (this also enables STBY_1)
    robot.begin();
    
    // Configure for smooth operation
    robot.setMaxSpeed(230);       // TB6612 can handle full speed better than L298N
    robot.setDeadzone(15);        // Lower deadzone for more responsive control
    robot.setRamping(true, 12);   // Smooth acceleration
    robot.setStopMode(StopMode::BRAKE);  // TB6612 supports short-brake mode
    
    // Fix motor directions if needed
    // TB6612 motor direction depends on wiring, adjust here if needed
    // robot.setInversion(false, true, false, true);
    
    Serial.println(F("TB6612FNG drivers initialized!"));
    Serial.println();
    Serial.println(F("Serial Commands:"));
    Serial.println(F("  w/s/a/d - Forward/Back/Left/Right"));
    Serial.println(F("  q/e     - Strafe Left/Right"));
    Serial.println(F("  z/c     - Rotate CCW/CW"));
    Serial.println(F("  x       - Stop"));
    Serial.println(F("  p       - Toggle power save (standby)"));
    Serial.println(F("  1-5     - Set speed preset"));
    Serial.println(F("  v       - Vector drive demo"));
    Serial.println();
}

void loop() {
    // Handle serial commands
    if (Serial.available()) {
        handleSerialCommand(Serial.read());
    }
    
    // Update ramping at fixed interval
    if (millis() - lastUpdate >= UPDATE_RATE_MS) {
        lastUpdate = millis();
        
        if (useRealJoystick) {
            handleJoystickInput();
        }
        
        robot.update();  // Apply ramping
    }
}

// ============================================================================
// Command Handler
// ============================================================================

void handleSerialCommand(char cmd) {
    static bool standbyMode = false;
    static int speedPreset = 180;
    
    switch (cmd) {
        // Movement commands
        case 'w':
        case 'W':
            Serial.println(F("Forward"));
            robot.forward(speedPreset);
            break;
            
        case 's':
        case 'S':
            Serial.println(F("Backward"));
            robot.backward(speedPreset);
            break;
            
        case 'a':
        case 'A':
            Serial.println(F("Turn Left (CCW)"));
            robot.left(speedPreset);
            break;
            
        case 'd':
        case 'D':
            Serial.println(F("Turn Right (CW)"));
            robot.right(speedPreset);
            break;
            
        case 'q':
        case 'Q':
            Serial.println(F("Strafe Left"));
            robot.strafeLeft(speedPreset);
            break;
            
        case 'e':
        case 'E':
            Serial.println(F("Strafe Right"));
            robot.strafeRight(speedPreset);
            break;
            
        case 'z':
        case 'Z':
            Serial.println(F("Rotate CCW"));
            robot.left(speedPreset / 2);
            break;
            
        case 'c':
        case 'C':
            Serial.println(F("Rotate CW"));
            robot.right(speedPreset / 2);
            break;
            
        case 'x':
        case 'X':
            Serial.println(F("STOP"));
            robot.stop();
            break;
            
        case 'b':
        case 'B':
            Serial.println(F("BRAKE"));
            robot.brake();
            break;
            
        // Speed presets
        case '1':
            speedPreset = 80;
            Serial.println(F("Speed: Slow (80)"));
            break;
        case '2':
            speedPreset = 120;
            Serial.println(F("Speed: Medium-Low (120)"));
            break;
        case '3':
            speedPreset = 160;
            Serial.println(F("Speed: Medium (160)"));
            break;
        case '4':
            speedPreset = 200;
            Serial.println(F("Speed: Medium-High (200)"));
            break;
        case '5':
            speedPreset = 240;
            Serial.println(F("Speed: Fast (240)"));
            break;
            
        // Power save (standby mode)
        case 'p':
        case 'P':
            standbyMode = !standbyMode;
            digitalWrite(STBY_1, standbyMode ? LOW : HIGH);
            digitalWrite(STBY_2, standbyMode ? LOW : HIGH);
            Serial.print(F("Standby mode: "));
            Serial.println(standbyMode ? F("ON (power save)") : F("OFF (active)"));
            break;
            
        // Vector drive demo
        case 'v':
        case 'V':
            vectorDriveDemo();
            break;
            
        // Diagonal movements
        case '7':
            Serial.println(F("Diagonal Front-Left"));
            robot.diagonalFL(speedPreset);
            break;
        case '9':
            Serial.println(F("Diagonal Front-Right"));
            robot.diagonalFR(speedPreset);
            break;
        case '1' + 128:  // Numpad 1 (if terminal supports)
            Serial.println(F("Diagonal Back-Left"));
            robot.diagonalBL(speedPreset);
            break;
        case '3' + 128:  // Numpad 3
            Serial.println(F("Diagonal Back-Right"));
            robot.diagonalBR(speedPreset);
            break;
            
        // Info
        case 'i':
        case 'I':
            printInfo();
            break;
    }
}

// ============================================================================
// Joystick Handler
// ============================================================================

void handleJoystickInput() {
    // Read analog joystick (ESP32: 0-4095)
    int rawX = analogRead(JOY_X_PIN);
    int rawY = analogRead(JOY_Y_PIN);
    int rawRot = analogRead(JOY_ROT_PIN);
    
    // Convert to -1.0 to 1.0
    float vx = (rawY - 2048) / 2048.0;
    float vy = (rawX - 2048) / 2048.0;
    float omega = (rawRot - 2048) / 2048.0;
    
    // Apply small deadzone
    if (abs(vx) < 0.08) vx = 0;
    if (abs(vy) < 0.08) vy = 0;
    if (abs(omega) < 0.08) omega = 0;
    
    // Drive with vector input
    robot.drive(vx, vy, omega);
}

// ============================================================================
// Vector Drive Demo
// ============================================================================

void vectorDriveDemo() {
    Serial.println(F("=== Vector Drive Demo ==="));
    
    Serial.println(F("1. Pure forward (vx=1, vy=0, omega=0)"));
    robot.drive(1.0, 0, 0);
    delay(1500);
    
    Serial.println(F("2. Pure strafe right (vx=0, vy=1, omega=0)"));
    robot.drive(0, 1.0, 0);
    delay(1500);
    
    Serial.println(F("3. Forward-right diagonal (vx=0.7, vy=0.7, omega=0)"));
    robot.drive(0.7, 0.7, 0);
    delay(1500);
    
    Serial.println(F("4. Forward while rotating (vx=0.6, vy=0, omega=0.4)"));
    robot.drive(0.6, 0, 0.4);
    delay(2000);
    
    Serial.println(F("5. Strafe while rotating (vx=0, vy=0.5, omega=0.5)"));
    robot.drive(0, 0.5, 0.5);
    delay(2000);
    
    Serial.println(F("6. Full omni movement (vx=0.4, vy=0.4, omega=0.3)"));
    robot.drive(0.4, 0.4, 0.3);
    delay(2000);
    
    Serial.println(F("7. Polar: 30° angle at 80% speed"));
    robot.driveAngle(30, 0.8, 0);
    delay(1500);
    
    Serial.println(F("8. Polar: 135° angle (back-left) at 70% speed"));
    robot.driveAngle(135, 0.7, 0);
    delay(1500);
    
    robot.stop();
    Serial.println(F("=== Demo Complete ==="));
    Serial.println();
}

// ============================================================================
// Info Display
// ============================================================================

void printInfo() {
    Serial.println(F("\n=== Robot Status ==="));
    Serial.print(F("Library Version: "));
    Serial.println(Mecmotor::getVersion());
    Serial.print(F("Initialized: "));
    Serial.println(robot.isInitialized() ? F("Yes") : F("No"));
    Serial.print(F("Ramping: "));
    Serial.println(robot.isRamping() ? F("Active") : F("Idle"));
    
    Serial.println(F("\nMotor Speeds:"));
    Serial.print(F("  FL: ")); Serial.println(robot.getMotorSpeed(Motor::FL));
    Serial.print(F("  FR: ")); Serial.println(robot.getMotorSpeed(Motor::FR));
    Serial.print(F("  BL: ")); Serial.println(robot.getMotorSpeed(Motor::BL));
    Serial.print(F("  BR: ")); Serial.println(robot.getMotorSpeed(Motor::BR));
    
    Serial.print(F("\nFree Heap: "));
    Serial.print(ESP.getFreeHeap());
    Serial.println(F(" bytes"));
    Serial.println();
}
