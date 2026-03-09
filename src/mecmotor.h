/**
 * @file mecmotor.h
 * @brief Mecmotor - Advanced Mecanum Wheel Robot Control Library
 * @version 1.2.0
 * @author Beastbroak30 (Antarip Kar)
 * @license MIT
 * 
 * Features:
 * - Multi-driver support: L298N, TB6612FNG, DRV8833, Cytron MD10C/MDD10A
 * - Field-centric (joystick-style) and polar drive modes
 * - Acceleration ramping and deadzone filtering
 * - Motor inversion for easy wheel direction correction
 * - Brake vs coast stop modes
 * - Non-blocking operation with update() method
 * - ESP32 LEDC PWM support with fallback for other MCUs
 * - RTOS-safe (no blocking delays)
 * 
 * GitHub: https://github.com/beastbroak30/Mecmotor
 */

#ifndef MECMOTOR_H
#define MECMOTOR_H

#include <Arduino.h>
#include <math.h>

// ============================================================================
// Configuration Defines (can be defined before including this header)
// ============================================================================

// Uncomment or define before including to enable debug output
// #define MEC_DEBUG

// PWM frequency for ESP32 (default 5000 Hz)
#ifndef MEC_PWM_FREQ
#define MEC_PWM_FREQ 5000
#endif

// PWM resolution for ESP32 (default 8-bit = 0-255)
#ifndef MEC_PWM_RESOLUTION
#define MEC_PWM_RESOLUTION 8
#endif

// Default deadzone threshold (PWM values below this are ignored)
#ifndef MEC_DEFAULT_DEADZONE
#define MEC_DEFAULT_DEADZONE 20
#endif

// Default acceleration rate (PWM units per update cycle)
#ifndef MEC_DEFAULT_ACCEL_RATE
#define MEC_DEFAULT_ACCEL_RATE 10
#endif

// ============================================================================
// Debug Macros
// ============================================================================

#ifdef MEC_DEBUG
  #define MEC_LOG(x)     Serial.print(x)
  #define MEC_LOGLN(x)   Serial.println(x)
  #define MEC_LOGF(...)  Serial.printf(__VA_ARGS__)
#else
  #define MEC_LOG(x)
  #define MEC_LOGLN(x)
  #define MEC_LOGF(...)
#endif

// ============================================================================
// Enums and Types
// ============================================================================

/**
 * @brief Supported motor driver types
 */
enum class DriverType : uint8_t {
    L298N,      ///< L298N: IN1, IN2 for direction, EN for PWM
    TB6612,     ///< TB6612FNG: IN1, IN2 for direction, PWM pin, STBY required
    DRV8833,    ///< DRV8833: IN1, IN2 both PWM capable (no separate EN)
    CYTRON_MD   ///< Cytron MD10C/MDD10A: DIR + PWM pins only
};

/**
 * @brief Stop mode for motors
 */
enum class StopMode : uint8_t {
    COAST,  ///< Coast to stop (both inputs LOW)
    BRAKE   ///< Active brake (both inputs HIGH for L298N/TB6612)
};

/**
 * @brief Motor identification enum
 */
enum class Motor : uint8_t {
    FL = 0,  ///< Front Left
    FR = 1,  ///< Front Right  
    BL = 2,  ///< Back Left
    BR = 3   ///< Back Right
};

// ============================================================================
// Pin Configuration Structures
// ============================================================================

/**
 * @brief Pin configuration for L298N/TB6612 style drivers (IN1, IN2, EN/PWM)
 */
struct MotorPinsL298N {
    int8_t in1;     ///< Direction pin 1
    int8_t in2;     ///< Direction pin 2
    int8_t en;      ///< Enable/PWM pin
};

/**
 * @brief Pin configuration for Cytron style drivers (DIR, PWM)
 */
struct MotorPinsCytron {
    int8_t dir;     ///< Direction pin
    int8_t pwm;     ///< PWM speed pin
};

/**
 * @brief Pin configuration for DRV8833 style drivers (both pins can be PWM)
 */
struct MotorPinsDRV8833 {
    int8_t in1;     ///< PWM input 1
    int8_t in2;     ///< PWM input 2
};

/**
 * @brief Complete motor configuration for all 4 motors
 */
struct MecMotorConfig {
    DriverType driverType;
    
    // Motor inversion flags (true = flip direction)
    bool invertFL;
    bool invertFR;
    bool invertBL;
    bool invertBR;
    
    // Optional standby pin for TB6612 (-1 if not used)
    int8_t stbyPin;
    
    // Union for different driver pin configurations
    union {
        MotorPinsL298N l298n[4];    // For L298N, TB6612
        MotorPinsCytron cytron[4];  // For Cytron MD
        MotorPinsDRV8833 drv8833[4]; // For DRV8833
    } pins;
};

// ============================================================================
// Main Mecmotor Class
// ============================================================================

/**
 * @brief Main class for controlling a 4-wheel Mecanum robot
 * 
 * Supports multiple motor driver types and provides both simple
 * direction-based control and advanced vector/field-centric control.
 */
class Mecmotor {
public:
    // ========================================================================
    // Constructors
    // ========================================================================
    
    /**
     * @brief Default constructor with ESP32-friendly L298N pins
     */
    Mecmotor();
    
    /**
     * @brief Constructor with custom L298N pins (backward compatible)
     * @param in1 FL motor IN1
     * @param in2 FL motor IN2  
     * @param ena FL motor EN/PWM
     * @param in3 FR motor IN1
     * @param in4 FR motor IN2
     * @param enb FR motor EN/PWM
     * @param in1_2 BL motor IN1
     * @param in2_2 BL motor IN2
     * @param ena_2 BL motor EN/PWM
     * @param in3_2 BR motor IN1
     * @param in4_2 BR motor IN2
     * @param enb_2 BR motor EN/PWM
     */
    Mecmotor(int in1, int in2, int ena,
             int in3, int in4, int enb,
             int in1_2, int in2_2, int ena_2,
             int in3_2, int in4_2, int enb_2);
    
    /**
     * @brief Full configuration constructor
     * @param config Complete motor configuration struct
     */
    Mecmotor(const MecMotorConfig& config);
    
    /**
     * @brief Constructor for Cytron MD/MDD drivers (DIR+PWM style)
     * @param driverType Must be DriverType::CYTRON_MD
     * @param dirFL Direction pin for front left
     * @param pwmFL PWM pin for front left
     * @param dirFR Direction pin for front right
     * @param pwmFR PWM pin for front right
     * @param dirBL Direction pin for back left
     * @param pwmBL PWM pin for back left
     * @param dirBR Direction pin for back right
     * @param pwmBR PWM pin for back right
     */
    Mecmotor(DriverType driverType,
             int dirFL, int pwmFL,
             int dirFR, int pwmFR,
             int dirBL, int pwmBL,
             int dirBR, int pwmBR);
    
    /**
     * @brief Constructor for TB6612FNG with standby pin
     * @param driverType Must be DriverType::TB6612
     * @param in1FL to enBR pins for all 4 motors
     * @param stbyPin Standby pin (pulled HIGH to enable)
     */
    Mecmotor(DriverType driverType,
             int in1FL, int in2FL, int pwmFL,
             int in1FR, int in2FR, int pwmFR,
             int in1BL, int in2BL, int pwmBL,
             int in1BR, int in2BR, int pwmBR,
             int stbyPin = -1);
    
    // ========================================================================
    // Initialization
    // ========================================================================
    
    /**
     * @brief Initialize pins and PWM channels
     * @note Call this in setup() before using motor controls
     */
    void begin();
    
    /**
     * @brief Set motor inversion flags
     * @param fl Invert front left
     * @param fr Invert front right
     * @param bl Invert back left
     * @param br Invert back right
     */
    void setInversion(bool fl, bool fr, bool bl, bool br);
    
    /**
     * @brief Set maximum speed cap
     * @param maxSpeed Maximum PWM value (0-255)
     */
    void setMaxSpeed(uint8_t maxSpeed);
    
    /**
     * @brief Set deadzone threshold
     * @param deadzone PWM values below this are treated as 0
     */
    void setDeadzone(uint8_t deadzone);
    
    /**
     * @brief Enable/disable ramping and set acceleration rate
     * @param enable Enable ramping
     * @param rate PWM units to change per update() call (default 10)
     */
    void setRamping(bool enable, uint8_t rate = MEC_DEFAULT_ACCEL_RATE);
    
    /**
     * @brief Set stop mode (coast or brake)
     * @param mode StopMode::COAST or StopMode::BRAKE
     */
    void setStopMode(StopMode mode);
    
    // ========================================================================
    // Basic Direction Controls (backward compatible)
    // ========================================================================
    
    /**
     * @brief Move forward at given speed
     * @param speed PWM value 0-255
     */
    void forward(int speed);
    
    /**
     * @brief Move backward at given speed
     * @param speed PWM value 0-255
     */
    void backward(int speed);
    
    /**
     * @brief Turn left (rotate CCW) at given speed
     * @param speed PWM value 0-255
     */
    void left(int speed);
    
    /**
     * @brief Turn right (rotate CW) at given speed
     * @param speed PWM value 0-255
     */
    void right(int speed);
    
    /**
     * @brief Strafe right at given speed
     * @param speed PWM value 0-255
     */
    void strafeRight(int speed);
    void strafer(int speed);  // Alias for backward compatibility
    
    /**
     * @brief Strafe left at given speed
     * @param speed PWM value 0-255
     */
    void strafeLeft(int speed);
    void strafel(int speed);  // Alias for backward compatibility
    
    /**
     * @brief Diagonal front-right movement
     * @param speed PWM value 0-255
     */
    void diagonalFR(int speed);
    
    /**
     * @brief Diagonal front-left movement
     * @param speed PWM value 0-255
     */
    void diagonalFL(int speed);
    
    /**
     * @brief Diagonal back-right movement
     * @param speed PWM value 0-255
     */
    void diagonalBR(int speed);
    
    /**
     * @brief Diagonal back-left movement
     * @param speed PWM value 0-255
     */
    void diagonalBL(int speed);
    
    // Pivot controls (backward compatible)
    void pivotfr(int speed);
    void pivotfl(int speed);
    void pivotbr(int speed);
    void pivotbl(int speed);
    
    /**
     * @brief Stop all motors using current stop mode
     */
    void stop();
    
    /**
     * @brief Stop all motors with brake (active stop)
     */
    void brake();
    
    /**
     * @brief Stop all motors with coast (passive stop)
     */
    void coast();
    
    // ========================================================================
    // Advanced Vector/Field-Centric Controls
    // ========================================================================
    
    /**
     * @brief Drive with joystick-style vector input
     * 
     * Uses inverse kinematics to calculate individual wheel speeds.
     * All inputs are normalized -1.0 to 1.0.
     * 
     * @param vx Forward/backward velocity (-1.0 = full backward, 1.0 = full forward)
     * @param vy Strafe velocity (-1.0 = full left, 1.0 = full right)
     * @param omega Rotation velocity (-1.0 = full CCW, 1.0 = full CW)
     * @param maxSpeed Maximum PWM output (0-255, default 255)
     */
    void drive(float vx, float vy, float omega, uint8_t maxSpeed = 255);
    
    /**
     * @brief Drive at a specific angle with optional rotation
     * 
     * @param angleDeg Direction angle in degrees (0=forward, 90=right, 180=back, 270=left)
     * @param speed Speed magnitude 0.0 to 1.0
     * @param omega Rotation velocity -1.0 to 1.0
     * @param maxSpeed Maximum PWM output (0-255, default 255)
     */
    void driveAngle(float angleDeg, float speed, float omega = 0, uint8_t maxSpeed = 255);
    
    /**
     * @brief Field-centric drive (requires heading input)
     * 
     * Transforms robot-centric inputs to field-centric based on robot heading.
     * Useful with IMU/gyroscope for consistent control regardless of robot orientation.
     * 
     * @param vx Forward velocity in field frame
     * @param vy Right velocity in field frame  
     * @param omega Rotation velocity
     * @param robotHeadingDeg Current robot heading in degrees (0 = field forward)
     * @param maxSpeed Maximum PWM output (0-255, default 255)
     */
    void driveFieldCentric(float vx, float vy, float omega, 
                           float robotHeadingDeg, uint8_t maxSpeed = 255);
    
    // ========================================================================
    // Individual Motor Control
    // ========================================================================
    
    /**
     * @brief Set individual motor speed and direction
     * @param motor Motor enum (FL, FR, BL, BR)
     * @param speed Signed speed -255 to 255 (negative = reverse)
     */
    void setMotorSpeed(Motor motor, int16_t speed);
    
    /**
     * @brief Set all four motor speeds directly
     * @param fl Front left speed -255 to 255
     * @param fr Front right speed -255 to 255
     * @param bl Back left speed -255 to 255
     * @param br Back right speed -255 to 255
     */
    void setMotorSpeeds(int16_t fl, int16_t fr, int16_t bl, int16_t br);
    
    // ========================================================================
    // Non-blocking / RTOS Support
    // ========================================================================
    
    /**
     * @brief Update motor outputs with ramping
     * 
     * Call this regularly in loop() when ramping is enabled.
     * Smoothly transitions motor speeds to target values.
     * 
     * @return true if motors are still ramping to target
     */
    bool update();
    
    /**
     * @brief Check if motors are currently ramping
     * @return true if any motor is still ramping to target speed
     */
    bool isRamping() const;
    
    /**
     * @brief Get current motor speed
     * @param motor Motor enum (FL, FR, BL, BR)
     * @return Current speed value -255 to 255
     */
    int16_t getMotorSpeed(Motor motor) const;
    
    /**
     * @brief Get target motor speed (when ramping)
     * @param motor Motor enum (FL, FR, BL, BR)
     * @return Target speed value -255 to 255
     */
    int16_t getTargetSpeed(Motor motor) const;
    
    // ========================================================================
    // Utility
    // ========================================================================
    
    /**
     * @brief Get library version string
     * @return Version string "1.2.0"
     */
    static const char* getVersion() { return "1.2.0"; }
    
    /**
     * @brief Check if library is initialized
     * @return true if begin() has been called
     */
    bool isInitialized() const { return _initialized; }

private:
    // Configuration
    MecMotorConfig _config;
    bool _initialized;
    uint8_t _maxSpeed;
    uint8_t _deadzone;
    StopMode _stopMode;
    
    // Ramping
    bool _rampingEnabled;
    uint8_t _rampRate;
    int16_t _currentSpeeds[4];
    int16_t _targetSpeeds[4];
    
    // ESP32 LEDC channels
    #if defined(ESP32)
    uint8_t _ledcChannels[4];
    static uint8_t _nextChannel;
    #endif
    
    // Private methods
    void _initPins();
    void _writePWM(uint8_t channel, uint8_t pin, uint8_t value);
    void _setMotorRaw(uint8_t motorIndex, int16_t speed);
    int16_t _applyDeadzone(int16_t speed);
    int16_t _clampSpeed(int16_t speed);
    void _calculateMecanumSpeeds(float vx, float vy, float omega, 
                                  int16_t* speeds, uint8_t maxSpeed);
};

// ============================================================================
// Backward Compatibility Alias
// ============================================================================

// Allow lowercase class name for backward compatibility
typedef Mecmotor mecmotor;

#endif // MECMOTOR_H
