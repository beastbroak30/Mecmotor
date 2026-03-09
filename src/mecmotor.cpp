/**
 * @file mecmotor.cpp
 * @brief Mecmotor - Advanced Mecanum Wheel Robot Control Library Implementation
 * @version 1.2.0
 * @author Beastbroak30 (Antarip Kar)
 * @license MIT
 */

#include "mecmotor.h"

// ============================================================================
// Static Variables
// ============================================================================

#if defined(ESP32)
uint8_t Mecmotor::_nextChannel = 0;
#endif

// ============================================================================
// Constructors
// ============================================================================

Mecmotor::Mecmotor() {
    // Default ESP32-friendly L298N pins
    _config.driverType = DriverType::L298N;
    _config.invertFL = false;
    _config.invertFR = false;
    _config.invertBL = false;
    _config.invertBR = false;
    _config.stbyPin = -1;
    
    // FL: GPIO18, GPIO19, GPIO21
    _config.pins.l298n[0] = {18, 19, 21};
    // FR: GPIO5, GPIO23, GPIO22
    _config.pins.l298n[1] = {5, 23, 22};
    // BL: GPIO13, GPIO12, GPIO14
    _config.pins.l298n[2] = {13, 12, 14};
    // BR: GPIO2, GPIO4, GPIO15
    _config.pins.l298n[3] = {2, 4, 15};
    
    _initDefaults();
}

Mecmotor::Mecmotor(int in1, int in2, int ena,
                   int in3, int in4, int enb,
                   int in1_2, int in2_2, int ena_2,
                   int in3_2, int in4_2, int enb_2) {
    _config.driverType = DriverType::L298N;
    _config.invertFL = false;
    _config.invertFR = false;
    _config.invertBL = false;
    _config.invertBR = false;
    _config.stbyPin = -1;
    
    _config.pins.l298n[0] = {(int8_t)in1, (int8_t)in2, (int8_t)ena};
    _config.pins.l298n[1] = {(int8_t)in3, (int8_t)in4, (int8_t)enb};
    _config.pins.l298n[2] = {(int8_t)in1_2, (int8_t)in2_2, (int8_t)ena_2};
    _config.pins.l298n[3] = {(int8_t)in3_2, (int8_t)in4_2, (int8_t)enb_2};
    
    _initDefaults();
}

Mecmotor::Mecmotor(const MecMotorConfig& config) {
    _config = config;
    _initDefaults();
}

Mecmotor::Mecmotor(DriverType driverType,
                   int dirFL, int pwmFL,
                   int dirFR, int pwmFR,
                   int dirBL, int pwmBL,
                   int dirBR, int pwmBR) {
    _config.driverType = driverType;
    _config.invertFL = false;
    _config.invertFR = false;
    _config.invertBL = false;
    _config.invertBR = false;
    _config.stbyPin = -1;
    
    if (driverType == DriverType::CYTRON_MD) {
        _config.pins.cytron[0] = {(int8_t)dirFL, (int8_t)pwmFL};
        _config.pins.cytron[1] = {(int8_t)dirFR, (int8_t)pwmFR};
        _config.pins.cytron[2] = {(int8_t)dirBL, (int8_t)pwmBL};
        _config.pins.cytron[3] = {(int8_t)dirBR, (int8_t)pwmBR};
    } else if (driverType == DriverType::DRV8833) {
        _config.pins.drv8833[0] = {(int8_t)dirFL, (int8_t)pwmFL};
        _config.pins.drv8833[1] = {(int8_t)dirFR, (int8_t)pwmFR};
        _config.pins.drv8833[2] = {(int8_t)dirBL, (int8_t)pwmBL};
        _config.pins.drv8833[3] = {(int8_t)dirBR, (int8_t)pwmBR};
    }
    
    _initDefaults();
}

Mecmotor::Mecmotor(DriverType driverType,
                   int in1FL, int in2FL, int pwmFL,
                   int in1FR, int in2FR, int pwmFR,
                   int in1BL, int in2BL, int pwmBL,
                   int in1BR, int in2BR, int pwmBR,
                   int stbyPin) {
    _config.driverType = driverType;
    _config.invertFL = false;
    _config.invertFR = false;
    _config.invertBL = false;
    _config.invertBR = false;
    _config.stbyPin = (int8_t)stbyPin;
    
    _config.pins.l298n[0] = {(int8_t)in1FL, (int8_t)in2FL, (int8_t)pwmFL};
    _config.pins.l298n[1] = {(int8_t)in1FR, (int8_t)in2FR, (int8_t)pwmFR};
    _config.pins.l298n[2] = {(int8_t)in1BL, (int8_t)in2BL, (int8_t)pwmBL};
    _config.pins.l298n[3] = {(int8_t)in1BR, (int8_t)in2BR, (int8_t)pwmBR};
    
    _initDefaults();
}

void Mecmotor::_initDefaults() {
    _initialized = false;
    _maxSpeed = 255;
    _deadzone = MEC_DEFAULT_DEADZONE;
    _stopMode = StopMode::COAST;
    _rampingEnabled = false;
    _rampRate = MEC_DEFAULT_ACCEL_RATE;
    
    for (int i = 0; i < 4; i++) {
        _currentSpeeds[i] = 0;
        _targetSpeeds[i] = 0;
    }
    
    #if defined(ESP32)
    for (int i = 0; i < 4; i++) {
        _ledcChannels[i] = 0;
    }
    #endif
}

// ============================================================================
// Initialization
// ============================================================================

void Mecmotor::begin() {
    MEC_LOGLN(F("[Mecmotor] Initializing..."));
    
    _initPins();
    
    // Enable TB6612 standby pin if configured
    if (_config.stbyPin >= 0) {
        pinMode(_config.stbyPin, OUTPUT);
        digitalWrite(_config.stbyPin, HIGH);
        MEC_LOGLN(F("[Mecmotor] STBY pin enabled"));
    }
    
    _initialized = true;
    
    // Stop all motors initially
    stop();
    
    MEC_LOGLN(F("[Mecmotor] Initialization complete"));
}

void Mecmotor::_initPins() {
    #if defined(ESP32)
    // ESP32: Use LEDC for PWM
    MEC_LOGLN(F("[Mecmotor] Configuring ESP32 LEDC PWM"));
    #endif
    
    switch (_config.driverType) {
        case DriverType::L298N:
        case DriverType::TB6612:
            for (int i = 0; i < 4; i++) {
                pinMode(_config.pins.l298n[i].in1, OUTPUT);
                pinMode(_config.pins.l298n[i].in2, OUTPUT);
                pinMode(_config.pins.l298n[i].en, OUTPUT);
                
                #if defined(ESP32)
                // Setup LEDC channel for ESP32
                _ledcChannels[i] = _nextChannel++;
                ledcSetup(_ledcChannels[i], MEC_PWM_FREQ, MEC_PWM_RESOLUTION);
                ledcAttachPin(_config.pins.l298n[i].en, _ledcChannels[i]);
                #endif
            }
            break;
            
        case DriverType::CYTRON_MD:
            for (int i = 0; i < 4; i++) {
                pinMode(_config.pins.cytron[i].dir, OUTPUT);
                pinMode(_config.pins.cytron[i].pwm, OUTPUT);
                
                #if defined(ESP32)
                _ledcChannels[i] = _nextChannel++;
                ledcSetup(_ledcChannels[i], MEC_PWM_FREQ, MEC_PWM_RESOLUTION);
                ledcAttachPin(_config.pins.cytron[i].pwm, _ledcChannels[i]);
                #endif
            }
            break;
            
        case DriverType::DRV8833:
            for (int i = 0; i < 4; i++) {
                pinMode(_config.pins.drv8833[i].in1, OUTPUT);
                pinMode(_config.pins.drv8833[i].in2, OUTPUT);
                
                // DRV8833 needs both pins to be PWM capable
                #if defined(ESP32)
                // Use two LEDC channels per motor for DRV8833
                _ledcChannels[i] = _nextChannel;
                _nextChannel += 2; // Reserve two channels
                ledcSetup(_ledcChannels[i], MEC_PWM_FREQ, MEC_PWM_RESOLUTION);
                ledcSetup(_ledcChannels[i] + 1, MEC_PWM_FREQ, MEC_PWM_RESOLUTION);
                ledcAttachPin(_config.pins.drv8833[i].in1, _ledcChannels[i]);
                ledcAttachPin(_config.pins.drv8833[i].in2, _ledcChannels[i] + 1);
                #endif
            }
            break;
    }
}

void Mecmotor::_writePWM(uint8_t channel, uint8_t pin, uint8_t value) {
    #if defined(ESP32)
    ledcWrite(channel, value);
    #else
    analogWrite(pin, value);
    #endif
}

// ============================================================================
// Configuration Methods
// ============================================================================

void Mecmotor::setInversion(bool fl, bool fr, bool bl, bool br) {
    _config.invertFL = fl;
    _config.invertFR = fr;
    _config.invertBL = bl;
    _config.invertBR = br;
    MEC_LOGLN(F("[Mecmotor] Motor inversion updated"));
}

void Mecmotor::setMaxSpeed(uint8_t maxSpeed) {
    _maxSpeed = maxSpeed;
    MEC_LOG(F("[Mecmotor] Max speed set to: "));
    MEC_LOGLN(maxSpeed);
}

void Mecmotor::setDeadzone(uint8_t deadzone) {
    _deadzone = deadzone;
    MEC_LOG(F("[Mecmotor] Deadzone set to: "));
    MEC_LOGLN(deadzone);
}

void Mecmotor::setRamping(bool enable, uint8_t rate) {
    _rampingEnabled = enable;
    _rampRate = rate;
    MEC_LOG(F("[Mecmotor] Ramping: "));
    MEC_LOGLN(enable ? F("enabled") : F("disabled"));
}

void Mecmotor::setStopMode(StopMode mode) {
    _stopMode = mode;
    MEC_LOG(F("[Mecmotor] Stop mode: "));
    MEC_LOGLN(mode == StopMode::BRAKE ? F("BRAKE") : F("COAST"));
}

// ============================================================================
// Motor Control - Low Level
// ============================================================================

int16_t Mecmotor::_applyDeadzone(int16_t speed) {
    if (abs(speed) < _deadzone) {
        return 0;
    }
    return speed;
}

int16_t Mecmotor::_clampSpeed(int16_t speed) {
    speed = _applyDeadzone(speed);
    if (speed > 0) {
        return min((int16_t)speed, (int16_t)_maxSpeed);
    } else {
        return max((int16_t)speed, (int16_t)-_maxSpeed);
    }
}

void Mecmotor::_setMotorRaw(uint8_t motorIndex, int16_t speed) {
    if (motorIndex >= 4) return;
    
    // Apply inversion
    bool invert = false;
    switch (motorIndex) {
        case 0: invert = _config.invertFL; break;
        case 1: invert = _config.invertFR; break;
        case 2: invert = _config.invertBL; break;
        case 3: invert = _config.invertBR; break;
    }
    if (invert) speed = -speed;
    
    // Get absolute speed
    uint8_t pwmValue = (uint8_t)min(abs(speed), 255);
    bool forward = (speed >= 0);
    
    switch (_config.driverType) {
        case DriverType::L298N:
        case DriverType::TB6612: {
            MotorPinsL298N& pins = _config.pins.l298n[motorIndex];
            if (speed == 0) {
                // Stop
                if (_stopMode == StopMode::BRAKE) {
                    digitalWrite(pins.in1, HIGH);
                    digitalWrite(pins.in2, HIGH);
                } else {
                    digitalWrite(pins.in1, LOW);
                    digitalWrite(pins.in2, LOW);
                }
                _writePWM(_ledcChannels[motorIndex], pins.en, 0);
            } else {
                digitalWrite(pins.in1, forward ? HIGH : LOW);
                digitalWrite(pins.in2, forward ? LOW : HIGH);
                _writePWM(_ledcChannels[motorIndex], pins.en, pwmValue);
            }
            break;
        }
        
        case DriverType::CYTRON_MD: {
            MotorPinsCytron& pins = _config.pins.cytron[motorIndex];
            digitalWrite(pins.dir, forward ? HIGH : LOW);
            _writePWM(_ledcChannels[motorIndex], pins.pwm, pwmValue);
            break;
        }
        
        case DriverType::DRV8833: {
            MotorPinsDRV8833& pins = _config.pins.drv8833[motorIndex];
            if (speed == 0) {
                // Coast stop
                #if defined(ESP32)
                ledcWrite(_ledcChannels[motorIndex], 0);
                ledcWrite(_ledcChannels[motorIndex] + 1, 0);
                #else
                analogWrite(pins.in1, 0);
                analogWrite(pins.in2, 0);
                #endif
            } else if (forward) {
                #if defined(ESP32)
                ledcWrite(_ledcChannels[motorIndex], pwmValue);
                ledcWrite(_ledcChannels[motorIndex] + 1, 0);
                #else
                analogWrite(pins.in1, pwmValue);
                analogWrite(pins.in2, 0);
                #endif
            } else {
                #if defined(ESP32)
                ledcWrite(_ledcChannels[motorIndex], 0);
                ledcWrite(_ledcChannels[motorIndex] + 1, pwmValue);
                #else
                analogWrite(pins.in1, 0);
                analogWrite(pins.in2, pwmValue);
                #endif
            }
            break;
        }
    }
    
    _currentSpeeds[motorIndex] = speed;
}

void Mecmotor::setMotorSpeed(Motor motor, int16_t speed) {
    uint8_t idx = static_cast<uint8_t>(motor);
    speed = _clampSpeed(speed);
    
    if (_rampingEnabled) {
        _targetSpeeds[idx] = speed;
    } else {
        _setMotorRaw(idx, speed);
    }
}

void Mecmotor::setMotorSpeeds(int16_t fl, int16_t fr, int16_t bl, int16_t br) {
    int16_t speeds[4] = {
        _clampSpeed(fl),
        _clampSpeed(fr),
        _clampSpeed(bl),
        _clampSpeed(br)
    };
    
    if (_rampingEnabled) {
        for (int i = 0; i < 4; i++) {
            _targetSpeeds[i] = speeds[i];
        }
    } else {
        for (int i = 0; i < 4; i++) {
            _setMotorRaw(i, speeds[i]);
        }
    }
}

// ============================================================================
// Non-blocking Update / Ramping
// ============================================================================

bool Mecmotor::update() {
    if (!_rampingEnabled) {
        return false;
    }
    
    bool stillRamping = false;
    
    for (int i = 0; i < 4; i++) {
        int16_t current = _currentSpeeds[i];
        int16_t target = _targetSpeeds[i];
        
        if (current != target) {
            stillRamping = true;
            
            if (current < target) {
                current += _rampRate;
                if (current > target) current = target;
            } else {
                current -= _rampRate;
                if (current < target) current = target;
            }
            
            _setMotorRaw(i, current);
        }
    }
    
    return stillRamping;
}

bool Mecmotor::isRamping() const {
    for (int i = 0; i < 4; i++) {
        if (_currentSpeeds[i] != _targetSpeeds[i]) {
            return true;
        }
    }
    return false;
}

int16_t Mecmotor::getMotorSpeed(Motor motor) const {
    return _currentSpeeds[static_cast<uint8_t>(motor)];
}

int16_t Mecmotor::getTargetSpeed(Motor motor) const {
    return _targetSpeeds[static_cast<uint8_t>(motor)];
}

// ============================================================================
// Mecanum Kinematics
// ============================================================================

void Mecmotor::_calculateMecanumSpeeds(float vx, float vy, float omega, 
                                        int16_t* speeds, uint8_t maxSpeed) {
    /*
     * Mecanum wheel inverse kinematics:
     * 
     * For a standard mecanum wheel configuration:
     *   FL \  / FR    (wheels angled like this)
     *   BL /  \ BR
     * 
     * FL = vx + vy + omega
     * FR = vx - vy - omega
     * BL = vx - vy + omega
     * BR = vx + vy - omega
     * 
     * Where:
     *   vx = forward/backward velocity
     *   vy = left/right strafe velocity
     *   omega = rotational velocity (positive = clockwise)
     */
    
    float fl = vx + vy + omega;
    float fr = vx - vy - omega;
    float bl = vx - vy + omega;
    float br = vx + vy - omega;
    
    // Find the maximum magnitude for normalization
    float maxMag = max(max(fabs(fl), fabs(fr)), max(fabs(bl), fabs(br)));
    
    // Normalize if any value exceeds 1.0
    if (maxMag > 1.0f) {
        fl /= maxMag;
        fr /= maxMag;
        bl /= maxMag;
        br /= maxMag;
    }
    
    // Scale to PWM range
    speeds[0] = (int16_t)(fl * maxSpeed);
    speeds[1] = (int16_t)(fr * maxSpeed);
    speeds[2] = (int16_t)(bl * maxSpeed);
    speeds[3] = (int16_t)(br * maxSpeed);
}

// ============================================================================
// Advanced Drive Controls
// ============================================================================

void Mecmotor::drive(float vx, float vy, float omega, uint8_t maxSpeed) {
    // Clamp inputs
    vx = constrain(vx, -1.0f, 1.0f);
    vy = constrain(vy, -1.0f, 1.0f);
    omega = constrain(omega, -1.0f, 1.0f);
    maxSpeed = min(maxSpeed, _maxSpeed);
    
    int16_t speeds[4];
    _calculateMecanumSpeeds(vx, vy, omega, speeds, maxSpeed);
    setMotorSpeeds(speeds[0], speeds[1], speeds[2], speeds[3]);
}

void Mecmotor::driveAngle(float angleDeg, float speed, float omega, uint8_t maxSpeed) {
    // Convert angle to radians
    float angleRad = angleDeg * PI / 180.0f;
    
    // Convert polar to cartesian
    float vx = speed * cos(angleRad);
    float vy = speed * sin(angleRad);
    
    // Call vector drive
    drive(vx, vy, omega, maxSpeed);
}

void Mecmotor::driveFieldCentric(float vx, float vy, float omega, 
                                  float robotHeadingDeg, uint8_t maxSpeed) {
    // Convert heading to radians
    float headingRad = robotHeadingDeg * PI / 180.0f;
    
    // Rotate input vector by negative robot heading to transform
    // from field-centric to robot-centric
    float cosH = cos(-headingRad);
    float sinH = sin(-headingRad);
    
    float robotVx = vx * cosH - vy * sinH;
    float robotVy = vx * sinH + vy * cosH;
    
    // Drive with transformed coordinates
    drive(robotVx, robotVy, omega, maxSpeed);
}

// ============================================================================
// Basic Direction Controls
// ============================================================================

void Mecmotor::forward(int speed) {
    speed = min(speed, (int)_maxSpeed);
    setMotorSpeeds(speed, speed, speed, speed);
}

void Mecmotor::backward(int speed) {
    speed = min(speed, (int)_maxSpeed);
    setMotorSpeeds(-speed, -speed, -speed, -speed);
}

void Mecmotor::left(int speed) {
    // Rotate CCW: left side backward, right side forward
    speed = min(speed, (int)_maxSpeed);
    setMotorSpeeds(-speed, speed, -speed, speed);
}

void Mecmotor::right(int speed) {
    // Rotate CW: left side forward, right side backward
    speed = min(speed, (int)_maxSpeed);
    setMotorSpeeds(speed, -speed, speed, -speed);
}

void Mecmotor::strafeRight(int speed) {
    // Strafe right: FL+, FR-, BL-, BR+
    speed = min(speed, (int)_maxSpeed);
    setMotorSpeeds(speed, -speed, -speed, speed);
}

void Mecmotor::strafer(int speed) {
    strafeRight(speed);
}

void Mecmotor::strafeLeft(int speed) {
    // Strafe left: FL-, FR+, BL+, BR-
    speed = min(speed, (int)_maxSpeed);
    setMotorSpeeds(-speed, speed, speed, -speed);
}

void Mecmotor::strafel(int speed) {
    strafeLeft(speed);
}

void Mecmotor::diagonalFR(int speed) {
    // Forward-right diagonal: FL+, FR0, BL0, BR+
    speed = min(speed, (int)_maxSpeed);
    setMotorSpeeds(speed, 0, 0, speed);
}

void Mecmotor::diagonalFL(int speed) {
    // Forward-left diagonal: FL0, FR+, BL+, BR0
    speed = min(speed, (int)_maxSpeed);
    setMotorSpeeds(0, speed, speed, 0);
}

void Mecmotor::diagonalBR(int speed) {
    // Backward-right diagonal: FL0, FR-, BL-, BR0
    speed = min(speed, (int)_maxSpeed);
    setMotorSpeeds(0, -speed, -speed, 0);
}

void Mecmotor::diagonalBL(int speed) {
    // Backward-left diagonal: FL-, FR0, BL0, BR-
    speed = min(speed, (int)_maxSpeed);
    setMotorSpeeds(-speed, 0, 0, -speed);
}

// Pivot controls (backward compatible)
void Mecmotor::pivotfr(int speed) {
    speed = min(speed, (int)_maxSpeed);
    setMotorSpeeds(speed, -speed, 0, 0);
}

void Mecmotor::pivotfl(int speed) {
    speed = min(speed, (int)_maxSpeed);
    setMotorSpeeds(-speed, speed, 0, 0);
}

void Mecmotor::pivotbr(int speed) {
    speed = min(speed, (int)_maxSpeed);
    setMotorSpeeds(0, 0, speed, -speed);
}

void Mecmotor::pivotbl(int speed) {
    speed = min(speed, (int)_maxSpeed);
    setMotorSpeeds(0, 0, -speed, speed);
}

// ============================================================================
// Stop Controls
// ============================================================================

void Mecmotor::stop() {
    setMotorSpeeds(0, 0, 0, 0);
    
    // If ramping, immediately stop (don't ramp to stop)
    if (_rampingEnabled) {
        for (int i = 0; i < 4; i++) {
            _targetSpeeds[i] = 0;
            _setMotorRaw(i, 0);
        }
    }
}

void Mecmotor::brake() {
    StopMode oldMode = _stopMode;
    _stopMode = StopMode::BRAKE;
    stop();
    _stopMode = oldMode;
}

void Mecmotor::coast() {
    StopMode oldMode = _stopMode;
    _stopMode = StopMode::COAST;
    stop();
    _stopMode = oldMode;
}
