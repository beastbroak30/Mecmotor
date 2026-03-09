# Mecmotor - Advanced Mecanum Wheel Robot Library

**The most feature-rich Arduino library for controlling 4-wheel Mecanum robots.**

![Arduino Library Manager](https://img.shields.io/badge/ArduinoLib-MecMotor-brightgreen?style=flat)
[![License: MIT](https://img.shields.io/badge/license-MIT-blue.svg)](https://opensource.org/licenses/MIT)
![version](https://img.shields.io/badge/version-1.2.0-blue)
![Platform](https://img.shields.io/badge/platform-ESP32%20%7C%20Arduino-orange)

> A powerful Arduino library to control 4-wheel Mecanum robots with multiple motor driver support, field-centric control, smooth ramping, and more!
> Created by **Beastbroak30** – [ESP-NOW Mecanum Car RC project](https://github.com/beastbroak30/ESP-NOW_mecanum-carRC/)

---

## ✨ New in v1.2.0

- **Multi-driver support**: L298N, TB6612FNG, DRV8833, Cytron MD10C/MDD10A
- **Field-centric drive**: Joystick-style control that works regardless of robot orientation
- **Polar drive**: Move at any angle with `driveAngle(angle, speed, rotation)`
- **Smooth acceleration ramping**: Prevent jerky starts/stops
- **Motor inversion flags**: Easy wheel direction correction
- **Brake vs Coast modes**: Active braking or passive coast to stop
- **Deadzone filtering**: Eliminate motor hum at low PWM
- **Non-blocking operation**: RTOS-safe, no blocking delays
- **ESP32 LEDC PWM**: Native hardware PWM on ESP32
- **Any MCU support**: Works with Arduino Uno, Mega, STM32, and more

---

## 🚗 Overview

**Mecmotor** makes it easy to control a Mecanum-wheeled robot platform. Whether you're building a simple RC car or an advanced autonomous robot, Mecmotor provides:

- **Beginner-friendly**: Simple functions like `forward()`, `strafeRight()`, `stop()`
- **Pro features**: Vector math (`drive(vx, vy, omega)`), field-centric control
- **Hardware flexibility**: Support for popular motor drivers
- **Clean code**: Well-documented, efficient, Arduino IDE compatible

---

## 🔧 Supported Motor Drivers

| Driver | Description | Pins per Motor |
|--------|-------------|----------------|
| **L298N** | Classic dual H-bridge (default) | IN1, IN2, EN |
| **TB6612FNG** | Efficient dual driver with STBY | IN1, IN2, PWM, STBY |
| **DRV8833** | Low-voltage dual driver | IN1, IN2 (both PWM) |
| **Cytron MD10C/MDD10A** | High-current DIR+PWM driver | DIR, PWM |

---

## 📦 Installation

### Arduino Library Manager (Recommended)

1. Open Arduino IDE
2. Go to **Sketch > Include Library > Manage Libraries**
3. Search for `Mecmotor`
4. Click **Install**

### Manual Installation

```bash
git clone https://github.com/beastbroak30/Mecmotor.git
```
Copy to your Arduino libraries folder:
- **Windows**: `Documents/Arduino/libraries/`
- **Linux/Mac**: `~/Arduino/libraries/`

---

## 🚀 Quick Start

### ESP32 with L298N (Default Pins)

```cpp
#include <mecmotor.h>

Mecmotor robot;

void setup() {
    Serial.begin(115200);
    robot.begin();  // Initialize pins and PWM
    
    // Optional configurations
    robot.setMaxSpeed(200);      // Limit max speed
    robot.setDeadzone(25);       // Ignore low PWM values
    robot.setRamping(true, 15);  // Enable smooth acceleration
}

void loop() {
    robot.forward(180);
    delay(1000);
    
    robot.strafeRight(180);
    delay(1000);
    
    robot.stop();
    delay(500);
}
```

### Joystick Vector Control

```cpp
#include <mecmotor.h>

Mecmotor robot;

void setup() {
    robot.begin();
    robot.setRamping(true, 10);
}

void loop() {
    // Read joystick (normalized -1.0 to 1.0)
    float vx = (analogRead(A0) - 512) / 512.0;  // Forward/back
    float vy = (analogRead(A1) - 512) / 512.0;  // Left/right strafe
    float omega = (analogRead(A2) - 512) / 512.0;  // Rotation
    
    robot.drive(vx, vy, omega);
    robot.update();  // Apply ramping
    delay(20);
}
```

### Field-Centric Drive (with IMU)

```cpp
#include <mecmotor.h>

Mecmotor robot;
float robotHeading = 0;  // From gyroscope/IMU

void setup() {
    robot.begin();
}

void loop() {
    // Read joystick inputs (field-relative)
    float fieldVx = getJoystickY();  // Push forward = move toward field north
    float fieldVy = getJoystickX();  // Push right = move toward field east
    float omega = getJoystickRotation();
    
    // Robot automatically compensates for its heading
    robot.driveFieldCentric(fieldVx, fieldVy, omega, robotHeading);
}
```

---

## 🔌 Wiring Diagrams

### L298N Dual Driver Setup

```
ESP32                L298N #1 (FL + FR)           L298N #2 (BL + BR)
─────                ─────────────────           ─────────────────
GPIO18 ──────────── IN1  (FL Motor)
GPIO19 ──────────── IN2  (FL Motor)
GPIO21 ──────────── ENA  (FL PWM)
GPIO5  ──────────── IN3  (FR Motor)
GPIO23 ──────────── IN4  (FR Motor)
GPIO22 ──────────── ENB  (FR PWM)
GPIO13 ─────────────────────────────────────── IN1  (BL Motor)
GPIO12 ─────────────────────────────────────── IN2  (BL Motor)
GPIO14 ─────────────────────────────────────── ENA  (BL PWM)
GPIO2  ─────────────────────────────────────── IN3  (BR Motor)
GPIO4  ─────────────────────────────────────── IN4  (BR Motor)
GPIO15 ─────────────────────────────────────── ENB  (BR PWM)
GND    ──────────── GND ─────────────────────── GND
```

**Motor Arrangement:**
```
    FRONT
  FL ┌──┐ FR
     │  │
  BL └──┘ BR
    BACK
```

### TB6612FNG Dual Driver Setup

```
ESP32              TB6612 #1                    TB6612 #2
─────              ─────────                    ─────────
GPIO18 ─────────── AIN1 (FL)
GPIO19 ─────────── AIN2 (FL)
GPIO21 ─────────── PWMA (FL)
GPIO5  ─────────── BIN1 (FR)
GPIO23 ─────────── BIN2 (FR)
GPIO22 ─────────── PWMB (FR)
GPIO25 ─────────── STBY ────────────────────── STBY (Connect together)
GPIO13 ────────────────────────────────────── AIN1 (BL)
GPIO12 ────────────────────────────────────── AIN2 (BL)
GPIO14 ────────────────────────────────────── PWMA (BL)
GPIO2  ────────────────────────────────────── BIN1 (BR)
GPIO4  ────────────────────────────────────── BIN2 (BR)
GPIO15 ────────────────────────────────────── PWMB (BR)
3.3V   ─────────── VCC ────────────────────── VCC
GND    ─────────── GND ────────────────────── GND
```

### Cytron MDD10A Setup

```cpp
// Cytron uses only DIR + PWM per motor (simpler wiring!)
Mecmotor robot(DriverType::CYTRON_MD,
    18, 21,    // FL: DIR, PWM
    5,  22,    // FR: DIR, PWM  
    13, 14,    // BL: DIR, PWM
    2,  15     // BR: DIR, PWM
);
```

---

## 📚 API Reference

### Constructors

```cpp
// Default L298N with ESP32 pins
Mecmotor robot;

// Custom L298N pins (12 pins)
Mecmotor robot(in1FL, in2FL, enaFL, in1FR, in2FR, enaFR,
               in1BL, in2BL, enaBL, in1BR, in2BR, enaBR);

// TB6612FNG with optional standby pin
Mecmotor robot(DriverType::TB6612,
               in1FL, in2FL, pwmFL, in1FR, in2FR, pwmFR,
               in1BL, in2BL, pwmBL, in1BR, in2BR, pwmBR,
               stbyPin);

// Cytron MD (DIR + PWM style)
Mecmotor robot(DriverType::CYTRON_MD,
               dirFL, pwmFL, dirFR, pwmFR,
               dirBL, pwmBL, dirBR, pwmBR);
```

### Initialization & Configuration

| Method | Description |
|--------|-------------|
| `begin()` | Initialize pins and PWM channels (call in setup) |
| `setMaxSpeed(uint8_t max)` | Cap maximum PWM output (0-255) |
| `setDeadzone(uint8_t dz)` | Ignore PWM values below threshold |
| `setRamping(bool en, uint8_t rate)` | Enable smooth acceleration |
| `setInversion(fl, fr, bl, br)` | Flip motor directions |
| `setStopMode(StopMode mode)` | COAST or BRAKE stop behavior |

### Basic Movement

| Method | Description |
|--------|-------------|
| `forward(speed)` | Move forward |
| `backward(speed)` | Move backward |
| `left(speed)` | Rotate counter-clockwise |
| `right(speed)` | Rotate clockwise |
| `strafeLeft(speed)` / `strafel(speed)` | Strafe left |
| `strafeRight(speed)` / `strafer(speed)` | Strafe right |
| `diagonalFL/FR/BL/BR(speed)` | Diagonal movement |
| `pivotfl/fr/bl/br(speed)` | Pivot around corner |
| `stop()` | Stop all motors |
| `brake()` | Active brake stop |
| `coast()` | Passive coast stop |

### Advanced Drive

| Method | Description |
|--------|-------------|
| `drive(vx, vy, omega, maxSpeed)` | Vector drive (-1.0 to 1.0 inputs) |
| `driveAngle(deg, speed, omega, maxSpeed)` | Drive at angle (0°=forward, 90°=right) |
| `driveFieldCentric(vx, vy, omega, heading, maxSpeed)` | Field-centric with IMU heading |
| `setMotorSpeed(Motor, speed)` | Control individual motor (-255 to 255) |
| `setMotorSpeeds(fl, fr, bl, br)` | Set all four motors directly |

### Non-blocking / RTOS

| Method | Description |
|--------|-------------|
| `update()` | Apply ramping (call in loop). Returns true if still ramping |
| `isRamping()` | Check if motors are transitioning |
| `getMotorSpeed(Motor)` | Get current speed |
| `getTargetSpeed(Motor)` | Get target speed (when ramping) |

---

## 🛠️ Advanced Examples

### Smooth Ramping Demo

```cpp
#include <mecmotor.h>

Mecmotor robot;

void setup() {
    Serial.begin(115200);
    robot.begin();
    robot.setRamping(true, 5);  // Slow ramp for demo
}

void loop() {
    robot.forward(200);
    
    // Non-blocking: update until target reached
    while (robot.isRamping()) {
        robot.update();
        Serial.print("Current speed: ");
        Serial.println(robot.getMotorSpeed(Motor::FL));
        delay(50);
    }
    
    delay(1000);
    
    robot.stop();
    while (robot.isRamping()) {
        robot.update();
        delay(50);
    }
    
    delay(1000);
}
```

### Motor Inversion (Wheel Direction Fix)

```cpp
#include <mecmotor.h>

Mecmotor robot;

void setup() {
    robot.begin();
    
    // If front-left and back-right spin wrong direction:
    robot.setInversion(true, false, false, true);
}
```

### Debug Mode

```cpp
// Enable before including the library
#define MEC_DEBUG

#include <mecmotor.h>

Mecmotor robot;

void setup() {
    Serial.begin(115200);
    robot.begin();  // Will print debug messages
}
```

---

## 📂 Folder Structure

```
Mecmotor/
├── examples/
│   ├── Basic_mec/           # Simple L298N example
│   ├── ESP32_mec/           # ESP32 with L298N
│   ├── ESP32_FieldCentric/  # Field-centric joystick demo
│   ├── Arduino_Uno_L298N/   # Arduino Uno example
│   └── ESP32_TB6612/        # TB6612FNG driver example
├── src/
│   ├── mecmotor.h           # Header file
│   └── mecmotor.cpp         # Implementation
├── keywords.txt             # IDE syntax highlighting
├── library.properties       # Arduino metadata
├── LICENSE                  # MIT License
└── README.md
```

---

## 🎯 Use Cases

- **RC Cars**: Hobby robotics, ESP-NOW remote control
- **Competition Robots**: FTC, WRO, MakeX mecanum platforms
- **Autonomous Robots**: ROS integration, pathfinding
- **Education**: Learning mecanum kinematics and robotics
- **Industrial**: AGV prototypes, warehouse robots

---

## 🤝 Contributing

Contributions are welcome! Please:

1. Fork the repository
2. Create a feature branch
3. Submit a pull request

---

## 📜 License

MIT License - see [LICENSE](LICENSE) file.

---

## 🔗 Links

- **GitHub**: [github.com/beastbroak30/Mecmotor](https://github.com/beastbroak30/Mecmotor)
- **Author**: Beastbroak30 (Antarip Kar)
- **Email**: akantarip30@gmail.com
- **Related Project**: [ESP-NOW Mecanum Car RC](https://github.com/beastbroak30/ESP-NOW_mecanum-carRC/)

---

Made with ❤️ for the robotics community

This library was originally developed for my project:

🔗 **ESP-NOW Mecanum Car RC**
[https://github.com/beastbroak30/ESP-NOW\_mecanum-carRC](https://github.com/beastbroak30/ESP-NOW_mecanum-carRC)

It uses ESP-NOW for wireless control of a fully mecanum-driven robot — built from scratch using ESP32 and L298N motor drivers.

---

## 🧑‍💻 Author & Contributor

### Beastbroak30
Email: `akantarip30@gmail.com`
GitHub: [@beastbroak30](https://github.com/beastbroak30)

---

## 📄 License

This project is licensed under the MIT License.

---

Enjoy building awesome bots! 🤖✨
