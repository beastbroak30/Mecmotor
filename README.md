# Mecmotor
The `mecmotor` library provides functions for controlling two L298N motor drivers with predefined pins and also supports custom definition . The library allows easy manipulation of the Mecanum wheels for forward, backward, strafe, and diagonal movements.

> A simple Arduino library to control a 4-wheel mecanum robot using two L298N motor drivers on an `ESP32` or `Any Microcontroller`.
> Created by **Beastbroak30** – used in the project [ESP-NOW Mecanum Car RC project](https://github.com/beastbroak30/ESP-NOW_mecanum-carRC/).

---

## 🚗 Overview

**Mecmotor** makes it easy to control a mecanum-wheeled robot platform using an ESP32 and two L298N motor drivers. It provides beginner-friendly functions like `forward()`, `backward()`, `strafer()`, and `pivotFL()` so you can focus on building your robot instead of writing boilerplate motor code.

You can either:

* Use **default pin configuration**, or
* **Customize** all 12 control pins in the constructor.

---

## 🔧 Hardware Setup

* **MCU**: ESP32 (DevKit recommended)
* **Motor Drivers**: 2 × L298N
* **Motors**: 4 × DC motors (mecanum-compatible)
* **Power**: 2S/3S Li-ion battery (recommended)

### 🖼️ Default Pin Mapping

| Motor | IN1 | IN2 | EN  | ESP32 Pin                |
| ----- | --- | --- | --- | ------------------------ |
| FL    | IN1 | IN2 | ENA | GPIO18 / GPIO19 / GPIO21 |
| FR    | IN3 | IN4 | ENB | GPIO5 / GPIO23 / GPIO22  |
| BL    | IN1 | IN2 | ENA | GPIO13 / GPIO12 / GPIO14 |
| BR    | IN3 | IN4 | ENB | GPIO2 / GPIO4 / GPIO15   |

> You can override these with your own pin numbers.

---

## 📦 Installation

### 🟢 Arduino IDE

> You can install it via arduino library manager search for `mecmotor`
> or you can manually install using below steps

1. Download this repo as `.zip` or use:

   ```
   git clone https://github.com/beastbroak30/Mecmotor.git
   ```
2. Copy it to your Arduino libraries folder:

   * Windows: `Documents/Arduino/libraries/`
   * Linux/Mac: `~/Arduino/libraries/`
3. Restart Arduino IDE.
4. Go to **File > Examples > Mecmotor > BasicTest**.

---

## 🚀 Getting Started

### 🟢 Default Pins (ESP32)

```cpp
#include <mecmotor.h>

mecmotor motor;  // Uses default pins

void setup() {
  Serial.begin(115200);
}

void loop() {
  motor.forward(200);
  delay(1000);
  motor.backward(200);
  delay(1000);
  motor.strafer(255);
  delay(500);
}
```

### 🔧 Custom Pins

```cpp
#include <mecmotor.h>

// Constructor with 12 pin arguments
mecmotor motor(18, 19, 21,   // Front Left
             5,  23, 22,   // Front Right
             13, 12, 14,   // Back Left
             2,  4,  15);  // Back Right

void setup() {
  Serial.begin(115200);
}

void loop() {
  motor.strafer(255);
  delay(500);
  motor.stop();
}
```

---

## 🧐 API Reference

| Function              | Description          |
| --------------------- | -------------------- |
| `forward(int speed)`  | Move forward         |
| `backward(int speed)` | Move backward        |
| `left(int speed)`     | Turn left            |
| `right(int speed)`    | Turn right           |
| `strafer(int speed)`  | Strafe right         |
| `strafel(int speed)`  | Strafe left          |
| `pivotfr(int speed)`  | Pivot forward right  |
| `pivotfl(int speed)`  | Pivot forward left   |
| `pivotbr(int speed)`  | Pivot backward right |
| `pivotbl(int speed)`  | Pivot backward left  |
| `stop()`              | Stop all motors      |

Speed range: `0–255`

---

## 📂 Folder Structure

```
Mecmotor/
├── examples/
│   └── BasicTest/        # Example sketch
├── src/     # Library source
│    ├── mecmotor.cpp
│    └── mecmotor.h       # Header file
├── keywords.txt          # IDE highlighting
├── library.properties    # Arduino metadata
└── README.md             # You're reading it!
```

---

## 📌 Project Showcase

This library was originally developed for my project:

🔗 **ESP-NOW Mecanum Car RC**
[https://github.com/beastbroak30/ESP-NOW\_mecanum-carRC](https://github.com/beastbroak30/ESP-NOW_mecanum-carRC)

It uses ESP-NOW for wireless control of a fully mecanum-driven robot — built from scratch using ESP32 and L298N motor drivers.

---

## 🧑‍💻 Author & Contributor

**Beastbroak30**
Email: `akantarip30@gmail.com`
GitHub: [@beastbroak30](https://github.com/beastbroak30)

---

## 📄 License

This project is licensed under the MIT License.

---

Enjoy building awesome bots! 🤖✨
