#ifndef mecmotor_h
#define mecmotor_h

#include <Arduino.h>

class mecmotor {
public:
    // Default constructor with predefined ESP32 pins
    mecmotor();

    // Constructor with custom pins
    mecmotor(int in1, int in2, int ena,
             int in3, int in4, int enb,
             int in1_2, int in2_2, int ena_2,
             int in3_2, int in4_2, int enb_2);

    // Basic motor controls
    void forward(int speed);
    void backward(int speed);
    void left(int speed);
    void right(int speed);
    void strafer(int speed);
    void strafel(int speed);
    void pivotfr(int speed);
    void pivotfl(int speed);
    void pivotbr(int speed);
    void pivotbl(int speed);
    void stop();

private:
    int IN1, IN2, ENA;
    int IN3, IN4, ENB;
    int IN1_2, IN2_2, ENA_2;
    int IN3_2, IN4_2, ENB_2;

    void setMotor(int in1, int in2, bool state1, bool state2, int enPin, int speed);
};

#endif
