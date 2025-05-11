#include <mecmotor.h>

// Use default pins
mecmotor motor;  

// OR define your own pins (Uncomment to test)
// mecmotor bot(18, 19, 21, 5, 23, 22, 13, 12, 14, 2, 4, 15);
//find a project made by author at https://github.com/beastbroak30/ESP-NOW_mecanum-carRC
void setup() {
  Serial.begin(115200);
  Serial.println("Mecanum Robot Ready!");
}

void loop() {
  // put your main code here, to run repeatedly:
  motor.forward(180);
  delay (2000);
  motor.backward(180);
  delay(2000);
  motor.strafel(255);
  delay(2000);
  motor.strafer(250);
  delay(2000);
}
