#include <Leg.h>
RServoDriver servo_driver;
int sig;
DogLeg leg;
bool error;
Point mounting_point = Point(112.5, 60, 0);
double timer;
Rot orientation = ROT_ZERO;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  servo_driver.defaultStartup();
  leg = DogLeg(&servo_driver, 0, 1, 2, false, false, &orientation, mounting_point, 70);
  leg.calibrateServos(4,-2,9);
  delay(1000);
  Serial.println(leg.getCurrentFootPositionFromShoulder().z);
  Serial.println("to home");
  leg.sendSignals();
  Serial.println(leg.getCurrentFootPositionFromShoulder().z);
  delay(1000);
  
//  // Test Instant Move
//  Serial.println("to -140");
//  leg.setToPositionFromShoulder(Point(0,0,-140));
//  leg.operate();
//  leg.sendSignals();

  // Test gradual move
  Serial.println("gradual to -140");
  leg.setToPositionFromShoulder(Point(40,40,-120), Frame::BODY, 1000);
  timer = millis();
}

void loop() {
  if (millis() - timer > 5) {
    leg.operate();
    leg.sendSignals();
    timer = millis();
  }
}
