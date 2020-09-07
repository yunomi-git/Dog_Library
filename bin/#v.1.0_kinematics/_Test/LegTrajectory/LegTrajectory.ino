#include <Leg.h>
RServoDriver servo_driver;
int sig;
DogLeg leg;

Point mounting_point = Point(60, 112.5, 0);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  servo_driver.defaultStartup();
  leg = DogLeg(&servo_driver, 0, 1, 2, false, false, mounting_point);
  leg.calibrateServos(4,-2,9);
  leg.sendSignals();
}

void loop() {
  leg.operate();
}
