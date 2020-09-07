#include <Leg.h>
RServoDriver servo_driver;
int sig;
DogLeg leg;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  servo_driver.defaultStartup();
  leg = DogLeg(&servo_driver, 12, 13, 14, true, false,Point(-60, -112.5, 0));
  leg.calibrateServos(0,0,0);
  leg.sendSignals();
//  leg.moveByPosition(Point(-20, -20, 0));
//  leg.sendSignals();
//  leg.gotoPositionFromBody(); // Calibrate Shoulder
}

void loop() {
  // put your main code here, to run repeatedly:
  if (Serial.available()) {
    sig = Serial.parseInt();
    if (sig != 0) {
//      if (sig == 100)
//        sig = 0;
        leg.gotoPosition(30, sig, -100);
        leg.sendSignals();
//       servo_driver.gotoAngle(1, sig);
       Serial.println(sig);
    }
  }
}
