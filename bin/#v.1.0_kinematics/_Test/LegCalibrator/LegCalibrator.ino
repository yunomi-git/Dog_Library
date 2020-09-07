#include <Leg.h>
RServoDriver servo_driver;
int sig;
DogLeg leg;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  servo_driver.defaultStartup();
  leg = DogLeg(&servo_driver, 0, 1, 2, true, true);
  leg.calibrateServos(-8, 1.5, 0.5);
 leg.gotoAngles(0, 0, -120); // Calibrate Shoulder
//servo_driver.gotoAngle(0, 0);
//servo_driver.gotoAngle(1, 0);
//  servo_driver.gotoAngle(2, 0);
//  leg.gotoAngles(0,-30,-180); // Calibrate Elbow
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
