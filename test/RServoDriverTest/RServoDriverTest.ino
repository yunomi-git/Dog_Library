#include <RServoDriver.h>

RServoDriver driver;
int sig;
void setup() {
  Serial.begin(9600);
  // put your setup code here, to run once:
  driver.defaultStartup();
//  driver.reverseDirection(0);
//  driver.setOffset(0, 45);
  driver.gotoAngle(0, 0);
  
}

void loop() {
  // put your main code here, to run repeatedly:
  if (Serial.available()) {
    sig = Serial.parseInt();
    if (sig != 0) {
      if (sig == 100)
        sig = 0;
       driver.gotoAngle(0, sig);
       Serial.println(sig);
    }
  }
}
