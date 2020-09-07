#include "kg20servo.h"
kg20servo s;
int sig;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

s.attach(6);
//s.setOffset(45);

//s.gotoAngle(0);
s.writeMicroseconds(1500);
}

void loop() {
  // put your main code here, to run repeatedly:
  if (Serial.available()) {
    sig = Serial.parseInt();
    if (sig != 0)
      s.gotoAngle(sig);
    
  }
}
