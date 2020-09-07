#include <Dog.h>
#include <RServoDriver.h>
#include "SoftwareSerial.h"
#include <math2.h>
//#define DEBUG
// Communication
// rx to pin9, tx to pin10
#define XBee Serial2

double joystick[4] = {0,0,0,0}; // RY, RX, LY, LX
int mode = -1;

RobotDog dog;
double timer;

void setup() {
  Serial.begin(9600);
  XBee.begin(19200);
  pinMode(13, OUTPUT);
  
  timer = millis();
  
  dog.begin();
  dog.sendAllSignals();
  
delay(3000);
  
  Serial.println("starting");
  Serial.println("-----------------");
  Serial.println("Start Loop");
}


void loop() {
  if (XBee.available()) {
    // First revieve joystick values
    for (int i = 0; i < 4; i++) {
      joystick[i] = (joystick[i]+XBee.parseFloat())/2;
      Serial.print(joystick[i]), Serial.print(" ");
    }
    
    // Then retrieve button values
    String s = XBee.readStringUntil('\n');
    Serial.println(s);
    // Do actions
    if (s.indexOf("LZ2") != -1) {
      double x = joystick[2];
      double y = joystick[3];
      double mag = sqrt(x*x+y*y) * 15;
      double angle = -atan2(y, x) RAD;
      dog.moveToPose(0, joystick[0] * 10, joystick[1] * 30, 0, 0, 0);
      if (mag > 4) {
        if (millis() - timer > 150) {          
          dog.wave_step(angle, 10);
//          XBee.flush();
          timer = millis();
        }
      }
    } else {
       dog.moveToPose(0, joystick[0] * 10, joystick[1] * 30, joystick[2], joystick[3], 0);
    }

    if(s.indexOf("UP") != -1) {
      dog.gotoDefaultStance();
    }
        if(s.indexOf("B1") != -1) {
      mode *= -1;
    }
    if (mode == 1) {
        digitalWrite(13, HIGH);
    } else {
        digitalWrite(13, LOW);
    }
  }
}
