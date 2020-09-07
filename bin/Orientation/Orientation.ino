#include <Dog.h>
#include "SoftwareSerial.h"

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
  while(!Serial);
  XBee.begin(19200);
  pinMode(13, OUTPUT);
  
  timer = millis();
  
  dog.begin();
  dog.sendAllSignals();
  
  Serial.println("Taring IMU...");
  delay(1000);
  dog.tareIMU();

  Serial.println("...Tared");
  delay(500);
  
delay(1000);
  
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
    //dog.setFromPosition(Rot(0, joystick[0] * 10, joystick[1] * 30), Point(joystick[2], joystick[3], 0), Frame::GROUND);
    Point position = Point(0, 0, 90+joystick[2] * 10);
    dog.setFromCentroid(Rot(joystick[1]*10, joystick[0] * 10, joystick[3] * 30), position, Frame::GROUND);

    Serial.print("Desired: "); position.print();

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
  } else {
    //Serial.print(".");
  }
  dog.operate();
  dog.printIMU();
  dog.printCentroid();
  //dog.printLegs();
  
}
