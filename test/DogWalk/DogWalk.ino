#include <Dog.h>
#include "SoftwareSerial.h"
#include "Timer.h"

// Communication
// rx to pin9, tx to pin10
#define XBee Serial2

double joystick[4] = {0,0,0,0}; // RY, RX, LY, LX
double trot_yaw;
double trot_x;
double trot_y;

RobotDog dog;

void setup() {
  Serial.begin(9600);  
  XBee.begin(19200);

  Serial.println("Starting...");
  dog.begin();
  dog.useIMU(true);
  dog.sendAllSignals();

  Serial.println("Taring IMU...");
  delay(1000);
  dog.tareIMU();

  Serial.println("...Tared");
  delay(500);

  Serial.println("Setting up trot");
  dog.beginTrot(&trot_x, &trot_y, &trot_yaw);
  dog.setTrotFrame(Frame::GROUND);
  
  Serial.println("-----------------");
  Serial.println("Start Loop");
}

void loop() {
  if (XBee.available()) {
    // First recieve joystick values
    for (int i = 0; i < 4; i++) {
      joystick[i] = (XBee.parseFloat());
      // RY, RX, LY, LX
//      Serial.print(joystick[i]), Serial.print(" ");
    }
    
    // Then retrieve button values
    String s = XBee.readStringUntil('\n');
    //Serial.println(s);
//    Serial.println();
  }
  trot_x = joystick[0] * 10;
  trot_y = joystick[1] * 10;
  trot_yaw = joystick[3] * -5;
//  dog.printCentroid();
//  dog.printLeg(2);

  dog.trot();
}
