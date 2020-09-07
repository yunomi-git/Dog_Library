// Checks if all servos on dog are calibrated correctly
#include <Dog.h>
#include <RServoDriver.h>
#include "SoftwareSerial.h"
#include <math2.h>

RobotDog dog;
RServoDriver *servo_driver;
float angle;
float wishbone_angle = 155;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  while(!Serial);  

  servo_driver = dog.getServoDriver();
  servo_driver->defaultStartup();

  sendAllTo(0);
  //sendElbowTo(0);

  
}

void loop() {
  if (Serial.available()) {
    angle = Serial.parseFloat();
    if (angle != 0) {
      if (angle == 1) angle = 0;
       sendElbowTo(angle);
       Serial.println("---");
    }
  }
}


void sendAllTo(float angle) {
  sendChestTo(angle);
  sendShoulderTo(angle);
  sendElbowTo(angle);
}

void sendChestTo(float angle) {
  for (int i = 0; i<16; i+=4) {
    servo_driver->gotoAngle(i, angle);
  }
}

// 80 to -100 standard
void sendShoulderTo(float angle) {
  for (int i = 1; i<16; i+=4) {
    servo_driver->gotoAngle(i, angle);
  }
}

// 0 to 90 standard.
void sendElbowTo(float angle) {
  for (int i = 2; i<16; i+=4) {
    servo_driver->gotoAngle(i, angle - 180); // zero is pointing straight down. -180 is pointing up
  }
}
