#include <Dog.h>
#include "Timer.h"

RobotDog dog;
DogLeg *leg0;
DogLeg *leg1;
DogLeg *leg2;
DogLeg *leg3;
double timer;
Timer timer_;

Point mounting_pos = Point(112.5, 60, 0);

// z -36.61 for E:90

void setup() {
  Serial.begin(9600);  
  timer = millis();

  Serial.println("Starting...");
  dog.begin();
  dog.useIMU(false);
  dog.sendAllSignals();
  leg0 = dog.getLeg(0);
  leg1 = dog.getLeg(1);
  leg2 = dog.getLeg(2);
  leg3 = dog.getLeg(3);

  Serial.println("Taring IMU...");
  delay(1000);
  dog.tareIMU();

  Serial.println("...Tared");
  delay(500);
  
  Serial.println("-----------------");
  Serial.println("Start Loop");

  calibrate_COM();

}


void calibrate_COM() {
  dog.useIMU(true);
  dog.setFromCentroid(Rot(0, 0, 0), Point(0,0,120), Frame::GROUND, 500);
  timer_.reset(500);
  while (!timer_.timeOut()) {
    dog.operate();
  }

  timer_.reset(1000);
  while(!timer_.timeOut()) {
    Serial.println("Taring again...");
  }
  dog.tareIMU();
  
  Serial.println("Grab hold of Dog, switching to 2 legs");
  int state = 0;
  leg0->setState(CoordinationState::FLOATING);
  leg0->setToPositionFromShoulder(Point(0,0,-60), Frame::BODY, 1000);
  leg2->setState(CoordinationState::FLOATING);
  leg2->setToPositionFromShoulder(Point(0,0,-70), Frame::BODY, 1000);
  
  timer_.reset(1100);
  while (1) {
    if (state == 0) {
      if (timer_.timeOut()) {
        Serial.println("legs have been raised. Fixing leg0");
        state = 1;
      }
    } else if (state == 1) {
      dog.printIMU();
    } 
    dog.operate();
  }
}




void loop() {
  dog.operate();
}
