#include <Dog.h>
#include "SoftwareSerial.h"
#include "Timer.h"

RobotDog dog;

const int num_waypoints = 2;
float travel = 80;
//Point waypoints[num_waypoints] = {Point(70, 0, 0), Point(-70, 0, 0)};
Point waypoints[num_waypoints] = {Point(0, 0, travel), Point(0, 0, -travel)};
//Point waypoints[num_waypoints] = {Point(0, travel, 0), Point(0, -travel, 0)};


int idx = 1;
int time = 2000;
Timer timer;

void setup() {
  Serial.begin(9600);  
  while(!Serial);

  Serial.println("Starting...");
    dog.useIMU(false);
  dog.begin();
  dog.sendAllSignals();

  Serial.println("-----------------");
  Serial.println("Start Loop");
  dog.getLeg(0)->getCurrentFootPositionFromShoulder().print();
  dog.setFromPosition(ROT_ZERO, Point(0, 0, -travel/2), Frame::BODY);
  dog.operate();
  delay(1000);
  dog.getLeg(0)->getCurrentFootPositionFromShoulder().print();

  timer.reset(time);
}

void loop() {
  if (timer.timeOut()) {
    dog.getLeg(0)->getCurrentFootPositionFromShoulder().print();
    timer.reset();
    idx = (idx+1)%num_waypoints;
    dog.setFromPosition(ROT_ZERO, waypoints[idx], Frame::BODY, time);
    
  }
  dog.operate();
}
