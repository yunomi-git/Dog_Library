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

//  testRotationalAccuracy_WithIMU(); // Yaw back and forth, using IMU
//  testRotationalAccuracy_NoIMU(); // Yaw forth, Z up and down, without IMU
//  sweep_z_IMU(Rot(0,0,30), -20, 20); // Z up and down at a given angle
  testFixedJoint_manual_moving();
//  printLegs_IMU();
//printLegs_IMU();
}

void testFixedJoint_manual_moving() {
  dog.useIMU(true);
  dog.setFromCentroid(ROT_ZERO, Point(0,0,120), Frame::GROUND, 500);
  timer_.reset(500);
  while (!timer_.timeOut()) {
    dog.operate();
  }
  
  Serial.println("Grab hold of Dog, switching to 2 legs");
  int state = 0;
  leg0->setState(CoordinationState::FLOATING);
  leg0->setToPositionFromShoulder(Point(0,0,-100), Frame::BODY, 1000);
  leg2->setState(CoordinationState::FLOATING);
  leg2->setToPositionFromShoulder(Point(0,0,-70), Frame::BODY, 1000);
  
  timer_.reset(1100);
  while (1) {
    if (state == 0) {
      if (timer_.timeOut()) {
        Serial.println("legs have been raised. Fixing leg0");
        leg0->setState(CoordinationState::FIXED);
        timer_.reset(500);
        state = 1;
      }
    } else if (state == 1) {
      if (timer_.timeOut()) {
        leg0->setToPositionFromCentroid(mounting_pos + Point(20,0,30), Frame::GROUND, 1000);
        state = 2;
        timer_.reset(1002);
      }
    } else if (state == 2) {
      if (timer_.timeOut()) {
        leg0->setToPositionFromCentroid(mounting_pos + Point(-20,0,30), Frame::GROUND, 1000);
        state = 1;
        timer_.reset(1002);
      }
    } 
    dog.operate();
  }
}

void testFixedJoint_manual() {
  dog.useIMU(true);
  dog.setFromCentroid(ROT_ZERO, Point(0,0,120), Frame::GROUND, 500);
  timer_.reset(500);
  while (!timer_.timeOut()) {
    dog.operate();
  }
  
  Serial.println("Grab hold of Dog, switching to 2 legs");
  int state = 0;
  leg0->setState(CoordinationState::FLOATING);
  leg0->setToPositionFromShoulder(Point(0,0,-100), Frame::BODY, 1000);
  leg2->setState(CoordinationState::FLOATING);
  leg2->setToPositionFromShoulder(Point(0,0,-70), Frame::BODY, 1000);
  
  timer_.reset(1100);
  while (1) {
    if (state == 0) {
      if (timer_.timeOut()) {
        Serial.println("legs have been raised. Fixing leg0");
        leg0->setState(CoordinationState::FIXED);
        timer_.reset(1000);
        state = 1;
      }
    } else if (state == 1) {
      if (timer_.timeOut()) {
//        dog.printIMU();
//        dog.printCentroid();
//        leg0->getCurrentFootPositionFromBody(Frame::GROUND).print();
//        leg0->getCurrentFootPositionFromCentroid(Frame::GROUND).print();
      }
    } 
    dog.operate();
  }
}

// Tests accuracy without IMU. Rotation tested in Z only
void testRotationalAccuracy_NoIMU() {
  Rot r = Rot(0,0,30);
  double min = -20;
  double max = 20;
  dog.useIMU(false);
  unsigned int timer = millis();
  unsigned int period = 1000;
  int state = 0;
  Serial.print("Original: "); dog.printLeg(0);
  while (1) {
    if (state == 0) {
      if (millis() - timer > period) {
        dog.setFromCentroid(r, Point(0, 0, max), Frame::BODY, period);
        timer = millis();
        state = 1;
      }
    } else if (state == 1) {
      if (millis() - timer > period * 1.5) {
        Serial.print("First Rotation: "); dog.printLeg(0);
        Serial.println("Expected: (X: -14.93, Y:64.29, Z:-120.00)"); // Assuming rot(0,0,30);
        state = 2;
        timer = millis();
      }
    } else if (state == 2) {
      if (millis() - timer > period) {
        dog.setFromCentroid(r, Point(0, 0, min), Frame::BODY, period);
        
        timer = millis();
        state = 3;
      }
    } else if (state == 3) {
      if (millis() - timer > period*1.1) {
        Serial.print("Second Rotation: "); dog.printLeg(0);
        Serial.println("Expected: (X: -14.93, Y:64.29, Z:-80.00)");
        state = 4;
      }
    }

    dog.operate();
  }
}

// Check if rotation is accurate with IMU turned on. Rotation tested in Z only
void testRotationalAccuracy_WithIMU() {
  Rot r = Rot(0,0,30);
  dog.useIMU(true);
  unsigned int timer = millis();
  unsigned int period = 1000;
  int state = 0;
  Serial.print("Original: "); dog.printLeg(0);
  while (1) {
    if (state == 0) {
      if (millis() - timer > period) {
        dog.setFromCentroid(r, Point(0, 0, 0), Frame::BODY, period);
        timer = millis();
        state = 1;
      }
    } else if (state == 1) {
      if (millis() - timer > period * 1.5) {
        Serial.print("First Rotation: "); dog.printLeg(0);
        Serial.println("Expected: (X: -14.93, Y:64.29, Z:-100.00)"); // Assuming rot(0,0,30);
        dog.printIMU();
        Serial.println("Expected: (0,0,30)");
        Serial.println("Waiting");
        timer = millis();
        state = 2;
      }
    } else if (state == 2) {
      if (millis() - timer > period) {
        Serial.println("Back Home");
        dog.setFromCentroid(ROT_ZERO, Point(0, 0, 0), Frame::BODY, period);
        timer = millis();
        state = 3;
      }
    } else if (state == 3) {
      if (millis() - timer > period*1.5) {
        Serial.print("Second Rotation: "); dog.printLeg(0);
        Serial.println("Expected: (X: 0.00, Y:0.00, Z:-100.00)");
        dog.printIMU();
        Serial.println("Expected: (0,0,0)");
        state = 4;
      }
    }

    dog.operate();
  }
}

// Check if ground frame estimates are accurate
void printLegs_IMU() {
  dog.useIMU(true);
  while(1) {
    dog.operate();
    dog.printIMU();
    dog.printLegsFromBody();
    delay(10);
  }
}

void testCentroidCalculation() {
  
}

void testCentroidMovement() {
  
}

void testRelativeMovement() {
  
}




// Sweep z while using IMU
void sweep_z_IMU(Rot r, double min, double max) {
  dog.useIMU(true);
  unsigned int timer = millis();
  int state = 0;
  unsigned int period = 1000;
  while (1) {
    if (state == 0) {
      if (millis() - timer > period*1.1) {
        dog.setFromCentroid(r, Point(0, 0, max), Frame::BODY, period);
        timer = millis();
        state = 1;
      }
    } else {
      if (millis() - timer > period*1.1) {
        dog.setFromCentroid(r, Point(0, 0, min), Frame::BODY, period);
        timer = millis();
        state = 0;
      }
    }

    dog.operate();
    //dog.printLegs();
  }
}

void sweep_z(Rot r, double min, double max) {
  dog.useIMU(false);
  unsigned int timer = millis();
  int state = 0;
  unsigned int period = 1000;
  while (1) {
    if (state == 0) {
      if (millis() - timer > period*1.1) {
        dog.setFromCentroid(r, Point(0, 0, max), Frame::BODY, period);
        timer = millis();
        state = 1;
      }
    } else {
      if (millis() - timer > period*1.1) {
        dog.setOrientation(r);
        dog.setFromCentroid(r, Point(0, 0, min), Frame::BODY, period);
        timer = millis();
        state = 0;
      }
    }

    dog.operate();
    //dog.printLegs();
  }
}



void sweep_x(Rot r, double min, double max) {
  dog.useIMU(false);
  unsigned int timer = millis();
  int state = 0;
  unsigned int period = 1000;
  while (1) {
    if (state == 0) {
      if (millis() - timer > period*1.1) {
        dog.setFromCentroid(r, Point(max, 0, 0), Frame::GROUND, period);
        timer = millis();
        state = 1;
      }
    } else {
      if (millis() - timer > period*1.1) {
        dog.setOrientation(r);
        dog.setFromCentroid(r, Point(min, 0, 0), Frame::GROUND, period);
        timer = millis();
        state = 0;
      }
    }

    dog.operate();
    //dog.printLegs();
  }
}

void sweep_y(double min, double max) {
  int timer = millis();
  double speed = 30; // mm/s
  double position = 0;
  double dt = 0;

  while (1) {
    dt = millis() - timer;
    timer = millis();
    double last_position = position;
    position += speed/1000.0 * dt;
    Serial.println(position);
    Serial.print("centroid "); dog.getCentroid().print();
//    dog.setFromCentroid(Point(0, position, 0), Frame::GROUND);
    if ((position > max && last_position < max) || (position < min && last_position > min)) {
      speed *= -1;
    }

    dog.operate();
    dog.printLegs();
  }
}

//void sweep_z(double min, double max) {
//  int timer = millis();
//  double speed = 40; // mm/s
//  double position = 0;
//  double dt = 0;
//
//  while (1) {
//    dt = millis() - timer;
//    timer = millis();
//    double last_position = position;
//    position += speed/1000.0 * dt;
//    Serial.println(position);
//    Serial.print("centroid "); dog.getCentroid().print();
//    dog.setFromCentroid(Point(0, 0, position), Frame::GROUND);
//    if ((position > max && last_position < max) || (position < min && last_position > min)) {
//      speed *= -1;
//    }
//
//    dog.operate();
//    dog.printLegs();
//  }
//}


void loop() {
  dog.operate();
}
