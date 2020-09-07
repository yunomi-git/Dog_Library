#include "IMU.h"

IMU bno_imu;

void setup() {
  Serial.begin(9600);
  delay(1000);
  // put your setup code here, to run once:
  bno_imu.defaultStartup();
  bno_imu.setCollectionMode(IMU::CONTINUOUS);
  bno_imu.tareOrientation();
}

void loop() {
  bno_imu.operate();
 //Print Orientations

// Print Accelerations
//  Point accel = bno_imu.getLinearAcceleration();
//
//  Serial.print("X: ");
//  Serial.print(accel.x, 4);
//  Serial.print("\tY: ");
//  Serial.print(accel.y, 4);
//  Serial.print("\tZ: ");
//  Serial.print(accel.z, 4);
//  Serial.println();
  
//  printPosition();
  printOrientationHistory();

  delay(5);
}

void printOrientationHistory() {
  Serial.print("Raw: ");
  Serial.print((bno_imu.getRawOrientation()).z);
  Serial.print(", Averaged: ");
  Serial.print((bno_imu.orientation_info.value).z);
  Serial.print(", History: ");
  for (int i = 0; i < bno_imu.orientation_info.history_size; i++) {
    Serial.print((bno_imu.orientation_info.getPrevHistoryValue(i)).z);
    Serial.print(", ");
  }
  Serial.println();
}

void printRawSmoothOrientation() {
    Serial.print("Raw: ");
  Serial.print((bno_imu.getRawOrientation()).z);
  Serial.print(", Averaged: ");
  Serial.print((bno_imu.orientation_info.value).z);
  Serial.println();
}

void printRawSmoothLinAccel() {
    Serial.print("Raw: ");
  Serial.print((bno_imu.getRawLinearAcceleration()).x);
  Serial.print(", Averaged: ");
  Serial.print((bno_imu.lin_accel_info.value).x);
  Serial.println();
}

void printOrientation() {
  Rot r = bno_imu.getOrientation();

  Serial.print("X: ");
  Serial.print(r.x, 4);
  Serial.print("\tY: ");
  Serial.print(r.y, 4);
  Serial.print("\tZ: ");
  Serial.print(r.z, 4);
  Serial.println();
}
//
//void printVelocity() {
//  Point vel = bno_imu.velocity;
//
//  Serial.print("X: ");
//  Serial.print(vel.x, 4);
//  Serial.print("\tY: ");
//  Serial.print(vel.y, 4);
//  Serial.print("\tZ: ");
//  Serial.print(vel.z, 4);
//  Serial.println();
//}
//
//void printPosition() {
//  Point pos = bno_imu.position;
//
//  Serial.print("X: ");
//  Serial.print(pos.x, 4);
//  Serial.print("\tY: ");
//  Serial.print(pos.y, 4);
//  Serial.print("\tZ: ");
//  Serial.print(pos.z, 4);
//  Serial.println();
//}
