#include <Leg.h>
RServoDriver servo_driver;
int sig;
DogLeg leg;
bool error;
Point mounting_point = Point(112.5, 60, 0);
double timer;
Rot orientation = ROT_ZERO;

#define DEFAULT_LENGTH 90

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  servo_driver.defaultStartup();
  leg = DogLeg(&servo_driver, 0, 1, 2, false, false, &orientation, mounting_point, DEFAULT_LENGTH);
  leg.calibrateServos(4,-2,9);
  delay(1000);
//  Serial.println(leg.getCurrentFootPositionFromShoulder().z);
  Serial.println("to home");
  leg.sendSignals();
//  Serial.println(leg.getCurrentFootPositionFromShoulder().z);
  delay(1000);
  
//  // Test Instant Move
//  Serial.println("to -140");
//  leg.setToPositionFromShoulder(Point(0,0,-140));
//  leg.operate();
//  leg.sendSignals();

//  test_rotated_frame();

  // Test Moveby
  test_body_move();
}

void loop() {
  if (millis() - timer > 5) {
    leg.operate();
    leg.sendSignals();
    timer = millis();
  }
}

void test_body_move() {
  orientation = Rot(0, 0, 30);
  Serial.println("Testing move_from_body. All values printed in body frame");
  Serial.println("Expected Start: (X:0, Y:0, Z:-90)");
  leg.setToPositionFromShoulder(Point(0,0,-DEFAULT_LENGTH), Frame::BODY);
  leg.operate();
  leg.sendSignals();
  leg.getCurrentFootPositionFromShoulder(Frame::BODY).print();
  Serial.println();
  delay(1000);
  
  Serial.println("Move in/out in body frame");
  Serial.println("Expected: (X:40.00, Y0.00, Z:-90)");
  leg.setToPositionFromBody(mounting_point + Point(0, 0, -DEFAULT_LENGTH) + Point(40,0,0), Frame::BODY);
  leg.operate();
  leg.sendSignals();
  leg.getCurrentFootPositionFromShoulder(Frame::BODY).print();
  Serial.println();
  delay(1000);

  Serial.println("Expected: (X:0.00, Y:0.00, Z:-90)");
  leg.setToPositionFromBody(mounting_point + Point(0, 0, -DEFAULT_LENGTH), Frame::BODY);
  leg.operate();
  leg.sendSignals();
  leg.getCurrentFootPositionFromShoulder(Frame::BODY).print();
  Serial.println();
  delay(1000);
  
  Serial.println("Move in ground frame, +30 degree yaw");
  Serial.println("Expected: (X:-45.07, Y:48.21, Z:-90)");
  leg.setToPositionFromBody(mounting_point + Point(0, 0, -DEFAULT_LENGTH), Frame::GROUND);
  leg.operate();
  leg.sendSignals();
  leg.getCurrentFootPositionFromShoulder(Frame::BODY).print();
  Serial.println();
  delay(1000);

  Serial.println("Expected: (X:0.00, Y:0.00, Z:-90)");
  leg.setToPositionFromBody(Point(127.43, -4.29, -DEFAULT_LENGTH), Frame::GROUND);
  leg.operate();
  leg.sendSignals();
  leg.getCurrentFootPositionFromShoulder(Frame::BODY).print();
}

void test_setFrom() {
  orientation = Rot(0, 0, 30);
  Serial.println("Testing setFrom");
  leg.setToPositionFromShoulder(Point(0,0,-DEFAULT_LENGTH), Frame::GROUND);
  leg.operate();
  leg.sendSignals();
  delay(1000);
  Serial.println("Move in/out in body frame");
  leg.setFromPosition(Point(40,0,0), Frame::BODY);
  leg.operate();
  leg.sendSignals();
  leg.getCurrentFootPositionFromShoulder(Frame::BODY).print();
  Serial.println();
  delay(1000);
  
  leg.setFromPosition(Point(-40,0,0), Frame::BODY);
  leg.operate();
  leg.sendSignals();
  leg.getCurrentFootPositionFromShoulder(Frame::BODY).print();
  Serial.println();
  delay(1000);
  
  Serial.println("Move in/out in ground frame");
  leg.setFromPosition(Point(40,0,0), Frame::GROUND);
  leg.operate();
  leg.sendSignals();
  leg.getCurrentFootPositionFromShoulder(Frame::BODY).print();
  Serial.println();
  delay(1000);
  
  leg.setFromPosition(Point(-40,0,0), Frame::GROUND);
  leg.operate();
  leg.sendSignals();
  leg.getCurrentFootPositionFromShoulder(Frame::BODY).print();

}

void test_rotated_frame() {
    // Test rotated reference frame
    Serial.println("Orientation 0");
    orientation = ROT_ZERO;
    leg.setToPositionFromShoulder(Point(0,40,-70), Frame::GROUND);
    leg.operate();
    leg.sendSignals();
    delay(1000);
    leg.setToPositionFromShoulder(Point(0,0,-70), Frame::GROUND);
    leg.operate();
    leg.sendSignals();
    
    Serial.println("Orientation 30");
    orientation = Rot(0, 0, 30);
    delay(1000);
    leg.setToPositionFromShoulder(Point(0,40,-70), Frame::GROUND);
    leg.operate();
    leg.sendSignals();
}
