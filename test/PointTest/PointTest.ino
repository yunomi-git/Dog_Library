#include <Point.h>

void setup() {
  Serial.begin(9600);
  delay(1000);
  Serial.println("begin");
  // Test comparison

  // put your setup code here, to run once:

  // Test Rotation
  test_rotation_inversion();
  
}

void test_rotation() {
  Point p = Point(1,2,3);
  Rot r = Rot(30, 20, 5);
  Serial.print("Original: "); p.print();
  Serial.print("Rotated: "); (p*r).print();
  Serial.print("Rotated Back: "); ((p*r)/r).print();

  p = Point(1,0,0);
  r = Rot(0,0,30);
  Serial.print("Original: "); p.print();
  Serial.print("Rotated: "); (p*r).print();
}

void test_rotation_inversion() {
  Point p = Point(1,2,3);
  Rot r = Rot(30, 20, 5);
  Serial.print("Original: "); p.print();
  Serial.print("Rotated -1: "); (p/r).print();
  Serial.print("Rotated inverse: "); ((p*(-r))).print();
}

void test_boolean() {
  Point p = Point(1, 2, 3);
  Point p2 = Point(1,2,3);
  Serial.println(p != p2);
  Serial.println(p != Point(1,2,3));
  Serial.println(p != Point(1,2,2));
  Serial.println(p != POINT_ZERO);
  // Test explicit boolean conversion
  if (p)
    Serial.println("p is not 0");
  else
    Serial.println("p is 0");

  if (POINT_ZERO)
    Serial.println("p0 is not 0");
  else
    Serial.println("p0 is 0");
}

void loop() {
  // put your main code here, to run repeatedly:

}
