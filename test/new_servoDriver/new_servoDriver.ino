#include <RServoDriver.h>

RServoDriver driver;
int sig;
//float sig_table_1[19] = {-90, -80, -70, -60, -50, -40, -30, -20, -10, 0, 10, 20, 30, 40, 50, 60, 70, 80, 90};
float angle_min = -90;
float angle_max = 90;
int sig_min = -900;
int sig_max = 900;


float sig_table_2[19] = {-900, -800, -700, -600, -500, -400, -300, -200, -100, 0, 100, 200, 300, 400, 500, 600, 700, 800, 900};
// ^ times 10 for increased resolution, corresponds to angles from -90 to 90


void setup() {
  Serial.begin(9600);
  while ( !Serial ) ;
  Serial.println("Test Beginning");

  // put your setup code here, to run once:
  driver.defaultStartup();
//  driver.reverseDirection(0);
//  driver.setOffset(0, 45);
//  driver.gotoAngle(0, 0); // send channel 0 to 0 deg

  Serial.println("-----No table setup-----");
  Serial.println("Input angle: 0");
  Serial.print("Obtained Signal: "); Serial.println(driver.getServo(0)->angleToPwm(0));
  Serial.println("Expected Signal: 2455");



  Serial.println("-----Table setup-----");
  driver.getServo(0)->setTable(sig_table_2); // set up the table
  int num_test = 5;
  float test_angles[num_test] = {0, 5, -5, -87.12, 87.12}; // angles to test
  float input_angle;
  for (int i = 0; i < num_test; i++) {
    input_angle = test_angles[i];
    Serial.print("Input angle: "); Serial.println(input_angle);
    Serial.print("Obtained Signal: "); Serial.println(driver.getServo(0)->angleToPwm(input_angle));
    Serial.print("Expected Signal: "); Serial.println((int) (input_angle*10));
    Serial.println("--");
  }



  Serial.println("-----Transformations: check if table and no table output same values-----");
  driver.getServo(1)->setAngles(angle_min, angle_max, sig_min, sig_max);
  Serial.println("--- 1. Default ---");
  num_test = 5;
  float test_angles2[num_test] = {0, 5, -5, -87.12, 87.12}; // angles to test
  for (int i = 0; i < num_test; i++) {
    input_angle = test_angles2[i];
    Serial.print("Input angle: "); Serial.println(input_angle);
    Serial.print("Table Signal: "); Serial.println(driver.getServo(0)->angleToPwm(input_angle));
    Serial.print("No Table Signal: "); Serial.println(driver.getServo(1)->angleToPwm(input_angle));
    Serial.print("Expected Signal: "); Serial.println((int) (input_angle*10));
    Serial.println("--");
  }
  
  Serial.println("--- 2. Shift +45, flip ---");
  driver.getServo(0)->reverseDirection();
  driver.getServo(0)->setOffset(45);
  driver.getServo(1)->reverseDirection();
  driver.getServo(1)->setOffset(45);
  num_test = 5;
  float test_angles3[num_test] = {0, 5, -5, 45, 87.12}; // angles to test
  for (int i = 0; i < num_test; i++) {
    input_angle = test_angles3[i];
    Serial.print("Input angle: "); Serial.println(input_angle);
    Serial.print("Table Signal: "); Serial.println(driver.getServo(0)->angleToPwm(input_angle));
    Serial.print("No Table Signal: "); Serial.println(driver.getServo(1)->angleToPwm(input_angle));
    Serial.print("Expected Signal: "); Serial.println((int) (-(input_angle-45)*10));
    Serial.println("--");
  }


  
  
}

void loop() {
  // put your main code here, to run repeatedly:
  if (Serial.available()) {
    sig = Serial.parseInt();
    if (sig != 0) {
      if (sig == 100)
        sig = 0;
       driver.gotoAngle(0, sig);
       Serial.println(sig);
    }
  }
}
