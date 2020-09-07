#include <Dog.h>
#include <RServoDriver.h>
#include "SoftwareSerial.h"
#include <math2.h>
//#define DEBUG
// Communication
// rx to pin9, tx to pin10
#define XBee Serial2

double joystick[4] = {0,0,0,0}; // RY, RX, LY, LX
int mode = -1;

int sig;
//RServoDriver servo_driver;
RobotDog dog;
double timer;

double kp = 1;
double kd = 0.1;
double ki = 0.001;

int i = 1;
double period = 6;

double perr[3] = {0,0,0};
double derr[3] = {0,0,0};
double ierr[3] = {0,0,0};

void setup() {
  Serial.begin(9600);
  XBee.begin(19200);
  pinMode(13, OUTPUT);
  timer = millis();
//  servo_driver.defaultStartup();
//  dog = RobotDog(&servo_driver);
  dog.begin();
  dog.sendAllSignals();

  Serial.println("Starting IMU");
  delay(1000);
  IMU_setup();
  Serial.println("IMU Successful");
  Serial.println("starting");
  delay(1000);
  Serial.println("-----------------");
//  dog.moveByPosition(0,0,0);

  Serial.println("Start Loop");
}

void loop() {
  // IMU
  IMU_operate();
  // Do actions
  //Serial.print(IMU_getAngY()); Serial.print(" | "); 
  //Serial.print(IMU_getAngZ()); Serial.print(" | ");
  //Serial.println(-IMU_getAngX());
 //dog.moveToPose(0, 0, 30*sin((millis() - timer)/1000*2*M_PI/period), 0, 0, 0);  
 Serial.println(ierr[2]);

  double dt = timer - millis();
  timer = millis();
  double err[3] = {-IMU_getAngY(), -IMU_getAngZ(), IMU_getAngX()};
  for (int i = 0; i < 3; i++) {
    derr[i] = (err[i] - perr[i])/dt;
    perr[i] = err[i];
    ierr[i] = ierr[i] - err[i]*dt; // Why is this -??
  }
  dog.moveToPose(kp*perr[0] + kd*derr[0] + ki*ierr[0], kp*perr[1] + kd*derr[1] + ki*ierr[1], kp*perr[2] + kd*derr[2] + ki*ierr[2], 0, 0, 0);  
  
  //dog.moveToPose(-k*IMU_getAngY(), -k*IMU_getAngZ(), -k*-IMU_getAngX(), 0, 0, 0);  

}
