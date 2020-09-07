/* Input-side (button) Arduino code */
#include "SoftwareSerial.h"
// RX: Arduino pin 2, XBee pin DOUT.  TX:  Arduino pin 3, XBee pin DIN
SoftwareSerial XBee(2, 3);
int mode = 1;
void setup()
{
  Serial.begin(9600);
  // Baud rate MUST match XBee settings (as set in XCTU)
  XBee.begin(19200);
  pinMode(13, OUTPUT);
}

double JRX = 0;
double JRY = 0;
double JLX = 0;
double JLY = 0;
double joystick[4];

void loop()
{
    if (XBee.available()) {
// First revieve joystick values
for (int i = 0; i < 4; i++) {
  joystick[i] = XBee.parseFloat();
  Serial.print(joystick[i]), Serial.print(" ");
}
Serial.println();
      String s = XBee.readStringUntil('\n');
      Serial.println();
      Serial.println(s);
      if (s.indexOf("B1") != -1) {
          mode *= -1;
          if (mode == 1) {
              digitalWrite(13, HIGH);
          }
          else {
              digitalWrite(13, LOW);
          }
          Serial.println(mode);
      }

    }
}
