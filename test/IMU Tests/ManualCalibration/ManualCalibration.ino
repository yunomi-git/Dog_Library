#include "IMU.h"

IMU bno_imu(IMU::CONTINUOUS);

/* This driver uses the Adafruit unified sensor library (Adafruit_Sensor),
   which provides a common 'type' for sensor data and some helper functions.

   To use this driver you will also need to download the Adafruit_Sensor
   library and include it in your libraries folder.

   You should also assign a unique ID to this sensor for use with
   the Adafruit Sensor API so that you can identify this particular
   sensor in any data logs, etc.  To assign a unique ID, simply
   provide an appropriate value in the constructor below (12345
   is used by default in this example).

   Connections
   ===========
   Connect SCL to analog 5
   Connect SDA to analog 4
   Connect VDD to 3-5V DC
   Connect GROUND to common ground

   History
   =======
   2015/MAR/03  - First release (KTOWN)
   2015/AUG/27  - Added calibration and system status helpers
   2015/NOV/13  - Added calibration save and restore
   */


/**************************************************************************/
/*
    Arduino setup function (automatically called at startup)
    */
/**************************************************************************/
void setup(void)
{
    Serial.begin(115200);
    delay(1000);
    Serial.println("Orientation Sensor Test"); Serial.println("");
  bno_imu.defaultStartup();
  bno_imu.tareOrientation();
}

void loop() {
  bno_imu.operate();

    printVelocity();

    /* Wait the specified delay before requesting new data */
    delay(5);
}

void printVelocity() {
  Point vel = bno_imu.velocity;

  Serial.print("X: ");
  Serial.print(vel.x, 4);
  Serial.print("\tY: ");
  Serial.print(vel.y, 4);
  Serial.print("\tZ: ");
  Serial.print(vel.z, 4);
  Serial.println();
}
