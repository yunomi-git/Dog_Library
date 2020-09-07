#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <EEPROM.h>

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
*/

/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS (100)

Adafruit_BNO055 bno = Adafruit_BNO055(54);

/**************************************************************************/
/*
    Display sensor calibration status
*/
/**************************************************************************/
void displayCalStatus(void)
{
  /* Get the four calibration values (0..3) */
  /* Any sensor data reporting 0 should be ignored, */
  /* 3 means 'fully calibrated" */
  uint8_t system, gyro, accel, mag;
  system = gyro = accel = mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);

  /* The data should be ignored until the system calibration is > 0 */
  Serial.print("\t");
  if (!system)
  {
    Serial.print("! ");
  }

  /* Display the individual values */
  Serial.print("Sys:");
  Serial.print(system, DEC);
  Serial.print(" G:");
  Serial.print(gyro, DEC);
  Serial.print(" A:");
  Serial.print(accel, DEC);
  Serial.print(" M:");
  Serial.print(mag, DEC);
}

/**************************************************************************/
/*
    Arduino setup function (automatically called at startup)
*/
/**************************************************************************/
void setup(void)
{
  Serial.begin(9600);
  Serial.println("Orientation Sensor Test"); Serial.println("");

  /* Initialise the sensor, no magnetometoer */
  if(!bno.begin(Adafruit_BNO055::OPERATION_MODE_IMUPLUS))
  {
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }

  bno.setAxisRemap(Adafruit_BNO055::REMAP_CONFIG_P3);
  bno.setAxisSign(Adafruit_BNO055::REMAP_SIGN_P3);

  /*
  *  Load Calibration data from EEPROM
  */
  int eeAddress = 0;
  long bnoID;

  EEPROM.get(eeAddress, bnoID);

  adafruit_bno055_offsets_t calibrationData;
  sensor_t sensor;

  bno.getSensor(&sensor);
  if (bnoID != sensor.sensor_id)
  {
      Serial.println("\nNo Calibration Data for this sensor exists in EEPROM");
      delay(500);
  }
  else
  {
      Serial.println("\nFound Calibration for this sensor in EEPROM.");
      eeAddress += sizeof(long);
      EEPROM.get(eeAddress, calibrationData);
      calibrationData.accel_offset_z -= -35;
      Serial.println("\n\nRestoring Calibration data to the BNO055...");
      bno.setSensorOffsets(calibrationData);

      Serial.println("\n\nCalibration data loaded into BNO055");
  }
  
  bno.setExtCrystalUse(true);

  delay(1500);
}

/**************************************************************************/
/*
    Arduino loop function, called once 'setup' is complete (your own code
    should go here)
*/
/**************************************************************************/
void loop(void)
{
  /* Get a new sensor event */
//  sensors_event_t event;
//  bno.getEvent(&event);

//// getsensor for acceleration
//  imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
//  
//  /* Display the floating point data */
//  Serial.print("X: ");
//  Serial.print(accel.x(), 4);Serial.print(","); 
//  Serial.print("\tY: ");
//  Serial.print(accel.y(), 4);Serial.print(","); 
//  Serial.print("\tZ: ");
//  Serial.print(accel.z(), 4);



//  /* Display the floating point data */
//  Serial.print("X: ");
//  Serial.print(event.acceleration.x, 4);
//  Serial.print("\tY: ");
//  Serial.print(event.acceleration.y, 4);
//  Serial.print("\tZ: ");
//  Serial.print(event.acceleration.z, 4);

// Quaternions for orientation
  imu::Quaternion q = bno.getQuat();
  q.normalize();
//  float temp = q.x();  q.x() = -q.y();  q.y() = temp;
//  q.z() = -q.z();
  imu::Vector<3> euler = q.toEuler();
  euler.toDegrees();

  Serial.print("X: ");
  Serial.print(euler.x(), 4);
  Serial.print("\tY: ");
  Serial.print(euler.y(), 4);
  Serial.print("\tZ: ");
  Serial.print(euler.z(), 4);

  /* New line for the next sample */
  Serial.println("");

  /* Wait the specified delay before requesting nex data */
  delay(5);
}
