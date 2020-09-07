#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#define BNO055_SAMPLERATE_DELAY_MS (10)

Adafruit_BNO055 bno = Adafruit_BNO055();

#define RST_PIN 7

int IMU_x_num_turns = 0;
double IMU_x = 0;
double IMU_x_prev = 0;
double error = 0;

int IMU_y_num_turns = 0;
double IMU_y = 0;
double IMU_y_prev = 0;
double IMU_y_offset = 0;

int IMU_z_num_turns = 0;
double IMU_z = 0;
double IMU_z_prev = 0;
double IMU_z_offset = 0;



void IMU_setup() {
  Serial.println("Orientation Sensor Raw Data Test"); Serial.println("");
  pinMode(RST_PIN, OUTPUT);
  digitalWrite(RST_PIN, HIGH);
  /* Initialise the sensor */
  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
    delay(1000);
  }


  bno.setExtCrystalUse(true);

  Serial.println("Calibration status values: 0=uncalibrated, 3=fully calibrated");
  //delay(1000);

  // Initialize
  delay(1000);
  sensors_event_t event;
  bno.getEvent(&event);
  IMU_y_offset = event.orientation.y;
  IMU_z_offset = event.orientation.z;
  Serial.println("offsets");
  Serial.println(IMU_y_offset);
  Serial.println(IMU_z_offset);
}



// Goes into loop
void IMU_operate() {
  sensors_event_t event;
  bno.getEvent(&event);
  //Serial.print(event.orientation.x);


    
  IMU_y = event.orientation.y - IMU_y_offset;
  IMU_z = event.orientation.z - IMU_z_offset;

  // x, the "measured" value
  IMU_x = event.orientation.x + IMU_x_num_turns * 360 + error;
  // Makes the IMU loop instead of just staying resetting
  if ((IMU_x - IMU_x_prev) >= 350) {
    IMU_x -= 360;
    IMU_x_num_turns--;
  } else if ((IMU_x_prev - IMU_x) >= 350) {
    IMU_x += 360;
    IMU_x_num_turns++;
  }

  // x, account for errors
  if (fabs(IMU_x - IMU_x_prev) > 60) {
    Serial.println("++++++++++++++++++++++++++++++++++++++++++");
    IMU_x -= error;
    error += IMU_x_prev - IMU_x;
    IMU_x += error;
  }
  IMU_x_prev = IMU_x;
}

// ====================================================
// =============== EXTERNAL FUNCTIONS ==================
// ====== functions that are called by other INOs ======
// =====================================================

double IMU_getAngX() {
  return IMU_x;
}

double IMU_getAngY() {
  return IMU_y;
}

double IMU_getAngZ() {
  return IMU_z;
}

void IMU_reset() {
  digitalWrite(RST_PIN, LOW);
  delayMicroseconds(30);
  digitalWrite(RST_PIN, HIGH);
  IMU_setup();
}

void IMU_print() {
  Serial.print("X: ");
  Serial.println(IMU_x);
}
