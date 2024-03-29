#ifndef __IMU__
#define __IMU__

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <EEPROM.h>

#include "Rot.h"
#include "Point.h"


struct IMU {
    #define HISTORY_SIZE 15
    
    enum collection_mode {SINGLE, CONTINUOUS};

    Adafruit_BNO055 bno = Adafruit_BNO055(54);
    Rot orientation_offsets; // BNO does not tare orientation
    collection_mode cmode;

    int IMU_z_num_turns = 0;
    Rot raw_orientation;

    // Point orientation_signs;  // For customization of axes
    template<typename T> struct sensor_history_info {
        T value;
        T value_history[HISTORY_SIZE];
        int history_size;
        int index = 0;

        sensor_history_info() {history_size = HISTORY_SIZE;}

        sensor_history_info(int size) {history_size = size;}

        void updateHistory(T new_value) {
            value = value + (new_value - value_history[index])/(history_size);
            value_history[index] = new_value;
            index = (index + 1)%history_size;
        }

        T getPrevHistoryValue(int di) {
            return value_history[(index - di + history_size) % history_size];
        }
    };

    // sensor_history_info<Point> lin_accel_info;
    // sensor_history_info<Point> gravity_info;
    sensor_history_info<Rot> orientation_info{1};
    sensor_history_info<Rot> gyro_info{10};

public:
    IMU() {cmode = SINGLE;}

    void setCollectionMode(collection_mode mode) {cmode = mode;}

    void defaultStartup() {
        /* Initialise the sensor, no magnetometoer */
        if(!bno.begin(Adafruit_BNO055::OPERATION_MODE_IMUPLUS))
        {
            Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
            while(1);
        }

        // Set reference frame...y must be swapped for consistency with dog
        //   ^ +z (up)
        //   |
        //   x --> +x (front)
        //   +y (left)
        bno.setAxisRemap(Adafruit_BNO055::REMAP_CONFIG_P3);
        bno.setAxisSign(Adafruit_BNO055::REMAP_SIGN_P3);

        // Load Calibration data from EEPROM
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
        } else {
            eeAddress += sizeof(long);
            EEPROM.get(eeAddress, calibrationData);
            calibrationData.accel_offset_x = 11; 
            calibrationData.accel_offset_y = 10; 
            calibrationData.accel_offset_z = -36; // It's off for some reason
            bno.setSensorOffsets(calibrationData);
        }
        
        // Crystal
        bno.setExtCrystalUse(true);

        raw_orientation = ROT_ZERO;
    }

    void tareOrientation(int n = 10) {
        Rot avg_offsets;

        for (int i = 0; i < n; i++) {
            avg_offsets += (getRawOrientation() + orientation_offsets)/n; // Averages raw orientation values
        }
        orientation_offsets = avg_offsets;
    }

// Returns processed sensor data
    Rot getOrientation() {
        if (cmode == CONTINUOUS) {
            return orientation_info.value;
        } else {
            return raw_orientation;
        }
    }

    Rot getRotVelocity() {
        if (cmode == CONTINUOUS) {
            return gyro_info.value;
        } else {
            return getRawRotVelocity();
        }
    }

// Directly reads sensor data and formats it conveniently
    Rot getRawOrientation() {
        // Quaternion for data stability
        imu::Quaternion q = bno.getQuat();
        q.normalize();

        // Conversion
        imu::Vector<3> euler = q.toEuler();
        euler.toDegrees();

        return Rot(-euler.z(), -euler.y(), euler.x()) - orientation_offsets; // Technically not correct? Rotation matrices may not concatenate as subtraction...but accurate enough
    }


    Rot getRawRotVelocity() {
        imu::Vector<3> v = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);

        return Rot(-v.z(), -v.y(), v.x());
    }

// Processes sensor data
    void operate() {
        // Continuous Z rotation
        raw_orientation = getRawOrientation();

        // the "measured" yaw value
        float IMU_z = raw_orientation.z;
        float IMU_z_prev = orientation_info.getPrevHistoryValue(1).z;

        IMU_z += IMU_z_num_turns * 360;
        // Makes the IMU loop instead of just staying resetting
        if ((IMU_z - IMU_z_prev) >= 350) {
            IMU_z -= 360;
            IMU_z_num_turns--;
        } else if ((IMU_z_prev - IMU_z) >= 350) {
            IMU_z += 360;
            IMU_z_num_turns++;
        }
        raw_orientation.z = IMU_z;
        

        orientation_info.updateHistory(raw_orientation);
//        lin_accel_info.updateHistory(getRawLinearAcceleration());
//        gravity_info.updateHistory(getRawGravity());
        gyro_info.updateHistory(getRawRotVelocity());
    }
};

#endif