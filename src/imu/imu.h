//
// Class for IMU
//

#ifndef IMU_H_
#define IMU_H_

#include "bcm2835/bcm2835.h"
#include "imu/MPU6050.h"
#include <time.h>

using namespace std;

class imu {

public:

    MPU6050* imu_dev;

    // IMU readings. Acceleration and gyro.
    int16_t ax, ay, az;
    int16_t gx, gy, gz;

    // Timestamps
    float delta_ts, cur_ts, prev_ts;
    // First sample flag - used to init timestamps
    bool isFirstSample;

    // Constructor
    imu();
    // Destructor
    ~imu();

    // Calibrate static gyro reading on Y axis
    int16_t gyroY_static;
    void calibrate();

    // Calculate the timestamps
    void cal_ts();

    // Calculate the tilt angle theta
    float prev_theta, theta;
    float cal_theta();

};

#endif // IMU_H_
