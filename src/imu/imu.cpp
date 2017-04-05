// imu.cpp

#include "imu/imu.h"
#include <iostream>

// Constructor
imu::imu() {
    I2Cdev::initialize();
    imu_dev = new MPU6050;

    if (imu_dev->testConnection())
        cout << "MPU6050 connection test successful. " << endl;
    else
        cerr << "MPU6050 connect test failed! Exit. " << endl;

    imu_dev->initialize();

    isFirstSample = true;
}

// Destructor
imu::~imu() {}

// Average out measurements of GyroY to get static offset at start position.
void imu::calibrate() {

    int GYROY_INIT_MEASURE = 50;

    for (int i=0;i<GYROY_INIT_MEASURE;i++) {
        gyroY_static += imu_dev->getRotationY();
        delay(10);
    }

    gyroY_static = gyroY_static/GYROY_INIT_MEASURE;
    cout << "GyroY static calibration done. GyroY_static = " << gyroY_static << "." << endl;
}

// Calculate timestamps
void imu::cal_ts() {
    struct timespec cur_time;
    // clock_gettime from http://yosh.ke.mu/raspberry_pi_getting_time
    clock_gettime(CLOCK_MONOTONIC, &cur_time); 

    //cout << cur_time.tv_sec  << " "  << cur_time.tv_nsec << endl;
    cur_ts = (float)cur_time.tv_sec + ((float)cur_time.tv_nsec)/1000000000.0;
    cout.precision(14);
    //cout << "Timestamp = " << cur_ts << endl;

    if (isFirstSample) {
        delta_ts = 0.0;
    } else {
        delta_ts = cur_ts - prev_ts;
    }
    prev_ts = cur_ts;
    cout << "Delta time = " << delta_ts << endl;
}

// Calculate the tilt angle theta
float imu::cal_theta() {
    // IMU readings
    imu_dev->getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    cout << ax << "\t" << ay << "\t" << az << "\t" << gx << "\t" << gy << "\t" << gz << endl;

    // Timestamp
    cal_ts();

    // Calculate acc theta
    float r = sqrt((float) az * (float) az + (float) ax * (float) ax);
    float acc_theta = (float) az / r; //approximates sine
    cout << "acc_theta = " << acc_theta << endl;

    if (isFirstSample) {
        theta = acc_theta;
        prev_theta = acc_theta;
        isFirstSample = false;
    } else {
        float degY = ((float) gy - (float) gyroY_static) * delta_ts / 131.0;
        cout << "degY = " << degY << endl;
        theta = 0.95 * (prev_theta + degY) + 0.05 * acc_theta;
        cout << "theta = " << theta << endl;
        prev_theta = theta;
    }

    cout << "degrees = " << theta*180.0/3.1415926 << endl;
    cout << endl;
    return theta;
}



