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

// Calculate the tilt angle theta
float imu::cal_theta() {
    imu_dev->getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    cout << ax << "\t" << ay << "\t" << az << "\t" << gx << "\t" << gy << "\t" << gz << endl;
    return 1.0;
}



