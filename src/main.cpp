// main.c
 
#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>
#include <stdlib.h>
#include <stdio.h>

#include "bcm2835/bcm2835.h"
#include "motor/motor.h"
#include "I2Cdev/I2Cdev.h"
#include "imu/MPU6050.h"
 
using namespace std;

int main(int argc, char **argv)
{
    // GPIO pin assignment
    // Test pin 
    //RPiGPIOPin PIN = RPI_V2_GPIO_P1_37;
    // Motor A
    RPiGPIOPin PWMA = RPI_V2_GPIO_P1_11;
    RPiGPIOPin AIN2 = RPI_V2_GPIO_P1_13;
    RPiGPIOPin AIN1 = RPI_V2_GPIO_P1_15;
    // Motor B
    RPiGPIOPin PWMB = RPI_V2_GPIO_P1_29;
    RPiGPIOPin BIN2 = RPI_V2_GPIO_P1_31;
    RPiGPIOPin BIN1 = RPI_V2_GPIO_P1_33;

    // If you call this, it will not actually access the GPIO
    // Use for testing
    //bcm2835_set_debug(1);
 
    if (!bcm2835_init())
	return 1;
 
    motor motor_a, motor_b;

    // motors init
    motor_a.init(PWMA, AIN1, AIN2);
    motor_b.init(PWMB, BIN1, BIN2);

    // IMU init
    int16_t ax, ay, az;
    int16_t gx, gy, gz;
    I2Cdev::initialize();
    MPU6050 imu ;
    if ( imu.testConnection() ) 
      cout << "MPU6050 connection test successful\n";
    else {
      cerr << "MPU6050 connection test failed! Exit.\n";
      return 1;
    }
    imu.initialize();

    // Average out measurements of GyroY to get static offset at start position.
    int16_t gyroY_static = 0;
    int GYROY_INIT_MEASURE = 25;
    for (int i=0;i<GYROY_INIT_MEASURE;i++) {
        gyroY_static += imu.getRotationY();
        bcm2835_delay(10);
    }
    gyroY_static = gyroY_static/GYROY_INIT_MEASURE;


    imu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    printf("  %d \t %d \t %d \t %d \t %d \t %d\r", ax, ay, az, gx, gy, gz);
    fflush(stdout);
    bcm2835_delay(100);

    // main loop
    int i=0;
    for(i=0; i<3; i++)
    {
        motor_a.forward();
        motor_b.forward();
 
	// wait a bit
	delay(500);
 
        motor_a.backward();
        motor_b.backward();
 
	// wait a bit
	delay(500);
    }

    motor_a.stop();
    motor_b.stop();
 
    return 0;
}
