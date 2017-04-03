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

int imu_test() {
  cout << "MPU6050 3-axis acceleromter example program\n";
  I2Cdev::initialize();
  MPU6050 accelgyro ;
  int16_t ax, ay, az;
  int16_t gx, gy, gz;
  if ( accelgyro.testConnection() ) 
    printf("MPU6050 connection test successful\n") ;
  else {
    fprintf( stderr, "MPU6050 connection test failed! something maybe wrong, continuing anyway though ...\n");
    //return 1;
  }
  accelgyro.initialize();
  // use the code below to change accel/gyro offset values
  /*
  printf("Updating internal sensor offsets...\n");
  // -76	-2359	1688	0	0	0
  printf("%i \t %i \t %i \t %i \t %i \t %i\n", 
	 accelgyro.getXAccelOffset(),
	 accelgyro.getYAccelOffset(),
	 accelgyro.getZAccelOffset(),
	 accelgyro.getXGyroOffset(),
	 accelgyro.getYGyroOffset(),
	 accelgyro.getZGyroOffset());
  accelgyro.setXGyroOffset(220);
  accelgyro.setYGyroOffset(76);
  accelgyro.setZGyroOffset(-85);
  printf("%i \t %i \t %i \t %i \t %i \t %i\n", 
	 accelgyro.getXAccelOffset(),
	 accelgyro.getYAccelOffset(),
	 accelgyro.getZAccelOffset(),
	 accelgyro.getXGyroOffset(),
	 accelgyro.getYGyroOffset(),
	 accelgyro.getZGyroOffset());
  */
  
  printf("\n");
  printf("  ax \t ay \t az \t gx \t gy \t gz:\n");
  while (true) {
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    printf("  %d \t %d \t %d \t %d \t %d \t %d\r", ax, ay, az, gx, gy, gz);
    fflush(stdout);
    bcm2835_delay(100);
  }
  return 1; 
}
 
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
 
    // main loop
    int i=0;
    for(i=0; i<3; i++)
    {
	// Turn it on
        motor_a.forward();
        motor_b.forward();
 
	// wait a bit
	delay(500);
 
	// turn it off
        motor_a.backward();
        motor_b.backward();
 
	// wait a bit
	delay(500);
    }

    motor_a.stop();
    motor_b.stop();
 
    imu_test();

    return 0;
}
