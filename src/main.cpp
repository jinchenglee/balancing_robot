// main.c
 
#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>
#include <stdlib.h>
#include <stdio.h>
#include <cppgpio.hpp>

#include "bcm2835/bcm2835.h"
#include "motor/motor.h"
#include "I2Cdev/I2Cdev.h"
#include "imu/imu.h"
 
using namespace std;

int main(int argc, char **argv)
{
    // GPIO pin assignment
    // Test pin 
    //RPiGPIOPin PIN = RPI_V2_GPIO_P1_37; // GPIO 26
    // Motor A
    RPiGPIOPin PWMA = RPI_V2_GPIO_P1_11; // GPIO 17
    RPiGPIOPin AIN2 = RPI_V2_GPIO_P1_13; // GPIO 27
    RPiGPIOPin AIN1 = RPI_V2_GPIO_P1_15; // GPIO 22
    // Motor B
    RPiGPIOPin PWMB = RPI_V2_GPIO_P1_29; // GPIO 5
    RPiGPIOPin BIN2 = RPI_V2_GPIO_P1_31; // GPIO 6
    RPiGPIOPin BIN1 = RPI_V2_GPIO_P1_33; // GPIO 13 

    if (!bcm2835_init())
	return 1;
 
    // devices in the system
    motor motor_a, motor_b;
    imu sensor; // The IMU

    // motors init
    motor_a.init(PWMA, AIN1, AIN2);
    motor_b.init(PWMB, BIN1, BIN2);
    // PWM
    GPIO::PWMOut pwma(17, 100, 0);
    GPIO::PWMOut pwmb(5, 100, 0);

    // IMU init
    sensor.calibrate();

    // main loop
    int i=0;
    for(i=0; i<500; i++)
    {
        int tmp = i%100;
        pwmb.set_ratio(tmp);
        pwma.set_ratio(tmp);
        motor_b.forward();
        motor_a.forward();
        sensor.cal_theta();

	// wait a bit
        delay(20);
 
        //motor_a.backward();
        //motor_b.backward();
 
	// wait a bit
	//delay(10);
    }

    motor_a.stop();
    motor_b.stop();
 
    return 0;
}
