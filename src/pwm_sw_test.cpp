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

#define ANGLE_BUFFER_LENGTH 5

using namespace std;

int main(int argc, char **argv)
{
        float throttle = 20.0;

    if (argc>1) { // User set PID
        if (argc!=2) {
            cerr << "Usage: balancing_robot throttle" << endl;
            return 0;
        } else {
            throttle = atof(argv[1]);
        }
    } 
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

        
    int i=0;
    for(i=0; i<3; i++)
    {


        pwma.set_ratio(throttle);
        pwmb.set_ratio(throttle);
        //delay(30);

        cout << "in loop " << i << endl;


        motor_a.forward();
        motor_b.forward();

        delay(300);

        motor_a.backward();
        motor_b.backward();
        // wait a bit
        delay(300);

    }

    motor_a.stop();
    motor_b.stop();

    return 0;
}

