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

    const float INIT_ANGLE = -0.005;
    const float SumErrMax = 3;
    const float SumErrMin = -3;
    float curErr = 0, prevErr = 0, SumErr = 0;
    float integralTerm = 0, derivativeTerm = 0;
    float Kp = 30000.0, Ki = 50000.0, Kd = 500.0;
    float Cn = 0.0;

    // main loop
    int i=0;
    for(i=0; i<500; i++)
    {
        float ang = sensor.cal_theta();

            curErr = ang - INIT_ANGLE; //error    
            SumErr += curErr;

            if (SumErr > SumErrMax) SumErr = SumErrMax;
            else if (SumErr < SumErrMin) SumErr = SumErrMin;

            //Ki*SumE/(Kp*Fs*X) 
            integralTerm = SumErr * sensor.delta_ts * Ki / Kp * 10.0; 
            derivativeTerm = curErr - prevErr;

            if(derivativeTerm > 0.1) derivativeTerm = 0.1;
            else if (derivativeTerm < -0.1) derivativeTerm = -0.1;

            // Kd(curErr-prevErr)*Ts/(Kp*X)
            derivativeTerm = derivativeTerm * Kd * sensor.delta_ts / Kp; 

            if(derivativeTerm > 120) derivativeTerm = 120;
            else if (derivativeTerm < -120) derivativeTerm = -120;

            Cn = (curErr + integralTerm + derivativeTerm) * Kp / 10.0;

            float throttle = abs(long(Cn));
            cout << "Cn = " << Cn << "throttle = " << throttle << endl;

            if (abs(long(ang)) > 0.7)  //if angle too large to correct, stop motor        
            {
                motor_a.stop();
                motor_b.stop();
            }
            else {
                pwmb.set_ratio(throttle);
                pwma.set_ratio(throttle);
                if (Cn >0) {
                    motor_b.forward();
                    motor_a.forward();
                } else {
                    motor_a.backward();
                    motor_b.backward();
                }
            }

            prevErr = curErr;


        // wait a bit
        delay(20);


        // wait a bit
        //delay(10);
    }

    motor_a.stop();
    motor_b.stop();

    return 0;
}

