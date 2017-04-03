// main.c
 
#include "bcm2835/bcm2835.h"
#include "motor/motor.h"
 

 
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
 
    return 0;
}
