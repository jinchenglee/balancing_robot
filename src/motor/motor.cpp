// motor.cpp

#include "motor/motor.h"

motor::motor() {}
motor::~motor() {}

void motor::init(RPiGPIOPin& pwm, RPiGPIOPin& in1, RPiGPIOPin& in2) {
    // GPIO pin assignment
    PWM = pwm;
    IN1 = in1;
    IN2 = in2;
    // Set selected GPIO pins to be output mode
    // TODO:HACK bcm2835_gpio_fsel(PWM, BCM2835_GPIO_FSEL_OUTP);
    bcm2835_gpio_fsel(IN1, BCM2835_GPIO_FSEL_OUTP);
    bcm2835_gpio_fsel(IN2, BCM2835_GPIO_FSEL_OUTP);
    // Tie PWMA/B to high
    // TODO:HACK bcm2835_gpio_write(PWM, HIGH);
 }

void motor::forward() {
    bcm2835_gpio_write(IN1, HIGH);
    bcm2835_gpio_write(IN2, LOW);
}

void motor::backward() {
    bcm2835_gpio_write(IN1, LOW);
    bcm2835_gpio_write(IN2, HIGH);
}

void motor::stop()
{
    bcm2835_gpio_write(IN1, LOW);
    bcm2835_gpio_write(IN2, LOW);
}
 

