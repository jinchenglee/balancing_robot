//
// Class to control robot motors.
//

#ifndef MOTOR_H_
#define MOTOR_H_

#include "bcm2835/bcm2835.h"

class motor {

public:

    // Pins controls the motor
    RPiGPIOPin PWM, IN1, IN2;

    // Constructor
    motor();
    // Destructor
    ~motor();

    // Initialization
    void init(
            RPiGPIOPin &pwm,
            RPiGPIOPin &in1,
            RPiGPIOPin &in2
            );

    // Motor movement control
    void forward();
    void backward();
    void stop();

};

#endif // MOTOR_H_
