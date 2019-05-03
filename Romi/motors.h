#ifndef _Motor_h
#define _Motor_h
#include <stdint.h>

//Pin definitions for motor

const byte default_max_power = 255;
const byte PWM_MAX = 255;

class Motor
{
    public:

        Motor(byte pwm, byte dir); //Constructor - stores pins for controlling the motor
        void setPower(float demand); //Sets the duty factor of the PWM signal sent to the Motor's H-Bridge

    private:

        byte pwm_pin;
        byte dir_pin;

        float max_power=default_max_power;
};


Motor::Motor(byte pwm, byte dir)
{

    //store pin numbers
    pwm_pin = pwm;
    dir_pin = dir;

    //set pins as outputs
    pinMode(pwm_pin, OUTPUT);
    pinMode(dir_pin, OUTPUT);

    //set initial speed and direction
    digitalWrite(pwm_pin, LOW);
    digitalWrite(dir_pin, LOW);

}


void Motor::setPower(float demand)
{
    // Toggle direction based on sign of demand.
    digitalWrite( dir_pin, demand < 0 ? HIGH : LOW );

    // Write out absolute magnitude to pwm pin
    demand = abs( demand );
    demand = constrain( demand, 0, max_power );
    analogWrite( pwm_pin, demand );
}


#endif

