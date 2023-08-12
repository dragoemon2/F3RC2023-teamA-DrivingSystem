#include "simulation.hpp"


MotorSimulation::MotorSimulation(DriveMotor* motor, float gain, int frequency): motor(motor), gain(gain), frequency(frequency)
{
    value = 0;
    ticker.attach([this] {interrupt();}, std::chrono::milliseconds(1000)/frequency);
}


void MotorSimulation::interrupt(){
    float pwm = motor->pwm;
    if(pwm > 1){
        pwm = 1;
    }else if(pwm < -1){
        pwm = -1;
    }

    value += gain * pwm / frequency;

    _s1 = int(value/MMPP);

    while(value > MMPP){
        value -= MMPP;
        motor->encoder.IncrementedNum += 1;
    }

    while(value < -MMPP){
        value += MMPP;
        motor->encoder.IncrementedNum -= 1;
    }

}