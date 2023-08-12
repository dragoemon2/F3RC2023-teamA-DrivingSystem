#pragma once
#include <functional>
#include <mbed.h>
#include "parameters.hpp"
#include "simulation.hpp"
#include "driveMotor.hpp"

using namespace std;


class MotorSimulation{
    public:
        DriveMotor* motor;
        float gain;
        int frequency;
        int _s1;
        MotorSimulation(DriveMotor* motor, float gain, int frequency=ENCODER_LOCALIZATION_FREQUENCY);
    private:
        void interrupt();
        float value = 0;

        Ticker ticker;
};