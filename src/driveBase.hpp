#pragma once

#include <mbed.h>
#include "driveMotor.hpp"
#include "encoder.hpp"
#include "parameters.hpp"
#include "localization.hpp"
#include "PIDcontroller.hpp"


float radiansMod(float x, float y=2*PI);

class DriveBase{
    public:
        DriveMotor* motors[4];
        Localization localization;

        PIDController pidController;
        PIDController pidRotateController;
        

        //直線移動
        void goTo(float X, float Y, float D, bool idle=true, bool stop=true);
        void rotateTo(float D, bool idle=true);
        void goParallelTo(float X, float Y, bool idle=true);

        void runNoEncoder(float pwmX, float pwmY, float dir, float pwmD, float time);

        //曲線移動 これから作る
        void runAlongPoints(float X, float Y, int num);      

        //移動の停止
        void stopMovement();

        bool moving = false;

        DriveBase(DriveMotor* motor_0, DriveMotor* motor_1, DriveMotor* motor_2, DriveMotor* motor_3, float kp_1=DRIVEBASE_KP, float ki_1=DRIVEBASE_KI, float kd_1=DRIVEBASE_KD, float kp_2=DRIVEBASE_ROTATE_KP, float ki_2=DRIVEBASE_ROTATE_KI, float kd_2=DRIVEBASE_ROTATE_KD);

        int _s1;

    private:
        void go(float targetSpeedX, float targetSpeedY, float targetSpeedD);
        void goTowardTargetAccDcc(float movement_threshold = MOVEMENT_THRESHOLD, float movement_threshold_rad = MOVEMENT_THRESHOLD_RAD, bool stop=true);
        void resetPID();

        Ticker movementTicker;
        Timer timer;

        //目標位置
        float target_X = 0.0f;
        float target_Y = 0.0f;
        float target_D = 0.0f;

        //前回の移動
        float delta_X_before = 0.0f;
        float delta_Y_before = 0.0f;
        float delta_D_before = 0.0f;
};



