#pragma once

#include <mbed.h>
#include "driveMotor.hpp"
#include "encoder.hpp"
#include "parameters.hpp"
#include "localization.hpp"
#include "PIDcontroller.hpp"
#include <functional>

using namespace std;

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

        //曲線移動
        void goCurveTo(float start_dir, float end_dir, float X, float Y, float D, bool stop=true, int num=8);  

        //デバッグ用
        void runNoEncoder(float pwmX, float pwmY, float dir, float pwmD, float time);

        //移動の停止
        void stopMovement(bool stop=true);

        void attachLoop(function<void(void)> loop_func);

        bool moving = false;

        DriveBase(DriveMotor* motor_0, DriveMotor* motor_1, DriveMotor* motor_2, DriveMotor* motor_3, float kp_1=DRIVEBASE_KP, float ki_1=DRIVEBASE_KI, float kd_1=DRIVEBASE_KD, float kp_2=DRIVEBASE_ROTATE_KP, float ki_2=DRIVEBASE_ROTATE_KI, float kd_2=DRIVEBASE_ROTATE_KD);

        int _s1;
        int _s2;

    private:
        void go(float targetSpeedX, float targetSpeedY, float targetSpeedD);
        void goTowardTargetAccDcc(float movement_threshold = MOVEMENT_THRESHOLD, float movement_threshold_rad = MOVEMENT_THRESHOLD_RAD, bool stop=true);
        void runAlongArch(float radius, float centerX, float centerY, float start_dir, float end_dir, float D, bool stop, int num);
        void resetPID();

        Ticker movementTicker;
        Timer timer;

        function<void(void)> loop;

        //目標位置
        float target_X = 0.0f;
        float target_Y = 0.0f;
        float target_D = 0.0f;

        //前回の移動
        float delta_X_before = 0.0f;
        float delta_Y_before = 0.0f;
        float delta_D_before = 0.0f;
};



