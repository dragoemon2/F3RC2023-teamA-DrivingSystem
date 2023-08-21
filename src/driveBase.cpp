#include <mbed.h>
#include <math.h>
#include <chrono>
#include "driveBase.hpp"
#include "parameters.hpp"
#include "simpleFunctions.hpp"

//初期化
DriveBase::DriveBase(DriveMotor* motor_0, DriveMotor* motor_1, DriveMotor* motor_2, DriveMotor* motor_3, float kp_1, float ki_1, float kd_1, float kp_2, float ki_2, float kd_2): localization(motor_0, motor_1, motor_2, motor_3), pidController(SPEED_ADJUSTMENT_FREQUENCY, kp_1, ki_1, kd_1), pidRotateController(SPEED_ADJUSTMENT_FREQUENCY, kp_2, ki_2, kd_2) {
    motors[0] = motor_0;
    motors[1] = motor_1;
    motors[2] = motor_2;
    motors[3] = motor_3;

    moving = false;
    loop = [this] {return;};
}

void DriveBase::resetPID(){
    for (int i=0;i<4;i++){
        motors[i]->pidController.reset();
    }
    pidController.reset();
    pidRotateController.reset();
}

//速度を指定して移動
void DriveBase::go(float targetSpeedX, float targetSpeedY, float targetSpeedD){
    float targetSpeedR = sqrtf(targetSpeedX*targetSpeedX + targetSpeedY*targetSpeedY);

    _s1 = int(targetSpeedX);
    _s2 = int(targetSpeedY);

    //速度を制限する
    if(targetSpeedR > MAX_SPEED){
        targetSpeedX = MAX_SPEED*(targetSpeedX/targetSpeedR);
        targetSpeedY = MAX_SPEED*(targetSpeedY/targetSpeedR);
        pidController.reset();
    }

    if(targetSpeedD > MAX_ROTATE_SPEED){
        targetSpeedD = MAX_ROTATE_SPEED;
        pidRotateController.reset();
    }else if(targetSpeedD < -MAX_ROTATE_SPEED){
        targetSpeedD = -MAX_ROTATE_SPEED;
        pidRotateController.reset();
    }

    float targetAccX = (targetSpeedX - localization.speedX) * SPEED_ADJUSTMENT_FREQUENCY;
    float targetAccY = (targetSpeedY - localization.speedY) * SPEED_ADJUSTMENT_FREQUENCY;
    float targetAccR = sqrtf(targetAccX*targetAccX + targetAccY*targetAccY);
    float targetAccD = (targetSpeedD - localization.rotateSpeed) * SPEED_ADJUSTMENT_FREQUENCY;

    //加速度を制限する
    if(targetAccR > MAX_ACCELERATION){
        targetAccX = MAX_ACCELERATION*(targetAccX/targetAccR);
        targetAccY = MAX_ACCELERATION*(targetAccY/targetAccR);
        targetSpeedX = localization.speedX + targetAccX/SPEED_ADJUSTMENT_FREQUENCY;
        targetSpeedY = localization.speedY + targetAccY/SPEED_ADJUSTMENT_FREQUENCY;
    }

    if(targetAccD > MAX_ROTATE_ACCELERATION){
        targetSpeedD = localization.rotateSpeed + MAX_ROTATE_ACCELERATION / SPEED_ADJUSTMENT_FREQUENCY;
    }else if(targetAccD < -MAX_ROTATE_ACCELERATION){
        targetSpeedD = localization.rotateSpeed - MAX_ROTATE_ACCELERATION / SPEED_ADJUSTMENT_FREQUENCY;
    }

    

    //X, Yに回転行列をかける
    float vx = cos(localization.direction)*targetSpeedX + sin(localization.direction)*targetSpeedY;
    float vy = -sin(localization.direction)*targetSpeedX + cos(localization.direction)*targetSpeedY;

    //各モーターの速度
    float speeds[4]; //モーターの速度
    speeds[0] = SQRT2/2 * (- vx + vy) + TRED_RADIUS * targetSpeedD;
    speeds[1] = SQRT2/2 * (- vx - vy) + TRED_RADIUS * targetSpeedD;
    speeds[2] = SQRT2/2 * (+ vx - vy) + TRED_RADIUS * targetSpeedD;
    speeds[3] = SQRT2/2 * (+ vx + vy) + TRED_RADIUS * targetSpeedD;
    
    
    for (int i=0;i<4;i++){
        motors[i]->rotate(speeds[i]);
    }

}


//現在決められている目標に向かって進む
//PID制御で目標値に向かうが，速度・加速度に制限を設けることで台形制御を実現する
void DriveBase::goTowardTargetAccDcc(float movement_threshold, float movement_threshold_rad, bool stop){
    float differenceX = target_X-localization.posX;
    float differenceY = target_Y-localization.posY;
    float differenceR = sqrtf(differenceX*differenceX + differenceY*differenceY);
    float differenceD = radiansMod(target_D-localization.direction);

    float targetSpeedR = pidController.calculate(differenceR);


    float targetSpeedX, targetSpeedY;

    if(differenceR == 0){
        //0除算の回避
        targetSpeedX = 0;
        targetSpeedY = 0;
    }else{
        targetSpeedX = targetSpeedR*(differenceX/differenceR);
        targetSpeedY = targetSpeedR*(differenceY/differenceR);
    }

    float targetSpeedD = pidRotateController.calculate(differenceD);



    //targetSpeedD = 0;

    go(targetSpeedX, targetSpeedY, targetSpeedD);

    if (differenceR < movement_threshold && abs(radiansMod(target_D - localization.direction)) < movement_threshold_rad){
        stopMovement(stop);
    }
}


//モーターの停止
void DriveBase::stopMovement(bool stop){
    movementTicker.detach();
    moving = false;
    if(stop){
        for(int i=0;i<4;i++){
            motors[i]->stop();
        }
    }
}


//目標位置に向かって直線移動する
void DriveBase::goTo(float X, float Y, float D, bool idle, bool stop){
    if(!moving){
        resetPID();
        moving = true;
    }

    if(stop){
        //目標位置の設定
        target_X = X;
        target_Y = Y;
        target_D = D;

        //割り込みの設定
        movementTicker.attach([this] {goTowardTargetAccDcc();}, std::chrono::milliseconds(1000)/SPEED_ADJUSTMENT_FREQUENCY);
    }else{
        float distance = sqrtf((X - localization.posX)*(X - localization.posX) + (Y - localization.posY)*(Y - localization.posY));
        if(distance == 0.0f){
            target_X = X;
            target_Y = Y;
            target_D = D;
        }else{
            target_X = X + MOVEMENT_SWITCH_THRESHOLD * (X - localization.posX)/distance;
            target_Y = Y + MOVEMENT_SWITCH_THRESHOLD * (Y - localization.posY)/distance;
            target_D = D;
        }

        //割り込みの設定
        movementTicker.attach([this] {goTowardTargetAccDcc(MOVEMENT_SWITCH_THRESHOLD, 2*PI, false);}, std::chrono::milliseconds(1000)/SPEED_ADJUSTMENT_FREQUENCY);
    }
    
    //idle=trueなら移動が終わるまで待機
    if(idle){
        while(moving) {
            loop();
            wait_ns(1);
        }
    }
}



void DriveBase::runNoEncoder(float pwmX, float pwmY, float dir, float pwmD, float time){
    if(moving){
        printf("warning: a motion requested while the robot is moving.");
        movementTicker.detach();
    }

    float vx = cos(dir)*pwmX + sin(dir)*pwmY;
    float vy = -sin(dir)*pwmX + cos(dir)*pwmY;

    //各モーターの速度
    float speeds[4]; //モーターの速度
    speeds[0] = SQRT2/2 * (- vx + vy) + pwmD;
    speeds[1] = SQRT2/2 * (- vx - vy) + pwmD;
    speeds[2] = SQRT2/2 * (+ vx - vy) + pwmD;
    speeds[3] = SQRT2/2 * (+ vx + vy) + pwmD;

    timer.reset();
    timer.start();

    for(int i=0;i<4;i++){
        motors[i]->setPWM(speeds[i]);
    }
    

    while(chrono::duration<float>(timer.elapsed_time()).count() < time){
        loop();
        wait_ns(1);
    }

    stopMovement();
}

//超信地旋回
void DriveBase::rotateTo(float D, bool idle){
    goTo(localization.posX, localization.posY, D, idle);
}

//平行移動
void DriveBase::goParallelTo(float X, float Y, bool idle){
    goTo(X, Y, localization.direction, idle);
}


void DriveBase::runAlongArch(float radius, float centerX, float centerY, float start_dir, float end_dir, float D, bool stop, int num){
    for(int i=0;i<num+1;i++){
        float X = centerX + radius*sin(start_dir + radiansMod(end_dir-start_dir) * (float(i)/num));
        float Y = centerY - radius*cos(start_dir + radiansMod(end_dir-start_dir) * (float(i)/num));
        
        if(i != 0 || X != localization.posX || Y != localization.posY){
            goTo(X, Y, D, true, i == num && stop);
        }
    }
}


void DriveBase::goCurveTo(float start_dir, float end_dir, float X, float Y, float D, bool stop, int num){
    if(radiansMod(start_dir - end_dir) == 0){
        goTo(X, Y, D, true, stop);
        return;
    }

    float deltaX = X - localization.posX;
    float deltaY = Y - localization.posY;

    //カーブの半径
    float radius1 = (+ sin(end_dir) * deltaX - cos(end_dir) * deltaY)/(1-cos(end_dir - start_dir));
    float radius2 = (- sin(start_dir)*deltaX + cos(start_dir)*deltaY)/(1-cos(end_dir - start_dir));

    if(abs(radius1) < abs(radius2)){
        float centerX = localization.posX - radius1*sin(start_dir);
        float centerY = localization.posY + radius1*cos(start_dir);
        runAlongArch(radius1, centerX, centerY, start_dir, end_dir, D, false, num);
        goTo(X, Y, D, true, stop);
    }else{
        float centerX = X - radius2*sin(end_dir);
        float centerY = Y + radius2*cos(end_dir);
        runAlongArch(radius2, centerX, centerY, start_dir, end_dir, D, stop, num);
    }
}

void DriveBase::attachLoop(function<void(void)> loop_func){
    loop = loop_func;
}