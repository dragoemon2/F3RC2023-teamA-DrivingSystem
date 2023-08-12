
#include <mbed.h>
#include "driveMotor.hpp"
#include "driveBase.hpp"
#include "parameters.hpp"

#include "simulation.hpp"



//エンコーダーのピン，モーターのピン，PIDゲイン
DriveMotor motor0(A0, A1, D2, D3, MOTOR_0_KP_1, MOTOR_0_KI_1, MOTOR_0_KD_1, MOTOR_0_KP_2, MOTOR_0_KI_2, MOTOR_0_KD_2);
DriveMotor motor1(A2, A3, D4, D5, MOTOR_1_KP_1, MOTOR_1_KI_1, MOTOR_1_KD_1, MOTOR_1_KP_2, MOTOR_1_KI_2, MOTOR_1_KD_2);
DriveMotor motor2(A4, A5, D6, D7, MOTOR_2_KP_1, MOTOR_2_KI_1, MOTOR_2_KD_1, MOTOR_2_KP_2, MOTOR_2_KI_2, MOTOR_2_KD_2);
DriveMotor motor3(D0, D1, D8, D9, MOTOR_3_KP_1, MOTOR_3_KI_1, MOTOR_3_KD_1, MOTOR_3_KP_2, MOTOR_3_KI_2, MOTOR_3_KD_2);

MotorSimulation simulation0(&motor0, 5000);
MotorSimulation simulation1(&motor1, 5000);
MotorSimulation simulation2(&motor2, 5000);
MotorSimulation simulation3(&motor3, 5000);

//足回り全体
DriveBase driveBase(&motor0, &motor1, &motor2, &motor3);



int main(){
    //motor0.attachLoop([] {printf("%d %d\n", int(motor0.pwm*100), int(motor0.encoder.getAmount()));});

    driveBase.attachLoop([] {printf("X:%d Y:%d D:%d\n", int(driveBase.localization.posX), int(driveBase.localization.posY), int(driveBase.localization.direction*180/PI));});

    //motor0.rotateTo(1000);

    //driveBase.goTo(2200,1250,PI);

    
    //受け取り

    driveBase.goTo(2864,1920,PI/2);
    driveBase.goTo(2864,2210,PI/2);

    //設置

    driveBase.goTo(2200,1250,PI);

    //受け取り

    driveBase.goTo(4050,213,0);
    driveBase.goTo(4280,213,0);
    
    
}

