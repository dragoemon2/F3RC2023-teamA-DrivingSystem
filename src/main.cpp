
#include <mbed.h>
#include "driveMotor.hpp"
#include "driveBase.hpp"
#include "parameters.hpp"
#include "serialCommunication.hpp"

#include "simulation.hpp"


//エンコーダーのピン，モーターのピン，PIDゲイン

DriveMotor motor0(PB_4, PB_5, PB_1, PC_9, MOTOR_0_KP_1, MOTOR_0_KI_1, MOTOR_0_KD_1, MOTOR_0_KP_2, MOTOR_0_KI_2, MOTOR_0_KD_2);
DriveMotor motor1(PA_0, PA_1, PB_15, PC_8, MOTOR_1_KP_1, MOTOR_1_KI_1, MOTOR_1_KD_1, MOTOR_1_KP_2, MOTOR_1_KI_2, MOTOR_1_KD_2);
DriveMotor motor2(PA_11, PB_12, PB_14, PC_6, MOTOR_2_KP_1, MOTOR_2_KI_1, MOTOR_2_KD_1, MOTOR_2_KP_2, MOTOR_2_KI_2, MOTOR_2_KD_2);
DriveMotor motor3(PC_11, PD_2, PB_13, PB_2, MOTOR_3_KP_1, MOTOR_3_KI_1, MOTOR_3_KD_1, MOTOR_3_KP_2, MOTOR_3_KI_2, MOTOR_3_KD_2);

//シミュレーション
MotorSimulation simulation1(&motor0, 5000);
MotorSimulation simulation2(&motor1, 5000);
MotorSimulation simulation3(&motor2, 5000);
MotorSimulation simulation4(&motor3, 5000);

//足回り全体
DriveBase driveBase(&motor0, &motor1, &motor2, &motor3);

//SerialCommunication pc;

bool flag=false;

string str;

Timer timer;


void drive(){
    driveBase.attachLoop([](){printf("%d %d %d %d\n", int(driveBase._s1), int(driveBase.localization.posX), int(driveBase.localization.posY), int(driveBase.localization.direction*180/PI));});
    driveBase.goTo(20000, 20000, PI/4);
    while(1){

    }
}



void run(string str){
    printf(str.c_str());
    float kp1, ki1, kd1, kp2, ki2, kd2;
    
    int N = sscanf(str.c_str(), "%f,%f,%f,%f,%f,%f", &kp1, &ki1, &kd1, &kp2, &ki2, &kd2);
    
    if(N!=4){
        return;
    }

    motor2.stop();

    motor2.pidController.setGain(kp1, ki1, kd1);
    motor2.pidSpeedController.setGain(kp2, ki2, kd2);
    motor2.pidController.reset();
    motor2.pidSpeedController.reset();

    motor2.rotateTo(1000,false);
}


void speed_test(){
    timer.start();
    //motor2.rotateTo(1000,false); 

    motor2.rotatePermanent(1000,false);

    //printf("hoge");

    while(true){
        printf("%d, %d, 0, 0\n", int(motor2._s2), int(motor2._s1));
        
        //printf("%d, %d, %d, %d\n", int(motor2._s2), int(motor2._s1), 1000, int(motor2.encoder.getAmount()));

        
        //printf("%d %d %d %d\n", int(motor0.encoder.getAmount()), int(motor1.encoder.getAmount()), int(motor2.encoder.getAmount()), int(motor3.encoder.getAmount()));
        //printf("%d,%d,%d| %d %d %d %d\n",int(driveBase.localization.posX), int(driveBase.localization.posY), int(180/PI*driveBase.localization.direction), int(motor0.encoder.getAmount()), int(motor1.encoder.getAmount()), int(motor2.encoder.getAmount()), int(motor3.encoder.getAmount()));
        //printf("%d,%d,%d| %d %d %d %d\n",int(driveBase.localization.posX), int(driveBase.localization.posY), int(180/PI*driveBase.localization.direction), int(motor0.encoder.getAmount()), int(motor1.encoder.getAmount()), int(motor2.encoder.getAmount()), int(motor3.encoder.getAmount()));
        //printf("%d,%d,%d\n",int(driveBase.localization.posX), int(driveBase.localization.posY), int(180/PI*driveBase.localization.direction));
        wait_us(20000);

        //pc.loop();
        if(chrono::duration<float>(timer.elapsed_time()).count() > 5 && !flag){
            flag = true;
            motor2.rotatePermanent(0,false);
        }

        if(chrono::duration<float>(timer.elapsed_time()).count() > 8){
            break;
            
        }
    }

    

    //motor2.rotatePermanent(0,false);

    motor2.stop();
}




void move_test(){
    motor2.rotateTo(5000,false); 

    //printf("hoge");

    while(motor2.moving){
        //printf("%d, %d, 0, 0\n", int(motor2._s2), int(motor2._s1));
        
        printf("%d, %d, %d, %d\n", int(motor2._s2), int(motor2._s1), 5000, int(motor2.encoder.getAmount()));

        
        //printf("%d %d %d %d\n", int(motor0.encoder.getAmount()), int(motor1.encoder.getAmount()), int(motor2.encoder.getAmount()), int(motor3.encoder.getAmount()));
        //printf("%d,%d,%d| %d %d %d %d\n",int(driveBase.localization.posX), int(driveBase.localization.posY), int(180/PI*driveBase.localization.direction), int(motor0.encoder.getAmount()), int(motor1.encoder.getAmount()), int(motor2.encoder.getAmount()), int(motor3.encoder.getAmount()));
        //printf("%d,%d,%d| %d %d %d %d\n",int(driveBase.localization.posX), int(driveBase.localization.posY), int(180/PI*driveBase.localization.direction), int(motor0.encoder.getAmount()), int(motor1.encoder.getAmount()), int(motor2.encoder.getAmount()), int(motor3.encoder.getAmount()));
        //printf("%d,%d,%d\n",int(driveBase.localization.posX), int(driveBase.localization.posY), int(180/PI*driveBase.localization.direction));
        wait_us(20000);
        
    }
}


int main(){
    drive();
}


