
#include <mbed.h>
#include "driveMotor.hpp"
#include "driveBase.hpp"
#include "parameters.hpp"
#include "serialCommunication.hpp"

//エンコーダー付きDCモーター
//引数：エンコーダーのピンA,エンコーダーのピンB,モーターのピンPWM,モーターのピンDIR,速度制御のPIDゲイン,位置制御のPIDゲイン,モーターの回転方向(デフォルト=1)
DriveMotor motor0(A5, A1, D10, D8, MOTOR_0_KP_1, MOTOR_0_KI_1, MOTOR_0_KD_1, MOTOR_0_KP_2, MOTOR_0_KI_2, MOTOR_0_KD_2, 0);
DriveMotor motor1(PC_6, A0, D5, D4, MOTOR_1_KP_1, MOTOR_1_KI_1, MOTOR_1_KD_1, MOTOR_1_KP_2, MOTOR_1_KI_2, MOTOR_1_KD_2);
DriveMotor motor2(PC_8, D9, D3, D2, MOTOR_2_KP_1, MOTOR_2_KI_1, MOTOR_2_KD_1, MOTOR_2_KP_2, MOTOR_2_KI_2, MOTOR_2_KD_2);
DriveMotor motor3(D6, D12, D11, D7, MOTOR_3_KP_1, MOTOR_3_KI_1, MOTOR_3_KD_1, MOTOR_3_KP_2, MOTOR_3_KI_2, MOTOR_3_KD_2);

#if SIMULATION
//シミュレーション
MotorSimulation simulation0(&motor0, 5000);
MotorSimulation simulation1(&motor1, 5000);
MotorSimulation simulation2(&motor2, 5000);
MotorSimulation simulation3(&motor3, 5000);
#endif

Timer timer;

//足回り全体
DriveBase driveBase(&motor0, &motor1, &motor2, &motor3);

//SerialCommunication pc;

bool flag=false;

string str;


//4つのモーターの方向のチェック，エンコーダーが読めてそうかみる．
void dir_check(){
    driveBase.attachLoop([]{printf("%d,%d,%d| %d %d %d %d\n",int(driveBase.localization.posX), int(driveBase.localization.posY), int(180/PI*driveBase.localization.direction), int(motor0.encoder.getAmount()), int(motor1.encoder.getAmount()), int(motor2.encoder.getAmount()), int(motor3.encoder.getAmount()));});
    //driveBase.goTo(1000,0,0);
    driveBase.runNoEncoder(0,0,0,0.5,5);
    while(1){

    }

}

//エンコーダー&自己位置推定．モーターなし
void show(){
    while(1){
        printf("%d,%d,%d| %d %d %d %d\n",int(driveBase.localization.posX), int(driveBase.localization.posY), int(180/PI*driveBase.localization.direction), int(motor0.encoder.getAmount()), int(motor1.encoder.getAmount()), int(motor2.encoder.getAmount()), int(motor3.encoder.getAmount()));
        wait_us(100000);
    }
}

//動作テスト
void drive(){
    //10秒待機
    wait_us(1000000*10);

    //出力を設定
    driveBase.attachLoop([](){printf("%d,%d\n", int(driveBase.localization.posX), int(driveBase.localization.posY));});

    //引数: 目標X座標[mm],目標Y座標[mm],目標方角[rad](絶対値はPI未満にする)
    driveBase.goTo(3000, 0, 0);
    driveBase.goTo(3000, 3000, 0);
    driveBase.goTo(0, 0, PI/2);
    while(1){

    }
}


#define test_motor (motor0)

//モーターの位置制御試す用
void move_test(){
    test_motor.rotateTo(5000,false); 

    //printf("hoge");

    while(test_motor.moving){
        //printf("%d, %d, 0, 0\n", int(motor2._s2), int(motor2._s1));
        
        printf("%d, %d, %d, %d\n", int(test_motor._s2), int(test_motor._s1), 5000, int(test_motor.encoder.getAmount()));

        
        //printf("%d %d %d %d\n", int(motor0.encoder.getAmount()), int(motor1.encoder.getAmount()), int(motor2.encoder.getAmount()), int(motor3.encoder.getAmount()));
        //printf("%d,%d,%d| %d %d %d %d\n",int(driveBase.localization.posX), int(driveBase.localization.posY), int(180/PI*driveBase.localization.direction), int(motor0.encoder.getAmount()), int(motor1.encoder.getAmount()), int(motor2.encoder.getAmount()), int(motor3.encoder.getAmount()));
        //printf("%d,%d,%d| %d %d %d %d\n",int(driveBase.localization.posX), int(driveBase.localization.posY), int(180/PI*driveBase.localization.direction), int(motor0.encoder.getAmount()), int(motor1.encoder.getAmount()), int(motor2.encoder.getAmount()), int(motor3.encoder.getAmount()));
        //printf("%d,%d,%d\n",int(driveBase.localization.posX), int(driveBase.localization.posY), int(180/PI*driveBase.localization.direction));
        wait_us(20000);
        
    }
}


//モーターの速度制御試す用
void speed_test(){
    timer.start();
    //test_motor.rotateTo(1000,false); 
    

    test_motor.rotatePermanent(1000,false);

    //printf("hoge");

    while(true){
        printf("%d, %d, 0, 0\n", int(test_motor._s2), int(test_motor._s1));
        
        //printf("%d, %d, %d, %d\n", int(motor2._s2), int(motor2._s1), 1000, int(motor2.encoder.getAmount()));

        
        //printf("%d %d %d %d\n", int(motor0.encoder.getAmount()), int(motor1.encoder.getAmount()), int(motor2.encoder.getAmount()), int(motor3.encoder.getAmount()));
        //printf("%d,%d,%d| %d %d %d %d\n",int(driveBase.localization.posX), int(driveBase.localization.posY), int(180/PI*driveBase.localization.direction), int(motor0.encoder.getAmount()), int(motor1.encoder.getAmount()), int(motor2.encoder.getAmount()), int(motor3.encoder.getAmount()));
        //printf("%d,%d,%d| %d %d %d %d\n",int(driveBase.localization.posX), int(driveBase.localization.posY), int(180/PI*driveBase.localization.direction), int(motor0.encoder.getAmount()), int(motor1.encoder.getAmount()), int(motor2.encoder.getAmount()), int(motor3.encoder.getAmount()));
        //printf("%d,%d,%d\n",int(driveBase.localization.posX), int(driveBase.localization.posY), int(180/PI*driveBase.localization.direction));
        wait_us(20000);
        
        //printf("%d,%d,%d| %d %d %d %d\n",int(driveBase.localization.posX), int(driveBase.localization.posY), int(180/PI*driveBase.localization.direction), int(motor0.encoder.getAmount()), int(motor1.encoder.getAmount()), int(motor2.encoder.getAmount()), int(motor3.encoder.getAmount()));


        //pc.loop();
        if(chrono::duration<float>(timer.elapsed_time()).count() > 5 && !flag){
            flag = true;
            test_motor.rotatePermanent(0,false);
        }

        if(chrono::duration<float>(timer.elapsed_time()).count() > 8){
            break;
            
        }
    }
    //motor2.rotatePermanent(0,false);

    test_motor.stop();
}





//#define test_motor (motor0)

//一定duty比で動かす
void motor_test(){
    timer.start();
    test_motor.setPWM(0.3);
    float l = test_motor.encoder.getAmount();
    while(chrono::duration<float>(timer.elapsed_time()).count() < 3000){
        int v = int((test_motor.encoder.getAmount() - l)*200);
        l = test_motor.encoder.getAmount();
        printf("%d\n", int(v));
        wait_us(5000);
    }
    test_motor.setPWM(0);
}

#if 0
//エンコーダーに乗ってるノイズの確認用．エンコーダーのピンAをA4にさして，analoginで値を読む

#define SAMPLING_N (500)

AnalogIn EncoderA(A4);


void encoder_test(){
    int array[SAMPLING_N];
    test_motor.setPWM(0.3);
    wait_us(1000000);
    timer.start();
    int next_time = std::chrono::duration_cast<std::chrono::microseconds>(timer.elapsed_time()).count();
    int i;
    while(true){
        if(std::chrono::duration_cast<std::chrono::microseconds>(timer.elapsed_time()).count() > next_time){
            array[i] = EncoderA.read_u16();
            next_time += 1000;
            i++;
            if(i==SAMPLING_N){
                break;
            }
        }
    }
    test_motor.setPWM(0);

    for(int v:array){
        printf("%d\n", v);
    }
}

#endif
int main(){
    drive();
}



