#include <mbed.h>
#include <math.h>
#include "parameters.hpp"
#include "encoder.hpp"

Encoder::Encoder(PinName pinA, PinName pinB): A(pinA), B(pinB) {
    #if !SIMULATION
    A.rise([this]() {increment(1);}); //インクリメント
    #endif
    //A.fall([this]() {increment(-1);});
    
}

void Encoder::increment(int sgn){
    //エンコーダーのインクリメント，デクリメント
    if (B.read() == 1){
        IncrementedNum += sgn;
    }else{
        IncrementedNum -= sgn;
    }
}

float Encoder::getAmount(){
    return MMPP * IncrementedNum;
}

//チャタリング対策で10us以内での連続信号を無視するの書く
