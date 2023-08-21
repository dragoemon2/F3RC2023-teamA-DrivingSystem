#include <mbed.h>
#include "serialCommunication.hpp"

using namespace std;

SerialCommunication::SerialCommunication(int speed, PinName tx, PinName rx): serialPort(tx, rx) {
    str = "";
    func = [](string comment) {return;};
    serialPort.baud(speed);
    serialPort.format(
        /* bits */ 8,
        /* parity */ SerialBase::None,
        /* stop bit */ 1
    );
    serialPort.attach([this]() {interrupt();});
}

void SerialCommunication::writeline(string comment) {
    //書きこみ
    serialPort.write((comment+'\n').c_str(), comment.size() + 1);
}

void SerialCommunication::readChar() {
    //1文字読んで，改行だったら割り込みで関数funcを実行
    char c;
    while(serialPort.readable()){
        serialPort.read(&c, 1);
        if(c == '\n'){
            func(str);
            str = "";
        }else{
            str = str + c;
        }
    }
    _s1 = true;
    
}

void SerialCommunication::attach(function<void(string)> f) {
    //受信時に割り込みで実行される関数を設定
    func = f;
}

void SerialCommunication::detach(){
    func = [](string comment) {return;};
}

void SerialCommunication::interrupt(){
    flag = true;
}

void SerialCommunication::loop(){
    if(flag){
        flag=false;
        readChar();
    }
}

