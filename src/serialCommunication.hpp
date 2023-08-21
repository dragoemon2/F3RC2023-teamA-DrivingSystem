#pragma once

#include <mbed.h>
#include <string>

using namespace std;

/*
mbedでのシリアル通信の汎用class
usbケーブルでPCとの通信はできることを確認済
esp32との通信は試してない
*/

class SerialCommunication{
    public:
        UnbufferedSerial serialPort;

        //初期化処理
        SerialCommunication(int speed=9600, PinName tx=USBTX, PinName rx=USBRX);

        //受信時に割り込みで実行する関数funcを設定
        void attach(function<void(string)> f);

        //funcの設定をリセット
        void detach();

        //一行送信(末尾の'\n'は必要なし)
        void writeline(string comment);

        void loop();

        bool _s1=false;

    private:
        //前の改行から現時点で受信した文字列
        string str;

        bool flag;

        //受信時に割り込みで実行する関数
        function<void(string)> func; 

        //1文字読んでstrの末尾に追加．改行なら関数funcを実行
        void readChar();

        void interrupt();
};