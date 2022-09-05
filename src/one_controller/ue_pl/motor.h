#ifndef motor_h
#define motor_h
#include <Arduino.h>

class Motor
{
  public:
    Motor();
    void init(int pwm,int dir);//コンストラクタ、モータのピン設定
    void Update();//モータの出力
    void SetSpeed(int spd,int dir);//モータの速度設定
    void SetMotor(int rev);//モータの逆転設定
  private:
    int pwm_pin;//ピン設定
    int dir_pin;//ピン設定
    int pwm=0;//速度
    int rev=0;//向き
    int rev_param=0;//回転方向の基準
};

#endif
