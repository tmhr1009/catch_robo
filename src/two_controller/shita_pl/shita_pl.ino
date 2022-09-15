#include "motor.h"
#include <MsTimer2.h>
#include <FlexCAN.h>
#include <kinetis_flexcan.h>
#include <FastLED.h>

#define DODAI_MIN 20
#define DODAI_MAX 590

#define NUM_LEDS 30
#define DATA_PIN 20
CRGB leds[NUM_LEDS];

FlexCAN CANTransmitter(1000000);
static CAN_message_t naka_msg;
static CAN_message_t rxmsg;

Motor mot0; //下テーブル 回転
Motor mot1; //下テーブル よこ

int now_shita_table = 0; //土台 現在位置 内部処理用
int shita_table_yoko = 0; //下 テーブルよこ移動 can値
int shita_table_yoko_sign = 0;
int shita_table_revo = 0; //下 テーブル回転 can値
int shita_table_revo_sign = 0; //0=プラス, 1=マイナス can値
int shita_led = 0; //下 LED色 can値
int can_robot_stop = 0; //受信用
int can_robot_stop_8 = 0;

void setup() {
  CANTransmitter.begin();
  pinMode(10, INPUT);
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
  naka_msg.id = 0x01;
  naka_msg.len = 8;
  FastLED.addLeds<WS2812B, DATA_PIN, RGB>(leds, NUM_LEDS);  // GRB ordering is typical
  for (int j = 0; j < NUM_LEDS; j++)
    leds[j] = CRGB(255, 0, 0);
  delay(150);
  FastLED.show();
  MsTimer2::set(2, timerInt);
  MsTimer2::start();
  Serial.begin(115200);
  pinMode(9, OUTPUT);
  mot0.init(21, 9);
  pinMode(8, OUTPUT);
  mot1.init(22, 8);
}

void loop() {
  //CAN値でLED色変更
  switch (shita_led) {
    case 0:
      for (int i = 0; i < NUM_LEDS; i++)
        leds[i] = CRGB(255, 0, 0);//緑
      break;
    case 1:
      for (int i = 0; i < NUM_LEDS; i++)
        leds[i] = CRGB(255, 255, 0);//黄
      break;
    case 2:
      for (int i = 0; i < NUM_LEDS; i++)
        leds[i] = CRGB(0, 255, 0);//赤
      break;
  }

  //CANで送る送られる値の符号変換
  //  if (shita_table_revo_sign == 1) shita_table_revo = shita_table_revo * -1;
  //  if (shita_table_yoko_sign == 1) shita_table_yoko = shita_table_yoko * -1;


  //下テーブルのエンコーダ値をCANに流す
  now_shita_table = map(analogRead(A0), DODAI_MIN, DODAI_MAX, 0, 600);
  naka_msg.buf[0] = now_shita_table;

  Serial.print("shita_table_revo : ");
  Serial.println(shita_table_revo);
  Serial.print("shita_table_yoko : ");
  Serial.println(shita_table_yoko);
  Serial.println();

  //モーター動作指示
  if (can_robot_stop == 1 || can_robot_stop_8 == 1) {
    mot0.SetSpeed(0, 0);
    mot1.SetSpeed(0, 0);
  } else if (can_robot_stop == 0 && can_robot_stop_8 == 0) {
    mot0.SetSpeed((int)abs(shita_table_revo), shita_table_revo < 0);
    mot1.SetSpeed((int)abs(shita_table_yoko), shita_table_yoko < 0);
  }
  mot0.Update();
  mot1.Update();

  delay(10);
}

void timerInt() {
  CANTransmitter.write(naka_msg);
  while ( CANTransmitter.read(rxmsg) ) {
    //中基盤から
    if (rxmsg.id == 0x04) {
      shita_table_yoko = rxmsg.buf[0];
      shita_table_revo = rxmsg.buf[1];
      shita_table_yoko = map(shita_table_yoko, 0, 200, -80, 80);
      shita_table_revo = map(shita_table_revo, 0, 200, -200, 200);
      shita_led = rxmsg.buf[2];
      can_robot_stop = rxmsg.buf[3];
      shita_table_yoko_sign = rxmsg.buf[5];
      shita_table_revo_sign = rxmsg.buf[6];
    }
    if (rxmsg.id == 0x08) {
      can_robot_stop_8 = rxmsg.buf[3];
    }
  }
  //LED点灯指示
  FastLED.show();
}
