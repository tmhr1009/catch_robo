#include "pid.h"
#include "motor.h"
#include <MsTimer2.h>
#include <FlexCAN.h>
#include <kinetis_flexcan.h>
#include <FastLED.h>

#define DODAI_MIN 20
#define DODAI_MAX 590

#define NUM_LEDS 5
#define DATA_PIN 21
CRGB leds[NUM_LEDS];

FlexCAN CANTransmitter(1000000);
static CAN_message_t dodai_msg;
static CAN_message_t rxmsg;

Motor mot0; //土台
Motor mot1; //上 横
Motor mot2; //上 たて
Motor mot3; //下 横
Motor mot4; //下 たて

int now_dodai = 0; //土台 現在位置 (エンコーダーから)
int v_mot0 = 0; //土台 can値
int v_mot1 = 0; //上 横 can値
int v_mot2 = 0; //上 たて can値
int v_mot3 = 0; //下 横 can値
int v_mot4 = 0; //下 たて can値

void setup() {
  CANTransmitter.begin();
  pinMode(10, INPUT);
  dodai_msg.id = 0x00;
  dodai_msg.len = 8;
  FastLED.addLeds<WS2812B, DATA_PIN, RGB>(leds, NUM_LEDS);
  MsTimer2::set(2, timerInt);
  MsTimer2::start();
  Serial.begin(115200);

  pinMode(1, OUTPUT);
  mot0.init(6, 1);
  pinMode(2, OUTPUT);
  mot1.init(7, 2);
  pinMode(3, OUTPUT);
  mot2.init(8, 3);
  pinMode(4, OUTPUT);
  mot3.init(9, 4);
  pinMode(50, OUTPUT);
  mot4.init(11, 5);
}


void loop() {
  now_dodai = map(analogRead(A0), DODAI_MIN, DODAI_MAX, 0, 600);


  mot0.SetSpeed((int)abs(v_mot0), v_mot0 < 0);
  mot1.SetSpeed((int)abs(v_mot1), v_mot1 < 0);
  mot2.SetSpeed((int)abs(v_mot2), v_mot2 < 0);
  mot3.SetSpeed((int)abs(v_mot3), v_mot3 < 0);
  mot4.SetSpeed((int)abs(v_mot4), v_mot4 < 0);
  mot0.Update();
  mot1.Update();
  mot2.Update();
  mot3.Update();
  mot4.Update();

  delay(5);
}

void timerInt() {
  dodai_msg.buf[0] = now_dodai;
  CANTransmitter.write(dodai_msg);
  while ( CANTransmitter.read(rxmsg) ) {
    if (rxmsg.id == 0x01) {
      v_mot0 = rxmsg.buf[0];
      v_mot1 = rxmsg.buf[1];
      v_mot2 = rxmsg.buf[2];
      v_mot3 = rxmsg.buf[3];
      v_mot4 = rxmsg.buf[4];
    }
  }
  FastLED.show();
}
