#include "motor.h"
#include <MsTimer2.h>
#include <FlexCAN.h>
#include <kinetis_flexcan.h>

FlexCAN CANTransmitter(1000000);
static CAN_message_t rxmsg;

Motor mot0; //上 たて
Motor mot1; //上 よこ

int up_tate = 0; //上 たて can値
int up_yoko = 0; //上 よこ can値
int up_vac = 0; //上 吸盤 can値
int online_vac = 0; //前回の指示値を保存

void setup() {
  CANTransmitter.begin();
  pinMode(10, INPUT);
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
  mot0.SetSpeed((int)abs(up_tate), up_tate < 0);
  mot1.SetSpeed((int)abs(up_yoko), up_yoko < 0);
  mot0.Update();
  mot1.Update();

  if (up_vac != online_vac) {
    if (up_vac == 1) {
      vac_pick();
      online_vac = up_vac;
    } else if (up_vac == 0) {
      vac_release();
      online_vac = up_vac;
    }
  }

  delay(5);
}

void timerInt() {
  while ( CANTransmitter.read(rxmsg) ) {
    if (rxmsg.id == 0x03) {
      up_tate = rxmsg.buf[0];
      up_yoko = rxmsg.buf[1];
      up_vac = rxmsg.buf[2];
    }
  }
}

//吸盤吸い込み vac_pick
void vac_pick() {
  digitalWrite(6, LOW);
  digitalWrite(7, HIGH);
}

//吸盤脱力 vac_release
void vac_release() {
  digitalWrite(6, HIGH);
  digitalWrite(7, LOW);
  digitalWrite(6, LOW);
  digitalWrite(7, LOW);
}
