#include "motor.h"
#include <MsTimer2.h>
#include <FlexCAN.h>
#include <kinetis_flexcan.h>

FlexCAN CANTransmitter(1000000);
static CAN_message_t rxmsg;

Motor mot0; //上 たて
Motor mot1; //上 よこ

static unsigned long testch[6];

int up_tate = 0; //上 たて can値
int up_yoko = 0; //上 よこ can値
int up_vac = 0; //上 吸盤 can値
int up_tate_sign = 0;
int up_yoko_sign = 0;
int robot_state = 0;

void setup() {
  CANTransmitter.begin();
  pinMode(13, OUTPUT);
  pinMode(10, INPUT);
  MsTimer2::set(2, timerInt);
  MsTimer2::start();
  Serial.begin(115200);
  Serial1.begin(100000, SERIAL_8E1);
  pinMode(16, OUTPUT); //電磁弁
  pinMode(17, OUTPUT); //真空モータ
  pinMode(10, OUTPUT);
  mot0.init(20, 10);
  pinMode(9, OUTPUT);
  mot1.init(21, 9);
}


void loop() {
  int stop_flag = 0;
  static int data[18];                      //入力データが入る？
  static int dataNumber = 0;                //入力データの数(Serial1.available()の返値)
  static unsigned long lastConnectTime = 0; //直前の通信の時間?
  if (Serial1.available() > 0) {
    for (int dataNum = Serial1.available(); dataNum > 0; dataNum--) {
      if (dataNumber < 0) {
        Serial1.read();
        dataNumber++;
        continue;
      }
      data[dataNumber % 18] = Serial1.read();
      dataNumber++;
      if (dataNumber > 18) {
        dataNumber = 0;
      }
      else if (dataNumber == 18) {
        testch[0] = (((data[1] & 0x07) << 8) | data[0]);          //ch0(364～1024～1684)
        testch[1] = (((data[2] & 0x3F) << 5) | (data[1] >> 3));   //ch1(364～1024～1684)
        testch[2] = (((data[4] & 0x01) << 10) | (data[3] << 2) | (data[2] >> 6)); //ch2(364～1024～1684)
        testch[3] = (((data[5] & 0x0F) << 7) | (data[4] >> 1));   //ch3(364～1024～1684)
        if (!(364 <= testch[0] && testch[0] <= 1684 && 364 <= testch[1] && testch[1] <= 1684 && 364 <= testch[2] && testch[2] <= 1684 && 364 <= testch[3] && testch[3] <= 1684)) {
          for (int i = 1; i < 18; i++) {
            testch[0] = (((data[(1 + i) % 18] & 0x07) << 8) | data[(0 + i) % 18]);  //ch0(364～1024～1684)
            testch[1] = (((data[(2 + i) % 18] & 0x3F) << 5) | (data[(1 + i) % 18] >> 3)); //ch1(364～1024～1684)
            testch[2] = (((data[(4 + i) % 18] & 0x01) << 10) | (data[(3 + i) % 18] << 2) | (data[2] >> 6)); //ch2(364～1024～1684)
            testch[3] = (((data[(5 + i) % 18] & 0x0F) << 7) | (data[(4 + i) % 18] >> 1)); //ch3(364～1024～1684)
            if (364 <= testch[0] && testch[0] <= 1684 && 364 <= testch[1] && testch[1] <= 1684 && 364 <= testch[2] && testch[2] <= 1684 && 364 <= testch[3] && testch[3] <= 1684) {
              dataNumber = -i;
              break;
            }
          }
          if (dataNumber > 18) {
            dataNumber = -1;
          }
        }
        else {
          dataNumber = 0;
        }
      }
    }
    digitalWrite(13, !digitalRead(13)); //プロポ受信したらLEDチカチカ
    stop_flag = 0;
  }
  else {
    stop_flag = 1;
  }


  if (up_tate_sign == 1) {
    up_tate = up_tate * -1;
  }

  if (up_yoko_sign == 1) {
    up_yoko = up_yoko * -1;
  }

  Serial.print("up_vac: ");
  Serial.println(up_vac);
  Serial.print("up_tate: ");
  Serial.println(up_tate);
  Serial.print("up_yoko: ");
  Serial.println(up_yoko);
  mot0.SetSpeed((int)abs(up_tate), up_tate > 0);
  mot1.SetSpeed((int)abs(up_yoko), up_yoko < 0);

  if (robot_state == 9) {
    mot0.SetSpeed(0, 0);
    mot1.SetSpeed(0, 0);
    up_vac = 0;
  }

  mot0.Update();
  mot1.Update();

  if (up_vac == 1) {
    vac_pick();
  } else if (up_vac == 0) {
    vac_release();
  }

  delay(5);
}

void timerInt() {
  while ( CANTransmitter.read(rxmsg) ) {
    if (rxmsg.id == 0x00) {
      robot_state = rxmsg.buf[0];
    }
    if (rxmsg.id == 0x03) {
      up_tate = rxmsg.buf[0];
      up_yoko = rxmsg.buf[1];
      up_vac = rxmsg.buf[2];
      up_tate_sign = rxmsg.buf[5];
      up_yoko_sign = rxmsg.buf[6];
    }
  }
}

//吸盤吸い込み vac_pick
void vac_pick() {
  digitalWrite(16, LOW);
  digitalWrite(17, HIGH);
}

//吸盤脱力 vac_release
void vac_release() {
  digitalWrite(16, HIGH);
  digitalWrite(17, LOW);

}
