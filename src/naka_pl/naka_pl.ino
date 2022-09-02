#include "motor.h"
#include "pid.h"
#include <MsTimer2.h>
#include <FlexCAN.h>
#include <kinetis_flexcan.h>

FlexCAN CANTransmitter(1000000);
static CAN_message_t ue_msg;
static CAN_message_t shita_msg;
static CAN_message_t state_msg;
static CAN_message_t rxmsg;

static unsigned long testch[6];

Motor mot0; //下 たて
Motor mot1; //下 よこ
Motor mot2; //上 テーブル回転

//Pid shita_tate; //下 たて
//Pid shita_yoko; //下 よこ
Pid pid0; //下 テーブルよこ移動
//Pid shita_table_revo; //下 テーブル回転
//Pid up_tate; //上 たて
//Pid up_yoko; //上 よこ
//Pid up_table_revo; //上 テーブル回転

int en0 = 0;
int en1 = 0;

int shita_tate = 0; //下 たて
int shita_yoko = 0; //下 よこ
int up_table_revo = 0; //上 テーブル回転
int shita_vac = 0; //下 吸盤
int up_tate = 0; //上 たて can値
int up_yoko = 0; //上 よこ can値
int up_vac = 0; //上 吸盤 can値
int shita_table_yoko = 0; //下 テーブルよこ移動 can値
int shita_table_yoko_goal = 0; //下 テーブルよこ移動 目標値
int shita_table_revo = 0; //下 テーブル回転 can値
int shita_led = 0; //下 LED色 can値
int robot_state = 0; //0=通常, 9=非常停止
int shita_table_revo_sign = 0; //0=プラス, 1=マイナス
int up_tate_sign = 0;
int up_yoko_sign = 0;
int now_shita_table = 0; //下基盤からのテーブル位置 can値

void setup() {
  CANTransmitter.begin();
  ue_msg.id = 0x03;
  ue_msg.len = 8;
  state_msg.id = 0x00;
  state_msg.len = 8;
  shita_msg.id = 0x04;
  shita_msg.len = 8;
  pinMode(10, INPUT);
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
  pinMode(16, OUTPUT); //電磁弁
  pinMode(17, OUTPUT); //真空モータ
  MsTimer2::set(2, timerInt);
  MsTimer2::start();
  Serial.begin(115200);
  Serial1.begin(100000, SERIAL_8E1);
  pid0.init(13, 0, 0.65);
  pinMode(10, OUTPUT);
  mot0.init(20, 10);
  pinMode(9, OUTPUT);
  mot1.init(21, 9);
  pinMode(8, OUTPUT);
  mot2.init(22, 8);
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

  shita_tate = map(testch[1], 364, 1684, -100, 100); //下 たて
  shita_yoko = map(testch[0], 364, 1684, -255, 255); //下 よこ
  shita_table_yoko_goal = shita_table_yoko_goal + map(testch[1], 364, 1684, -1, 1); //下 テーブルよこ移動
  shita_table_revo = map(testch[0], 364, 1684, -100, 100); //下 テーブル回転
  up_tate = map(testch[3], 364, 1684, -100, 100); //上 たて
  up_yoko = map(testch[2], 364, 1684, -255, 255); //上 よこ
  up_table_revo = map(testch[2], 364, 1684, -100, 100); //上 テーブル回転
  en0 = map(testch[3], 364, 1684, -255, 255); //左 たてY
  en1 = map(testch[1], 364, 1684, -255, 255); //右 たてY

  //上吸盤
  if ((data[5] & 0xC0) >> 6 == 1) {
    if (en0 > 200) {
      up_vac = 1;
    } else if (en0 < -250) {
      up_vac = 0;
    }
    //下吸盤
    if (en1 > 200) {
      shita_vac = 1;
    } else if (en1 < -250) {
      shita_vac = 0;
    }
  }
  //  Serial.println(now_shita_table);
  pid0.now_value(now_shita_table);
  shita_table_yoko_goal = min(max(shita_table_yoko_goal, 25), 575);
  shita_table_yoko = pid0.pid_out(shita_table_yoko_goal);

  shita_msg.buf[2] = shita_led; //下 LED

  mot0.SetSpeed((int)abs(shita_tate), shita_tate < 0);
  mot1.SetSpeed((int)abs(shita_yoko), shita_yoko < 0);
  mot2.SetSpeed((int)abs(up_table_revo), up_table_revo < 0);

  if (robot_state == 9) {
    mot0.SetSpeed(0, 0);
    mot1.SetSpeed(0, 0);
    mot2.SetSpeed(0, 0);
    shita_vac = 0;
  }

  if (shita_table_revo < 0) {
    shita_table_revo = -1 * shita_table_revo;
    shita_table_revo_sign = 1;
  } else {
    shita_table_revo_sign = 0;
  }

  if (up_tate < 0) {
    up_tate = -1 * up_tate;
    up_tate_sign = 1;
  } else {
    up_tate_sign = 0;
  }

  if (up_yoko < 0) {
    up_yoko = -1 * up_yoko;
    up_yoko_sign = 1;
  } else {
    up_yoko_sign = 0;
  }

  ue_msg.buf[5] = up_tate_sign; //上 たて 符号
  ue_msg.buf[6] = up_yoko_sign; //上 よこ 符号
  shita_msg.buf[6] = shita_table_revo_sign; //下 テーブル回転 符号
  
  if ((data[5] & 0x30) >> 4 != 2 || stop_flag == 1) {
    if ((data[5] & 0xC0) >> 6 == 1) {
      Serial.println("Switch: 1");
      mot2.SetSpeed(0, 0); //上 テーブル回転
      shita_msg.buf[0] = 0; //下 テーブルよこ移動
      shita_msg.buf[1] = 0; //下 テーブル回転
      ue_msg.buf[0] = 0; //上 たて
      ue_msg.buf[1] = 0; //上 よこ
      mot0.SetSpeed(0, 0);
      mot1.SetSpeed(0, 0);
      ue_msg.buf[2] = up_vac; //上 吸盤
    } else if ((data[5] & 0xC0) >> 6 == 3) {
      Serial.println("Switch: 3");
      mot2.SetSpeed(0, 0); //上 テーブル回転
      shita_msg.buf[0] = 0; //下 テーブルよこ移動
      shita_msg.buf[1] = 0; //下 テーブル回転
      ue_msg.buf[0] = up_tate; //上 たて
      ue_msg.buf[1] = up_yoko; //上 よこ
      ue_msg.buf[2] = up_vac; //上 吸盤
    } else if ((data[5] & 0xC0) >> 6 == 2) {
      Serial.println("Switch: 2");
      shita_msg.buf[0] = shita_table_yoko; //下 テーブルよこ移動
      shita_msg.buf[1] = shita_table_revo; //下 テーブル回転
      ue_msg.buf[0] = 0; //上 たて
      ue_msg.buf[1] = 0; //上 よこ
      mot0.SetSpeed(0, 0);
      mot1.SetSpeed(0, 0);
      ue_msg.buf[2] = up_vac; //上 吸盤
    }
  } else {
    Serial.println("Switch: OFF");
    shita_msg.buf[0] = 0; //下 テーブルよこ移動
    shita_msg.buf[1] = 0; //下 テーブル回転
    ue_msg.buf[0] = 0; //上 たて
    ue_msg.buf[1] = 0; //上 よこ
    shita_tate = 0; //下 たて
    shita_yoko = 0; //下 よこ
    ue_msg.buf[2] = 0; //上 吸盤
    shita_vac = 0;
    mot0.SetSpeed(0, 0);
    mot1.SetSpeed(0, 0);
    mot2.SetSpeed(0, 0);
  }

  mot0.Update();
  mot1.Update();
  mot2.Update();

  Serial.println(shita_table_revo);
  Serial.println(shita_table_revo_sign);

  if (shita_vac == 1) {
    vac_pick();
  } else if (shita_vac == 0) {
    vac_release();
  }

  delay(5);
}

void timerInt() {
  //  if (analogRead(A0)) {
  //    robot_state = 9;
  //  shita_led = 3;
  //  } else {
  //    robot_state = 0;
  //  shita_led = 0;
  //  }
  CANTransmitter.write(ue_msg);
  CANTransmitter.write(shita_msg);
  CANTransmitter.write(state_msg);
  while ( CANTransmitter.read(rxmsg) ) {
    if (rxmsg.id == 0x01) {
      now_shita_table = rxmsg.buf[0];
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
