#include "motor.h"
#include "pid.h"
#include <MsTimer2.h>
#include <FlexCAN.h>
#include <kinetis_flexcan.h>
#define TABLE_MIN 25
#define TABLE_MAX 450

FlexCAN CANTransmitter(1000000);
static CAN_message_t ue_msg;
static CAN_message_t shita_msg;
static CAN_message_t rxmsg;

static unsigned long testch[6];

Motor mot0; //下 たて
Motor mot1; //下 よこ
Motor mot2; //上 テーブル回転

Pid pid0; //下 テーブルよこ移動

int en0 = 0;
int en1 = 0;

int shita_tate = 0; //下 たて
int shita_yoko = 0; //下 よこ
int up_table_revo = 0; //上 テーブル回転
int shita_vac = 0; //下 吸盤
int shita_table_yoko = 0; //下 テーブルよこ移動 can値
int shita_table_yoko_goal = 0; //下 テーブルよこ移動 目標値
int shita_table_revo = 0; //下 テーブル回転 can値
int shita_led = 0; //下 LED色 can値
int shita_table_revo_sign = 0; //0=プラス, 1=マイナス
int shita_table_yoko_sign = 0;
int up_table_revo_sign = 0;
int now_shita_table = 0; //下基盤からのテーブル位置 can値
int robot_stop = 0;
int can_robot_stop = 0; //受信用
int send_can_robot_stop = 0; //送信用

void setup() {
  CANTransmitter.begin();
  ue_msg.id = 0x03;
  ue_msg.len = 8;
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

  //pid
  //  pid0.now_value(now_shita_table);
  //  shita_table_yoko_goal = shita_table_yoko_goal + map(testch[1], 364, 1684, -1, 1); //下 テーブルよこ移動
  //  shita_table_yoko_goal = min(max(shita_table_yoko_goal, TABLE_MIN), TABLE_MAX);
  //  shita_table_yoko = pid0.pid_out(shita_table_yoko_goal);

  //通常コントローラー値受信
  shita_table_yoko = map(testch[1], 364, 1684, -100, 100); //下 テーブルよこ
  shita_tate = map(testch[3], 364, 1684, -100, 100); //下 たて
  shita_yoko = map(testch[2], 364, 1684, -255, 255); //下 よこ
  shita_table_revo = map(testch[0], 364, 1684, -200, 200); //下 テーブル回転
  shita_msg.buf[2] = shita_led; //下 LED

  //  Serial.println(up_table_revo);

  //動作許可
  if ((data[5] & 0xC0) >> 6 == 2) {
    robot_stop = 1;
    send_can_robot_stop = 1;
  } else if ((data[5] & 0xC0) >> 6 == 3) {
    robot_stop = 1;
    send_can_robot_stop = 0;
  } else if ((data[5] & 0xC0) >> 6 == 1) {
    robot_stop = 0;
    send_can_robot_stop = 0;
  }
  ue_msg.buf[3] = send_can_robot_stop;

  //コントローラーからの吸盤操作
  if ((data[5] & 0x30) >> 4 == 1) {
    shita_vac = 1;
  } else if ((data[5] & 0x30) >> 4 == 2) {
    shita_vac = 0;
  }

  //CANで送る送られる値の符号変換
  if (up_table_revo_sign == 1) up_table_revo = up_table_revo * -1;
  if (shita_table_revo < 0) {
    shita_table_revo = -1 * shita_table_revo;
    shita_table_revo_sign = 1;
  } else {
    shita_table_revo_sign = 0;
  }
  if (shita_table_yoko < 0) {
    shita_table_yoko = -1 * shita_table_yoko;
    shita_table_yoko_sign = 1;
  } else {
    shita_table_yoko_sign = 0;
  }

  //主動作部分
  //コントローラーからSTOP or CANからSTOP or コントローラー受信してないSTOP
  if (can_robot_stop == 1 || robot_stop == 1 || stop_flag == 1) {
    Serial.println("STOP");
    shita_vac = 0;
    shita_msg.buf[0] = 0; //下 テーブル移動
    shita_msg.buf[1] = 0; //下 テーブル回転
    shita_msg.buf[5] = 0; //下 テーブル移動 符号
    shita_msg.buf[6] = 0; //下 テーブル回転 符号
    mot0.SetSpeed(0, 0);
    mot1.SetSpeed(0, 0);
    mot2.SetSpeed(0, 0);
  } //動作可能
  else if (can_robot_stop == 0 && robot_stop == 0) {
    shita_msg.buf[0] = shita_table_yoko; //下 テーブル移動
    shita_msg.buf[1] = shita_table_revo; //下 テーブル回転
    shita_msg.buf[5] = shita_table_yoko_sign; //下 テーブル移動 符号
    shita_msg.buf[6] = shita_table_revo_sign; //下 テーブル回転 符号
    mot0.SetSpeed((int)abs(shita_tate), shita_tate < 0);
    mot1.SetSpeed((int)abs(shita_yoko), shita_yoko < 0);
    mot2.SetSpeed((int)abs(up_table_revo), up_table_revo < 0);
  }

  //吸盤動作
  if (shita_vac == 1) {
    vac_pick();
  } else if (shita_vac == 0) {
    vac_release();
  }

  //モーター動作指示
  mot0.Update();
  mot1.Update();
  mot2.Update();

  delay(5);
}

void timerInt() {
  CANTransmitter.write(ue_msg);
  CANTransmitter.write(shita_msg);
  while ( CANTransmitter.read(rxmsg) ) {
    //下基盤から
    if (rxmsg.id == 0x01) {
      now_shita_table = rxmsg.buf[0];
    }
    //上基盤から
    if (rxmsg.id == 0x06) {
      up_table_revo = rxmsg.buf[0];
      up_table_revo_sign = rxmsg.buf[5];
      can_robot_stop = rxmsg.buf[3];
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
