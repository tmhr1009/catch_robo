#include "motor.h"
#include "pid.h"
#include <MsTimer2.h>
#include <FlexCAN.h>
#include <kinetis_flexcan.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

FlexCAN CANTransmitter(1000000);
static CAN_message_t rxmsg;
static CAN_message_t uenaka_msg;
static CAN_message_t ueshita_msg;
Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28);

Motor mot0; //上 たて
Motor mot1; //上 よこ
Motor mot2; //上 テーブル回転


Pid pid0; //上 テーブル回転

int up_tate = 0; //上 たて
int up_yoko = 0; //上 よこ
int up_vac = 0; //上 吸盤
float up_table_revo = 0; //上 テーブル回転 can値
int robot_stop = 0;
int can_robot_stop = 0; //受信用
int send_can_robot_stop = 0; //送信用
int read_flag = 0;
int gyro_x = 0;
int save_v = 0;
int gyro_goal = 0;
int timers = 0;

static unsigned long testch[6];

void setup() {
  CANTransmitter.begin();
  uenaka_msg.id = 0x06;
  uenaka_msg.len = 8;
  ueshita_msg.id = 0x08;
  ueshita_msg.len = 8;
  pinMode(13, OUTPUT);
  pinMode(10, INPUT);
  pinMode(5, OUTPUT); //ブザー
  MsTimer2::set(2, timerInt);
  MsTimer2::start();
  Serial.begin(115200);
  Serial1.begin(100000, SERIAL_8E1);
  pinMode(16, OUTPUT); //電磁弁
  pinMode(17, OUTPUT); //真空モータ
  pinMode(10, OUTPUT);
//  pid0.init(6, 0.0001, 0.001);
  pid0.init(6.5, 0, 0.001);
  mot0.init(20, 10);
  pinMode(9, OUTPUT);
  mot1.init(21, 9);
  pinMode(8, OUTPUT);
  mot2.init(22, 8);
  if (!bno.begin())
  {
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    digitalWrite(5, HIGH);
    while (1);
  }
}


void loop() {
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  int gyro_x = euler.x();
  pid0.now_value(gyro_x);
  Serial.print(gyro_x);
  Serial.print("   :    ");
  Serial.println(save_v);
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
    mot0.SetSpeed(0, 0);
    mot1.SetSpeed(0, 0);
    mot2.SetSpeed(0, 0);
  }

  //コントローラー値受信
  up_tate = map(testch[3], 364, 1684, -100, 100); //上 たて
  up_yoko = map(testch[2], 364, 1684, -255, 255); //上 よこ
  up_table_revo = map(testch[0], 364, 1684, -1, 1); //上 テーブル回転
//  up_table_revo = (float(testch[0])-1024)/100;
  if (up_table_revo == 1 || up_table_revo == -1) {
    if (timers > 1) {
      timers = 0;
    } else if (timers < 1) {
      up_table_revo = 0;
    }
  }
  timers++;

  //動作許可
  if (stop_flag != 1) {
    if ((data[5] & 0xC0) >> 6 == 2) {
      Serial.println("STOP");
      robot_stop = 1;
      send_can_robot_stop = 1;
      gyro_goal = gyro_x;
    } else if ((data[5] & 0xC0) >> 6 == 3) {
      robot_stop = 1;
      send_can_robot_stop = 0;
      gyro_goal = gyro_x;
    } else if ((data[5] & 0xC0) >> 6 == 1) {
      robot_stop = 0;
      send_can_robot_stop = 0;
    }
  }
  uenaka_msg.buf[3] = send_can_robot_stop;
  ueshita_msg.buf[3] = send_can_robot_stop;

  //コントローラーからの吸盤操作
  if ((data[5] & 0x30) >> 4 == 1) {
    up_vac = 1;
  } else if ((data[5] & 0x30) >> 4 == 2) {
    up_vac = 0;
  }

  gyro_goal = up_table_revo + gyro_goal;

  //主動作部分
  //コントローラーからSTOP or CANからSTOP or コントローラー受信してないSTOP
  if (stop_flag != 1) {
    if (can_robot_stop == 1 || robot_stop == 1) {
      //    Serial.println("STOP");
      up_vac = 0;
      mot0.SetSpeed(0, 0);
      mot1.SetSpeed(0, 0);
      mot2.SetSpeed(0, 0);
    } //動作可能
    else if (can_robot_stop == 0 && robot_stop == 0) {

      if (gyro_goal > 360)gyro_goal = gyro_goal - 360;
      if (gyro_goal < 0)gyro_goal = gyro_goal + 360;
      if (gyro_goal - gyro_x > 180) {
        up_table_revo = pid0.pid_out(gyro_goal - 360);
      }
      else if (gyro_goal - gyro_x < -180) {
        up_table_revo = pid0.pid_out(gyro_goal + 360);
      }
      else {
        up_table_revo = pid0.pid_out(gyro_goal);
      }

      //      up_table_revo = pid0.pid_out(gyro_goal);
      up_table_revo = min(max(up_table_revo, -100), 100);
      mot0.SetSpeed((int)abs(up_tate), up_tate > 0);
      mot1.SetSpeed((int)abs(up_yoko), up_yoko > 0);
      mot2.SetSpeed((int)abs(up_table_revo), up_table_revo < 0);
    }
  }
  Serial.print("pidout: ");
  Serial.println(up_table_revo);
  Serial.print("gyro_goal: ");
  Serial.println(gyro_goal);

  save_v = gyro_x;

  //吸盤動作
  if (up_vac == 1) {
    vac_pick();
  } else if (up_vac == 0) {
    vac_release();
  }

  //モーター動作指示
  mot0.Update();
  mot1.Update();
  mot2.Update();

  delay(25);
}

void timerInt() {
  CANTransmitter.write(uenaka_msg);
  while ( CANTransmitter.read(rxmsg) ) {
    //中基盤から
    if (rxmsg.id == 0x03) {
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
