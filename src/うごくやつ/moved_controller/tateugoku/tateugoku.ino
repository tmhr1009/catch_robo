#include "pid.h"
#include "motor.h"

Motor mot0;
Motor mot1;

float en1 = 0;
float en2 = 0;

void setup() {
  pinMode(10, OUTPUT);
  mot0.init(20, 10);
  pinMode(9, OUTPUT);
  mot1.init(21, 9);
  Serial.begin(115200);
  Serial1.begin(100000, SERIAL_8E1);
}

static unsigned long testch[6];

//goal min=25, max=575 まで
void loop() {
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
  }
  else {
  }

  en1 = map(testch[1], 364, 1684, -255, 255);
  en2 = map(testch[0], 364, 1684, -500, 500);


  if (((data[5] & 0xC0) >> 6) == 2) {
    en1 = 0;
    en2 = 0;
  }

  Serial.println(en2);

  mot0.SetSpeed((int)abs(en1), en1 < 0);
  mot0.Update();
  mot1.SetSpeed((int)abs(en2), en2 < 0);
  mot1.Update();

  delay(5);
}
