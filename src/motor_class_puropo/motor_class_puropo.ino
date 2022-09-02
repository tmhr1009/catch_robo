#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <FlexCAN.h>


void setup() {
  Serial.begin(115200);
  Serial1.begin(100000, SERIAL_8E1);
  pinMode(10, INPUT);
}

static unsigned long testch[6];

void loop() {
  static int data[18];
  static int dataNumber = 0;
  static unsigned long lastConnectTime = 0;
  if (Serial1.available() > 0) {
    digitalWrite(13, HIGH);
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
  }
  else {
    digitalWrite(13, LOW);
  }

  Serial.print(testch[0]);
  Serial.print(",");
  Serial.print((data[5] & 0xC0) >> 6);
  Serial.print(",");
  Serial.println((data[5] & 0x30) >> 4);
  Serial.print("joyRightX : ");
  Serial.print(testch[0]);
  Serial.println(",");
  Serial.print("joyRightY : ");
  Serial.print(testch[1]);
  Serial.println(",");
  Serial.print("joyLeftX : ");
  Serial.print(testch[2]);
  Serial.println(",");
  Serial.print("joyLeftY : ");
  Serial.print(testch[3]);
  Serial.println(",");
  Serial.print("toggleL : ");
  Serial.print((data[5] & 0xC0) >> 6);
  Serial.println(",");
  Serial.print("toggleR : ");
  Serial.println((data[5] & 0x30) >> 4);
  Serial.println();
  delay(5);
}
