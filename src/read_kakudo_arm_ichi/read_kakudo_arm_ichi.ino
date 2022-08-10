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

int now_dodai = 0; //土台 現在位置 (エンコーダーから)

void setup() {
  CANTransmitter.begin();
  pinMode(10, INPUT);
  dodai_msg.id = 0x00;
  dodai_msg.len = 8;
  FastLED.addLeds<WS2812B, DATA_PIN, RGB>(leds, NUM_LEDS);
  MsTimer2::set(2, timerInt);
  MsTimer2::start();
  Serial.begin(115200);
}

void loop() {
  now_dodai = map(analogRead(A0), DODAI_MIN, DODAI_MAX, 0, 600);

  dodai_msg.buf[0] = now_dodai;
  delay(5);
}

void timerInt() {
  CANTransmitter.write(dodai_msg);
  FastLED.show();
}
