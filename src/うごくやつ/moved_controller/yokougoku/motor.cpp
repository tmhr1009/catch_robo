#include <Arduino.h>
#include "motor.h"

Motor::Motor() {
}

void Motor::init(int pwm, int dir) {
  pwm_pin = pwm;
  dir_pin = dir;
  pinMode(dir_pin, OUTPUT);
}

void Motor::Update() {
  digitalWrite(dir_pin, rev ^ rev_param);
  analogWrite(pwm_pin, pwm);
}

void Motor::SetSpeed(int spd, int dir) {
  pwm = min(max(spd, 0), 255);
  rev = dir;
}

void Motor::SetMotor(int rev) {
  rev_param = rev;
}
