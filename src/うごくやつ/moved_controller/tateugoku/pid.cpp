#include "pid.h"
#include <Arduino.h>

Pid::Pid() {
  for (int i = 0; i < 3; i++)gain[i] = 0;
  in = 0;
  pre = 0;
  dif = 0;
  last_measure = 0;
  measure_tim = 0;
}

void Pid::init(float p_gain, float i_gain, float d_gain) {
  gain[0] = p_gain;
  gain[1] = i_gain;
  gain[2] = d_gain;
}

void Pid::now_value(int measure) {
  if (measure > 32767)measure = measure - 65535;
  last_measure = measure;
  pre_measure[measure_tim % 10] = measure;
  measure_tim++;
}

int Pid::pid_out(int target) {
  pre = in;
  in = last_measure;
  dif += target - last_measure;
  return (int)(gain[0] * (target - in) + gain[1] * dif  + gain[2] * (in - pre));
}

int Pid::debug() {
  int remeasure = 0;
  for (int i = 0; i < 10; i++) {
    remeasure = remeasure + pre_measure[i];
  }
  return remeasure / 10;
}
