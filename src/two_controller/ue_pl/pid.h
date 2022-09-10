#ifndef pid_h
#define pid_h
#include <Arduino.h>

class Pid {
  public:
    Pid();
    void init(float p_gain, float i_gain, float d_gain);
    void now_value(int measure);
    int pid_out(int target);
    int debug();
  private:
    float gain[3];
    int in, pre, dif;
    int last_measure;
    int pre_measure[10];
    int measure_tim;
};

#endif
