#include "pid.h"
#include "motor.h"
#define MIN 20
#define MAX 590

Pid pid0;
Motor mot0;

float en1 = 0;
int goal = 0;
float vx = 0;

void setup() {
  pinMode(10, OUTPUT);
  mot0.init(20, 10);
  pid0.init(13, 0, 0.65);
  Serial.begin(115200);
  goal = 500;
}


//goal min=25, max=575 まで
void loop() {
  en1 = map(analogRead(A0), MIN, MAX, 0, 600);

  goal = min(max(goal, 25), 575);

  pid0.now_value(en1);
  vx = pid0.pid_out(goal);

  Serial.print(en1);
  Serial.print(",");
  Serial.println(goal);

  mot0.SetSpeed((int)abs(vx), vx < 0);
  mot0.Update();

  delay(5);
}
