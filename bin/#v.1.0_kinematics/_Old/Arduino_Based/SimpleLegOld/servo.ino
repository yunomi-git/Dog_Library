#include <Servo.h>
Servo s_top;
Servo s_bot;
double t;
double ds = 0  ;
double time_to_move = 600;
int state = 1;
int min_top = 1500;
int max_top = 1000;

int max_bot = 2300;
int min_bot = 1200;
void setup() {
  // put your setup code here, to run once:
  s_top.attach(4);
  s_bot.attach(3);
  pinMode(4, OUTPUT);
  pinMode(3, OUTPUT);
  t = millis();
}


void loop() {
  // put your main code here, to run repeatedly:
  if (millis() - t > 5000) {
    t = millis();
    state *= -1;
  }
  int cur_time = millis() - t;
  if (state == 1) {
      if (cur_time <= time_to_move) {
              s_bot.writeMicroseconds(map(cur_time, 0, time_to_move, max_bot, min_bot));
              s_top.writeMicroseconds(map(cur_time, 0, time_to_move, max_top, min_top));
      }

  } else {
      if (cur_time <= time_to_move) {
        s_bot.writeMicroseconds(map(cur_time, 0, time_to_move, min_bot, max_bot));
        s_top.writeMicroseconds(map(cur_time, 0, time_to_move, min_top, max_top));
      }
  }

}
