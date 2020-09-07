// Bring out the paperclip and protractor...
// run and type in desired signal to send. change channel to appropriate channel

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

int test_signals[19] = {1420,1505,1600,1720,1830,1950,2060,2200,2320,2440,2550,2660,2775,2890,3000,3120,3240,3349,3459};
int NUM_SIG = 15;

int sig;
int channel = 1; // <--- change channel

void setup() {
  Serial.begin(9600);
  while ( !Serial ) ;
  Serial.println("16 channel PWM test!");

  pwm.begin();
  pwm.setPWMFreq(400);  // This is the maximum PWM frequency

  Wire.setClock(400000);
  pwm.setPWM(channel, 0, 2440); // approximate 0 deg
//  631, 3612
// 2440 is 0, 3460 +90, 1400 -90. dog has 0 at 2455

}

void loop() {
  if (Serial.available()) {
    sig = Serial.parseInt();
    if (sig > 0) {
       pwm.setPWM(channel, 0, sig ); // pwm.setPWM(channel, [begin_time], [end_time])
       Serial.println(sig);
    }
    if (sig == -10) {
      for (int i = 0; i < NUM_SIG; i++) {
        sig = test_signals[i];
        pwm.setPWM(channel, 0, sig ); // pwm.setPWM(channel, [begin_time], [end_time])
        Serial.println(sig);
        delay(1000);
      }
    }
  }
}
