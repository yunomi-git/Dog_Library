/*
// # Description:
// # The sketch for using the gamepad and print the button state and the analog values of the gamepad
// # Enable the vibration function
// #   Send 'v' via the Arduino Serial monitor to enable the vibration
// #   Send 's' via the Arduino Serial monitor to stop it
 
*/
//#define DEBUG

struct button_info {
  String name;
  int pin;
  bool state;
  bool prev_state;
  bool is_sticky;
};

#define NUM_DB 14
// 3-4: Select, Start | 5-8: U L D R | 9-12: 1423 | 13-16: 
button_info dig_button[NUM_DB]={{"SELECT",   3, 0, 0, 0},
                                {"START",    4, 0, 0, 0},
                                {"UP",       5, 0, 0, 0},
                                {"LEFT",     6, 0, 0, 0},
                                {"DOWN",     7, 0, 0, 0},
                                {"RIGHT",    8, 0, 0, 0},
                                {"B1",       9, 0, 0, 0},
                                {"B4",      10, 0, 0, 0},
                                {"B2",      11, 0, 0, 0},
                                {"B3",      12, 0, 0, 0},
                                {"RZ1",     13, 0, 0, 0},
                                {"RZ2",     14, 0, 0, 1},
                                {"LZ1",     15, 0, 0, 0},
                                {"LZ2",     16, 0, 0, 1}};
#define NUM_AB 2            
button_info ana_button[NUM_AB]={{"JBL",      0, 0, 0, 1},
                                {"JBR",      1, 0, 0, 1}};
struct joystick_info {
  String name;
  int pin;
  double state;
  double raw_offset;
  double direction;
};

// RX:2 RY:3 LX:4 LY:5                   
joystick_info joystick[4] = {{"JRY", 2, 0, -2.5, 1},
                             {"JRX", 3, 0, -10.5, -1},
                             {"JLY", 4, 0, -6.5, 1},
                             {"JLX", 5, 0, -.5, -1}};

int inputCommand = 0;

#define virbrationMotorPin 2
#define JOYSTICK_MID 511.5
#define JOYSTICK_RANGE 511.5

void setup()
{
  Serial.begin(9600);  //Init the Serial baudrate
  Serial1.begin(19200);  //Init the Serial1 port to enable the xbee wireless communication
  InitIO();             //Initialize the inputs/outputs and the buffers
}

void InitIO(){ 
  for(int i = 0; i < 17; i++) pinMode(i, INPUT); 
  pinMode(virbrationMotorPin,OUTPUT);
  digitalWrite(virbrationMotorPin,LOW);  // Stop shacking of the gamepad
}

unsigned long timer = 0;
#define UPDATE_RATE 30
void loop()
{ 
  if(millis() - timer > UPDATE_RATE){  // manage the updating freq of all the controlling information
    DataUpdate();   //print the datas and states
    printData();  //read the buttons and the joysticks data
    timer = millis(); 
  }
//  
//  if(Serial.available()){
//    char input = Serial.read();
//    
//    switch(input){
//      case 'v':
//        Serial.println("Vibration");
//        inputCommand = input;
//        digitalWrite(virbrationMotorPin,HIGH);
//        break;
//      
//      case 's':
//        Serial.println("Stop");
//        inputCommand = input;
//        digitalWrite(virbrationMotorPin,LOW);
//        break;
//        
//      default:
//        break;
//    }
//  }
}

double sign(double x) {
  if (x >= 0) return 1;
  return -1;
}

double joy_val;

void DataUpdate(){
  for(int i = 0; i < NUM_DB; i++) {
    dig_button[i].prev_state = dig_button[i].state;
    dig_button[i].state = !digitalRead(dig_button[i].pin); 
  }
  for(int i = 0; i < NUM_AB; i++) {
    ana_button[i].prev_state = ana_button[i].state;
    ana_button[i].state = !analogRead(ana_button[i].pin);
  }
  for(int i = 0; i < 4; i++) {
    joy_val = joystick[i].direction * (analogRead(joystick[i].pin)- JOYSTICK_MID - joystick[i].raw_offset)/JOYSTICK_RANGE;
    if (joy_val > 1)        joy_val = 1;
    else if (joy_val < -1)  joy_val = -1;
    joystick[i].state = sign(joy_val) * joy_val*joy_val; // Square to smooth out signal
  }
}

void printData(){
  // Joystick
//  Serial1.print(">");
  for(int i = 0; i < 4; i++) {
    Serial1.print(joystick[i].state, 3), Serial1.print(" ");
  }
  Serial1.print("|");
  
  // Digital Buttons
  for(int i = 0; i < NUM_DB; i++) {
    if (dig_button[i].is_sticky) { 
      // Print so long as button is held down
      if (dig_button[i].state) Serial1.print(dig_button[i].name), Serial1.print(" ");
    } else { 
      // Print only when state changes from false->true
      if (!dig_button[i].prev_state && dig_button[i].state) Serial1.print(dig_button[i].name), Serial1.print(" ");
    }
  }
  // Analog Buttons
  for(int i = 0; i < NUM_AB; i++) {
    if (ana_button[i].is_sticky) { 
      // Print so long as button is held down
      if (ana_button[i].state) Serial1.print(ana_button[i].name), Serial1.print(" ");
    } else { 
      // Print only when state changes from false->true
      if (!ana_button[i].prev_state && ana_button[i].state) Serial1.print(ana_button[i].name), Serial1.print(" ");
    }
  }
  Serial1.println();

#ifdef DEBUG
    // Joystick
  for(int i = 0; i < 4; i++) {
    Serial.print(joystick[i].state, 3), Serial.print(" ");
  }
  Serial.print("|");
  
  // Digital Buttons
  for(int i = 0; i < NUM_DB; i++) {
    if (dig_button[i].is_sticky) { 
      // Print so long as button is held down
      if (dig_button[i].state) Serial.print(dig_button[i].name), Serial.print(" ");
    } else { 
      // Print only when state changes from false->true
      if (!dig_button[i].prev_state && dig_button[i].state) Serial.print(dig_button[i].name), Serial.print(" ");
    }
  }
  // Analog Buttons
  for(int i = 0; i < NUM_AB; i++) {
    if (ana_button[i].is_sticky) { 
      // Print so long as button is held down
      if (ana_button[i].state) Serial.print(ana_button[i].name), Serial.print(" ");
    } else { 
      // Print only when state changes from false->true
      if (!ana_button[i].prev_state && ana_button[i].state) Serial.print(ana_button[i].name), Serial.print(" ");
    }
  }
  Serial.println();
#endif
}
