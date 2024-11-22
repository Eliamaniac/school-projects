#include <MeAuriga.h>

#define LEDNUM  12
#define LEDPIN  44
#define RINGALLLEDS 0
#define cw 0
#define ccw 1
#define motorStop 2

MeRGBLed led( PORT0, LEDNUM );
MeUltrasonicSensor ultraSensor(PORT_8);

enum State {run, slow, stop, back, turn};
State currentState = run; 

float modifier = 0.7;// 70% 
int maxPwm = 255*modifier;
int halfPwm = 125*modifier;
int turnPwm = 150*modifier;

//Motor Left
const int m1_pwm = 11;
const int m1_in1 = 48; // M1 ENA
const int m1_in2 = 49; // M1 ENB

//Motor Right
const int m2_pwm = 10;
const int m2_in1 = 47; // M2 ENA
const int m2_in2 = 46; // M2 ENB

unsigned long currentTime = millis();
int printRate = 250;
int exitTime = 500;
int turnTime = 1600;

void setup() {
    //Set Pwm 8KHz
  TCCR1A = _BV(WGM10);
  TCCR1B = _BV(CS11) | _BV(WGM12);

  TCCR2A = _BV(WGM21) | _BV(WGM20);
  TCCR2B = _BV(CS21);
  
  led.setpin(LEDPIN); // OBLIGATOIRE Configuration de la broche

  Serial.begin(9600);

  pinMode(m1_pwm, OUTPUT);  
  pinMode(m1_in2, OUTPUT);  
  pinMode(m1_in1, OUTPUT);

  pinMode(m2_pwm, OUTPUT);  
  pinMode(m2_in2, OUTPUT);  
  pinMode(m2_in1, OUTPUT);
}

void loop() {
  currentTime = millis();
  serialManager();
  stateManager();
}

void setMotor(int direction, float modifier) {
  int speed = maxPwm * modifier;
  int correction = speed*0.25; //wheel speed correction
  if (direction) {
    //ccw
    digitalWrite(m1_in1, LOW);
    digitalWrite(m1_in2, HIGH);
    digitalWrite(m2_in1, LOW);
    digitalWrite(m2_in2, HIGH);
  } else {
    //cw
    digitalWrite(m1_in1, HIGH);
    digitalWrite(m1_in2, LOW);
    digitalWrite(m2_in1, HIGH);
    digitalWrite(m2_in2, LOW);
  }
  analogWrite(m1_pwm, speed+correction); //right side correction
  analogWrite(m2_pwm, speed);
}

void setMotor(int direction, float modifier, int motor) { // fonction surchargée
  int speed = maxPwm * modifier;
  int in1 = motor ? m2_in1 : m1_in1;
  int in2 = motor ? m2_in2 : m1_in2;
  int pwm = motor ? m2_pwm : m1_pwm;
  if (direction == 1) {
    //ccw
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  } else if (direction == 0) {
    //cw
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  }
  analogWrite(pwm, speed);
}


void runState(){
  int dist = readDist();
  static bool firstTime = true; 
  if (dist <= 100 && dist >= 30) {
    ledTask(0x000000);
    currentState = slow;
    firstTime=true;
    return;
  }
  else if (dist <= 30) {
    ledTask(0x000000);
    currentState = stop;
    firstTime=true;
    return;
  }
  if (firstTime) {
    // Remplacer ça par setmotor 1 pour debugger, puis slow
    setMotor(cw,1.0); //direction : cw, speed : 100%
    led.setColor(0, 0);
    ledTask(0x000100);
    firstTime = false;
  }
}

void slowState() {
  int dist = readDist();
  static bool firstTime = true; 
  if (dist >= 100) {
    ledTask(0x000000);
    currentState = run;
    firstTime=true;
    return;
  }
  else if (dist <= 30) {
    ledTask(0x000000);
    currentState = stop;
    firstTime=true;
    return;
  }
  if (firstTime) {
    setMotor(cw,0.5); //direction : cw, speed : 50%
    led.setColor(0, 0);
    ledTask(0x010100);
    firstTime = false;
  }
}

void stopState() {
  static bool firstTime = true; 
  static unsigned long lastTime = currentTime;
  if (firstTime) {
    setMotor(cw,0.0); //direction : cw, speed : 0%
    led.setColor(0, 0x020000);
    led.show();
    lastTime = currentTime;
    firstTime = false;
  }
  if(currentTime-lastTime>=exitTime) {
    currentState = back;
    firstTime = true;
    return;
  }
}

void backState() {
  static bool firstTime = true; 
  static unsigned long lastTime = currentTime;
  if (firstTime) {
    setMotor(ccw,0.8); //direction : ccw, speed : 80%
    lastTime = currentTime;
    firstTime = false;
  }
  if(currentTime-lastTime>=exitTime) {
    currentState = turn;
    firstTime = true;
    return;
  }
}

void turnState() {
  static bool firstTime = true; 
  static unsigned long lastTime = currentTime;
  currentTime = millis();
  if (firstTime) {
    setMotor(cw, 0.0); // stop
    setMotor(ccw,0.7,0);
    setMotor(cw,0.5,1);
    lastTime = currentTime;
    firstTime = false;
  }
  if(currentTime-lastTime>=turnTime) {
    currentState = run;
    firstTime = true;
    return;
  }
}

int readDist() {
  currentTime = millis();
  static unsigned long lastTime = 0;
  static int dist = 150;
  if (currentTime-lastTime >= printRate) {
    lastTime = currentTime;
    dist = ultraSensor.distanceCm();
  }
  return dist; 
}

void serialManager() {
  currentTime = millis();
  static unsigned long lastTime = 0;
  if (currentTime-lastTime >= printRate) {
    lastTime=currentTime;
    Serial.print("Distance : ");
    Serial.print(ultraSensor.distanceCm() );
    Serial.println(" cm");
    Serial.print("Current state : ");
    Serial.println(currentState);
  }
}

void stateManager() {
  switch(currentState) {
    case run:
      runState();
      break;
    case slow:
      slowState();
      break;
    case stop:
      stopState();
      break;
    case back:
      backState();
      break;
    case turn:
      turnState();
      break;
    default: currentState = stop;
  }
}

void ledTask(unsigned long color) {
  short idx = 1; // 0 = anneau complet
  // static unsigned long lastTime = 0;
  // int rate = 100;

  // if (currentTime - lastTime < rate) return;
  
  // lastTime = currentTime;
  // led.setColor(100, 100, 0); // Configure la couleur jaune
  for(int i = 1;i<=6;i++) {
    led.setColor(idx, color);
    idx = idx >= LEDNUM ? 1 : idx + 1;
  }
  
  // idx = idx >= LEDNUM ? 1 : idx + 1;
  
  led.show(); // Active l'anneau avec la couleur  
}
