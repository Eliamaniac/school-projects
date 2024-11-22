#include <Adafruit_seesaw.h>
#include <MeAuriga.h>

#define NB_IR 5
#define PULSE 9
#define RATIO 39.267
#define LEDNUM  12
#define LEDPIN  44

int seuil = 600;  // Seuil de détection de la ligne
int valMin = 1023;
int valMax = 0;
int speed = 150;
const int nbCapteurs = 5;
int sensorValues[NB_IR];
int sensorNormalized[NB_IR];
unsigned long currentTime = millis();
short lastSensor = 2;

Adafruit_seesaw ss;
MeGyro gyro(0, 0x69);

MeEncoderOnBoard encoderRight(SLOT1);
MeEncoderOnBoard encoderLeft(SLOT2);

MeRGBLed led( PORT0, LEDNUM );
MeUltrasonicSensor ultraSensor(PORT_8);

void setup() {
  gyro.begin();
  EncoderConfig();
  led.setpin(LEDPIN);
  Serial.begin(115200);

  if (!ss.begin()) {
    Serial.println("Erreur de connexion au LyneTracker");
    while (1)
      ;
  }
  Serial.println("Connexion réussie au LyneTracker! En cours de calibrage");
  unsigned long lastTime = currentTime;
  while (currentTime-lastTime<3000) {
    currentTime = millis();
    Calibrate();
  }
}

void loop() {
  currentTime = millis();
  Calibrate();
  ReadValues();
  gyro.update();
  // Normaliser les valeurs des capteurs
  // normaliser();

  // // Calculer la position de la ligne
  // float position = computePosition();

  // // Calculer l'ajustement à apporter à la trajectoire
  // float adjustment = computePID(position);

  // Ajuster la trajectoire du robot en fonction de l'ajustement
  // Par exemple, ajuster la vitesse des moteurs
  float adjustedSpeed = speed;
  Follow(adjustedSpeed);
  SerialTask();
  EncodersTask();
}

void EncodersTask() {
  encoderRight.loop();
  encoderLeft.loop();
}

void EncoderConfig() {
  encoderRight.setPulse(PULSE);
  encoderLeft.setPulse(PULSE);
  
  encoderRight.setRatio(RATIO);
  encoderLeft.setRatio(RATIO);
  
  encoderRight.setPosPid(1.8,0,1.2);
  encoderLeft.setPosPid(1.8,0,1.2);
  
  encoderRight.setSpeedPid(0.18,0,0);
  encoderLeft.setSpeedPid(0.18,0,0);
  
  // DÉBUT : Ne pas modifier ce code!
  // Configuration de la fréquence du PWM
  // Copier-coller ce code si on désire
  // travailler avec les encodeurs
  TCCR1A = _BV(WGM10);
  TCCR1B = _BV(CS11) | _BV(WGM12);

  TCCR2A = _BV(WGM21) | _BV(WGM20);
  TCCR2B = _BV(CS21);
}

// void SetMotor(int direction, float modifier, int motor) {
//   int speed = maxPwm * modifier;
//   int in1 = motor ? m2_in1 : m1_in1;
//   int in2 = motor ? m2_in2 : m1_in2;
//   int pwm = motor ? m2_pwm : m1_pwm;
//   if (direction == 1) {
//     //ccw
//     digitalWrite(in1, LOW);
//     digitalWrite(in2, HIGH);
//   } else if (direction == 0) {
//     //cw
//     digitalWrite(in1, HIGH);
//     digitalWrite(in2, LOW);
//   }
//   analogWrite(pwm, speed);
// }

void Forward(float pwm) {
  encoderLeft.setMotorPwm(pwm);
  encoderRight.setMotorPwm(-pwm);
}

void TurnLeft(float pwm) {
  encoderLeft.setMotorPwm(-pwm);
  encoderRight.setMotorPwm(-pwm);
}

void TurnRight(float pwm) {
  encoderLeft.setMotorPwm(pwm);
  encoderRight.setMotorPwm(pwm);
}

void Stop() {
  encoderLeft.setMotorPwm(0);
  encoderRight.setMotorPwm(0);
}

void StopLed() {
  led.setColor(0, 0);
  led.show();
}

void Return(bool reset) {
  static bool turn = true;
  static int zAngleGoal = 0;
  AnimationTask();
  if (reset) {
    turn = true;
    zAngleGoal = gyro.getAngleZ() - 170;
  }

  if (turn) {
    if (gyro.getAngleZ() > zAngleGoal) {
      TurnLeft(speed);
    } else {
      turn = false;
    }
  } else {
    Stop();
  }
}

void FindLine(bool reset) {
  if (reset) {
    Return(true);
  } else if (lastSensor == 0) {
    TurnLeft(speed);
  } else if (lastSensor == 4) {
    TurnRight(speed);
  } else if (lastSensor == 2) {
    Return(false);
  }
}

void Follow(float pwm) {
  static bool ReturnReset = true;

  if (sensorValues[2] < seuil) {
    Forward(pwm);
    lastSensor = 2;
    ReturnReset = true;
    StopLed();
  } else if (sensorValues[0] < seuil || sensorValues[1] < seuil) {
    TurnLeft(pwm);
    lastSensor = 0;
    ReturnReset = true;
    StopLed();
  } else if (sensorValues[3] < seuil || sensorValues[4] < seuil) {
    TurnRight(pwm);
    lastSensor = 4;
    ReturnReset = true;
    StopLed();
  } else {
    FindLine(ReturnReset);
    ReturnReset = false;
  }
}


void SerialTask() {
  static unsigned long lastTime = 0;
  if (currentTime - lastTime < 1000) return;
  lastTime = currentTime;
  for (int i = 0; i < NB_IR; i++) {
    sensorValues[i] = ss.analogRead(i);
    Serial.print("IR");
    Serial.print(i);
    Serial.print(":");
    Serial.print(sensorValues[i]);
    Serial.print("\t");
  }
  Serial.println();
  Serial.print("valmin");
  Serial.print(valMin);
  Serial.print("valMax");
  Serial.print(valMax);
  Serial.print("seuil");
  Serial.print(seuil);
}

void AnimationTask() {
  static unsigned long lastTime = currentTime;
  static short idx = 1;
  static short idx2 = 6;
  if(currentTime-lastTime <= 100) return;
  lastTime=currentTime;
  idx = idx > LEDNUM ? 1 : idx + 1;
  idx2 = idx2 <= 1 ? LEDNUM : idx2 - 1;
  led.setColor(0,0);
  led.setColor(idx, 0x000A00);
  led.setColor(idx2,0x08000A);
  led.show();
}

void ReadValues() {
  for (int i = 0; i < NB_IR; i++) {
    sensorValues[i] = ss.analogRead(i);
  }
}

void Calibrate() {
  for (int i = 0; i < NB_IR; i++) {
    int val = ss.analogRead(i);
    if (val > valMax) {valMax = val;}
    if (val < valMin) {valMin = val;}
  }
  seuil = (valMin + valMax) / 2 + 100;
}

// float lectureNormalisee(int index) {
//   float num = (sensorValues[index] - valMin) * 1.0;
//   float denum = (valMax-valMin) * 1000.0;
//   return num / denum;
// }

// void normaliser() {
//   for (int i = 0; i < NB_IR; i++) {
//     sensorNormalized[i] = lectureNormalisee(i);
//   }
// }

// float computePosition() {
//   unsigned long numerateur = 0;
//   unsigned long denominateur = 0;
//   int poids = 0;
//   for (int i = 0; i < NB_IR; i++) {
//     poids = 1000*i;
//     numerateur += sensorNormalized[i] * poids;
//     denominateur += sensorNormalized[i];
//   }
//   return (numerateur / denominateur);
// }

// float computePID(float position) {
//   // Ajuster les coefficients selon vos besoins
//   static float kp = 0.1;   // Coefficient proportionnel
//   static float ki = 0.01;  // Coefficient intégral
//   static float kd = 0.01;  // Coefficient dérivé

//   static float integral = 0;
//   static float derivative = 0;
//   static float lastError = 0;

//   float error = position - 2000;  // 2000 est la position du milieu

//   integral += error;
//   derivative = error - lastError;
//   lastError = error;

//   float output = kp * error + ki * integral + kd * derivative;

//   return output;
// }