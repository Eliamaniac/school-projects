//Numéro : 30
// Moitié arrière
// Anti-horaire 
// Croisement


#include <MeAuriga.h>

enum AppState {stop, straight, turning, back, turnAgain,end};

AppState currentState = straight;

#define DIST_WHEEL 151
#define DIA_WHEEL 64.5
#define PULSE 9
#define RATIO 39.267
#define FULL_TURN_CIRC 948.8
#define FULL_SPIN_CIRC 474.4
#define CIRC_WHEEL 202.6
#define LEDNUM  12
#define LEDPIN  44

MeGyro gyro(0, 0x69);

MeEncoderOnBoard encoderRight(SLOT1);
MeEncoderOnBoard encoderLeft(SLOT2);

MeRGBLed led( PORT0, LEDNUM );
MeUltrasonicSensor ultraSensor(PORT_8);

unsigned long currentTime = 0;

unsigned long serialPrintPrevious = 0;
int serialPrintInterval = 500;
float distance = 100;
int waitTime = 3000;
long lastDistance = 0;
short speed = 100;
double zAngleGoal = 0.0;

// ********* INTERRUPTIONS ***********

void rightEncoderInterrupt(void)
{
  if(digitalRead(encoderRight.getPortB()) == 0)
  {
    encoderRight.pulsePosMinus();
  }
  else
  {
    encoderRight.pulsePosPlus();;
  }
}

void leftEncoderInterrupt(void) {
  if(digitalRead(encoderLeft.getPortB()) == 0)
  {
    encoderLeft.pulsePosMinus();
  }
  else
  {
    encoderLeft.pulsePosPlus();
  }
}

// ************* DÉBUT ************

void encoderConfig() {
  attachInterrupt(encoderRight.getIntNum(), rightEncoderInterrupt, RISING);
  attachInterrupt(encoderLeft.getIntNum(), leftEncoderInterrupt, RISING);
  
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
  // FIN : Ne pas modifier ce code!
}


void setup() {
  Serial.begin(115200);
  encoderConfig();
  gyro.begin();
  led.setpin(LEDPIN);
  // Waiting 3 sec before start
  Serial.println("Waiting 1 sec");
  delay (1000);
}

void loop() {
  currentTime = millis();
  
  stateManager();
  gyroTask(currentTime);
  encodersTask(currentTime);
  distance = distanceTask(currentTime);
  serialTask();
}

void stateManager() {
  switch (currentState) {
    case stop:
      stopState();
      break;
    case straight:
      straightState();
      break;
    case turning:
      turnState();
      break;
    case back:
      backState();
      break;
    case turnAgain:
      turnAgainState();
      break;
    case end:
      endState();
      break;
    default:
      stopState();
      break;
  }
}

void serialTask() {
  static unsigned long lastTime = 0;
  if(currentTime-lastTime <= 500) return;
  lastTime = currentTime;
  Serial.print("État : ");
  Serial.println(currentState);
  Serial.print("distance : ");
  Serial.println(distance);
  Serial.print("batterie : ");
  Serial.println(analogRead(A4));
}

void gyroTask(unsigned long ct) {
  gyro.update();
}

void encodersTask(unsigned long ct) {
  encoderRight.loop();
  encoderLeft.loop();
}

float distanceTask(unsigned long ct) {
  //if(ct-lt>)
  static unsigned long lastTime = 0;
  if(currentTime-lastTime<=250) return;
  return ultraSensor.distanceCm();
}

void ledTask() {
  static unsigned long lastTime = 0;
  static bool allume = true;
  short idx = 6;
  if(currentTime-lastTime <= 250) return;
  lastTime = currentTime;
  if(allume) {
    for(int i = 1;i<=6;i++) {
      led.setColor(idx, 0x0A0500);
      idx = idx >= LEDNUM ? 1 : idx + 1;
    }
    allume = false;
  } else {
    led.setColor(0, 0);
    allume = true;
  }
  led.show();
}

void animationTask() {
  static unsigned long lastTime = currentTime;
  static short idx = 1;
  static short idx2 = 6;
  if(currentTime-lastTime <= 100) return;
  lastTime=currentTime;
  idx = idx > LEDNUM ? 1 : idx + 1;
  idx2 = idx2 <= 1 ? LEDNUM : idx2 - 1;
  led.setColor(0,0);
  led.setColor(idx, 0x000A00);
  led.setColor(idx2,0x0A0000);
  led.show();
}

void gostraight(short speed = 100, short firstRun = 0) {
    static double error = 0.0;
    static double previousError = 0.0;
    static double integral = 0.0;
    static double output = 0;
    
    // PID Controller Constants
    const double kp = 3.0;
    const double ki = 0.05;  // Integral term, tune this
    const double kd = 1.0;    
    
    if (firstRun) {
      firstRun = 0;

      zAngleGoal = gyro.getAngleZ();
      Serial.println ("Setting speed");
      
      encoderLeft.setTarPWM(speed);
      encoderRight.setTarPWM(-speed);
      
      return;
    }
    
    // Calculate error
    error = gyro.getAngleZ() - zAngleGoal;
    
    // Accumulate integral
    integral += error;
    
    // Optional: Limit the integral to avoid "integral windup"
    if (integral > 1000) integral = 1000;
    if (integral < -1000) integral = -1000;
    
    // PID Output
    output = kp * error + ki * integral + kd * (error - previousError);
    
    // Update previous error for the next loop
    previousError = error;
    
    // Adjust motor speed based on PID output
    encoderLeft.setTarPWM(speed - output);
    encoderRight.setTarPWM(-speed - output);
}


void goBack(short speed = 100, short firstRun = 0) {
    static double error = 0.0;
    static double previousError = 0.0;
    static double output = 0;
    
    // PD Controller
    // Change these values to suit your needs
    // higher kp = more reactive, might have oscillation
    // lowewr kp = sluggish, but less oscillation
    // higher kd = limit oscillation, the right value stops oscillation
    const double kp = 3.0;
    const double kd = 1.0;    
    
    if (firstRun) {
      firstRun = 0;

      zAngleGoal = gyro.getAngleZ();
      Serial.println ("Setting speed");
      
      encoderLeft.setTarPWM(-speed);
      encoderRight.setTarPWM(speed);
      
      return;
    }
    
    error = gyro.getAngleZ() - zAngleGoal;
    
    output = kp * error + kd * (error - previousError);
    
    previousError = error;        
    
    encoderLeft.setTarPWM(-speed - output);
    encoderRight.setTarPWM(speed - output);
    if(encoderLeft.getPulsePos()<0) {
      currentState = end;
    }
}

void stopState() { 
  static unsigned long lastTime = currentTime;
  static bool firstTime = true;
  if (firstTime) {
    firstTime = false;
    encoderLeft.setTarPWM(0);
    encoderRight.setTarPWM(0);
    led.setColor(0,0);
    led.show();
    lastDistance = encoderLeft.getPulsePos();
  }
  if(currentTime - lastTime < waitTime) return;
  currentState = turning;
}

void straightState() {  
  static bool firstTime = true;
  // if (firstTime) {
  //   firstTime = false;
  // }
  ledTask();
  gostraight();
  if (distance <= 30) {
    currentState = stop;
  }
}

void turnState() {
  //antihoraire
  encoderLeft.setTarPWM(-speed);
  encoderRight.setTarPWM(-speed);
  int currentAngle = gyro.getAngleZ();
  if(currentAngle < -80) {
    Serial.println("Sortie d'état 'turn'");
    encoderLeft.setTarPWM(0);
    encoderRight.setTarPWM(0);
    currentState = turnAgain;
  }

void endState() {
  animationTask();
  encoderLeft.setTarPWM(0);
  encoderRight.setTarPWM(0); 
}