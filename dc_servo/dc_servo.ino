#include <PID_v1.h>

//#define LOG

#define motorEnablePWM 6
#define motorPinA 8
#define motorPinB 9

#define servoEncoderPin0 0 // has to be an interrupt!
#define servoEncoderPin1 1

#define directionPin 10
#define stepPin 7


volatile int target; // target position
volatile int position = 0; // current position

void updateCounter() {
  int dir = digitalRead(directionPin);

  if(dir == LOW)
    target--;
  else
    target++;
  //Serial.println(target);
}

unsigned char oldEnc = 0;
void servo0ISR() {
  unsigned char v = PIND;
  if(!(oldEnc & B00000100) && (v & B00000100)) {
    if(v & B00001000) 
      position++;
    else
      position--;
  } else {
    if(v & B00001000) 
      position--;
    else
      position++;
  }

  oldEnc = v;
  
  //Serial.println(position);
}

void setMotorSpeed(int speed) {
  // set direction
  if(speed > 0) {
    PORTB = (PORTB & B11011111) | B00010000;
  } else {
    PORTB = (PORTB & B11101111) | B00100000;
  }

  analogWrite(motorEnablePWM, abs(speed));
}

float Kp = 4;
float Ki = 0.0;
float Kd = 0.01;

float I = 0;
float oldE = 0;

int minPWM = 150;
int maxPWN = 200;

void setup() {
  pinMode(directionPin, INPUT);
  pinMode(stepPin, INPUT);
  
  pinMode(motorPinA, OUTPUT);
  pinMode(motorPinB, OUTPUT);
  pinMode(motorEnablePWM, OUTPUT);

  pinMode(servoEncoderPin0, INPUT);
  pinMode(servoEncoderPin1, INPUT);

  attachInterrupt(4, updateCounter, RISING);
  attachInterrupt(2, servo0ISR, CHANGE);

  TCCR1B = TCCR1B & 0b11111000 | 1; // set 31Kh PWM

#ifdef LOG
  Serial.begin(115200); 
  while (!Serial) {
    ; // wait for serial port to connect. Needed for Leonardo only
  }

  Serial.println("DC Servo");
  target = 500;
  oldE = target - position;
#endif
}

unsigned long lastUpdateTime = 0;
int updateTime = 1; // 1ms

float lastPWM = 0;

void updatePID() {
  unsigned long now = millis();

  if(now - lastUpdateTime > updateTime) {
    float e = target - position;
    float pwm = e * Kp;
  
    float d = (e - oldE) / 1e-3;
    pwm += d * Kd;
    oldE = e;
  
    if(fabs(e) < 1) {
      I = 0;
      pwm = 0; // stop the motor
    } else {
      I = I + (e > 0 ? 1 : -1);
      pwm += I * Ki;
  
      // we have to move!
      if(fabs(pwm) < minPWM) {
        pwm = pwm > 0 ? minPWM : -minPWM;  
      }

      float a = 0.8f;
      pwm = a * pwm + (1.0f - a) * lastPWM;
      lastPWM = pwm;
    }
  
    pwm = min(max(pwm, -maxPWN), maxPWN);
    setMotorSpeed(pwm);

    lastUpdateTime = now;
  }
}

void loop() {
  updatePID();
  
#ifdef LOG
  if(millis() % 1000 == 0) {
    Serial.print(position);
    Serial.println();
  }
#endif
}
