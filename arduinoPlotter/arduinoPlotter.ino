#include <Stepper.h>

//#define ENABLE_LOG
#ifdef ENABLE_LOG
#define LOG(X) Serial.print(X)
#else
#define LOG(X)
#endif

struct DCMotor {
  DCMotor(int enablePin, int motorPinA, int motorPinB) {
    MotorPinA = motorPinA;
    MotorPinB = motorPinB;
    EnablePin = enablePin;
    
    pinMode(motorPinA, OUTPUT);
    pinMode(motorPinB, OUTPUT);
    pinMode(enablePin, OUTPUT);

    SetSpeed(0);

  }

  void SetSpeed(int speed) {
    // set direction
    if(speed > 0) {
      digitalWrite(MotorPinA, HIGH);
      digitalWrite(MotorPinB, LOW);
    } else {
      digitalWrite(MotorPinA, LOW);
      digitalWrite(MotorPinB, HIGH);      
    }

    analogWrite(EnablePin, abs(speed));
  }

  int MotorPinA;
  int MotorPinB;
  int EnablePin;
};

DCMotor penMotor(5, 6, 7);
Stepper stepper(24, 10, 11, 12, 13);
#define Motor0StepPin 2
#define Motor0DirPin 3
#define SensorXPin 8

// scale to use the HPGL plotter scale (0.025mm per step)
#define xScale 3.4
#define yScale 5.1

void stepDCMotor(int step) {
  if(step < 0)
    digitalWrite(Motor0DirPin, LOW);
  else
    digitalWrite(Motor0DirPin, HIGH);
  
  for(int i = 0; i < abs(step); i++) {
    digitalWrite(Motor0StepPin, LOW);
    delayMicroseconds(500);
    digitalWrite(Motor0StepPin, HIGH);
    delayMicroseconds(500);
  }
}

/*
 * Machine state
 */
bool penUp = false;
int y = 0;
int x = 0;

/*
 * Move Pen up or Down
 */
void movePenUp(bool up, bool force = false) {
  // don't move pen if not needed
  if(penUp == up && !force)
    return;
  
  penMotor.SetSpeed(up ? 180 : -100);

  delay(up ? 200 : 100);
  penMotor.SetSpeed(0);

  penUp = up;

  // a little delay for pen rebound
  delay(100);
}

/*
 * Reset the X axis
 */
void resetAxis() {
  while(digitalRead(SensorXPin) == LOW) {
    stepDCMotor(-1);
  }

  stepDCMotor(900); // to put the pen on top of the sheet

  // reset x value
  x = 0;
}

void setup() {
  pinMode(Motor0StepPin, OUTPUT);
  pinMode(Motor0DirPin, OUTPUT);
  pinMode(SensorXPin, INPUT);
  
  stepper.setSpeed(100);
  
  //Initialize serial and wait for port to open:
  Serial.begin(115200);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for Leonardo only
  }

  // force pen up
  movePenUp(true, true);
  resetAxis();

  Serial.println("READY;");
}

int fxy;
int dx;
int dy;
int endx = 0;
int endy = 0;
int radius = 0;
int dir = 0;

void circleOperation() {
  fxy = x * x + y * y - radius * radius;
  int dx = 2 * x;
  int dy = 2 * y;

  int state = (dy<0?0:1) + ((dx<0?0:1) << 1) +  ((fxy<0?0:1) << 2) + (dir << 3);
  //Serial.println(state);
  int incX = 0;
  int incY = 0;

  switch(state) {
    case 0x0: incY = -1; break;
    case 0x1: incX = -1; break;
    case 0x2: incX = 1;  break;
    case 0x3: incY = 1;  break;
    case 0x4: incX = 1;  break;
    case 0x5: incY = -1; break;
    case 0x6: incY = 1;  break;
    case 0x7: incX = -1; break;
    case 0x8: incX = -1; break;
    case 0x9: incY = 1;  break;
    case 0xA: incY = -1; break;
    case 0xB: incX = 1;  break;
    case 0xC: incY = 1;  break;
    case 0xD: incX = 1;  break;
    case 0xE: incX = -1; break;
    case 0xF: incY = -1; break;    
  }

  if(incX != 0) {
    stepDCMotor(incX);
    x += incX;
  }

  if(incY != 0) {
    stepper.step(incY);
    y += incY;
  }
}

void moveTo(int targetX, int targetY) {
  int dx = targetX - x;
  int dy = targetY - y;
  int fxy = abs(dx) - abs(dy);

  while(targetX != x || targetY != y) {
    if(fxy < 0) {
      // step on y
      if(dy > 0) {
        stepper.step(1);
        y++;
      } else if(dy < 0) {
        stepper.step(-1);
        y--;
      }
  
      fxy += abs(dx);
    }
  
    else {
      // step on x
      if(dx > 0) {
        stepDCMotor(1);
        x++;
      } else if(dx < 0) {
        stepDCMotor(-1);
        x--;
      }
  
      fxy -= abs(dy);
    }

    // this delay will slow down the motion 
    delayMicroseconds(1000);
  }
}

/*************************************
 * Serial command parser
 *************************************/
char serialBuffer[64];
char *serialBufferPtr = serialBuffer;
int parseInt(char *&ptr) {
  int v = 0;
  while(isdigit(*ptr) && *ptr != 0) {
    v = v * 10 + (*ptr - '0');
    ptr++;
  }

  return v;
}

void parseCommand() {
  bool valid = true;
  if(strncmp(serialBuffer, "IN", 2) == 0) {
    LOG("Starting job\n");

    movePenUp(true);
    resetAxis();
    stepper.step(-y);
  }

  else if(strncmp(serialBuffer, "IP", 2) == 0) {
    // reset origin
    x = 0;
    y = 0;

    LOG("Set initial point");
  }
  
  else if(strncmp(serialBuffer, "PD", 2) == 0) {
    char *p = &serialBuffer[2];
    int x = parseInt(p) / xScale; p++;
    int y = parseInt(p) / yScale;
    
    LOG("Pen Down(");
    LOG(x);
    LOG(", ");
    LOG(y);
    LOG(")\n");
    
    movePenUp(false);
    moveTo(x, y);  
  }
  else if(strncmp(serialBuffer, "PU", 2) == 0) {
    char *p = &serialBuffer[2];
    int x = parseInt(p) / xScale; p++;
    int y = parseInt(p) / yScale;

    LOG("Pen Up(");
    LOG(x);
    LOG(", ");
    LOG(y);
    LOG(")\n");

    movePenUp(true);
    moveTo(x, y);    
  }
  else if(strncmp(serialBuffer, "SP", 2) == 0) {
    int pen = Serial.parseInt();
    
    LOG("Selecting Pen ");
    LOG(pen);
    LOG("\n");
  }

  else {
    LOG("Unknown Command ");
    LOG(serialBuffer[0]);LOG(serialBuffer[1]);
    LOG("\n");

    valid = false;
    Serial.println("Unknown Command ");
    Serial.print(serialBuffer[0]);
    Serial.print(serialBuffer[1]);
    Serial.print("\n");
  }

  if(valid)
    Serial.println("DONE;");
}

void processSerialData() {
  while(Serial.available() > 0) {
    char c = Serial.read();

    if(c != '\n') {
      *serialBufferPtr = c;
      serialBufferPtr++;
      
      if(c == ';') {
        // terminate string
        *serialBufferPtr = 0;
        //LOG(serialBuffer); LOG(" -> ");
        parseCommand();
  
        // reset pointer
        serialBufferPtr = serialBuffer;
      }
    }
  }
}

void loop() {
  processSerialData();
}
