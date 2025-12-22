#include "TimerOne.h"
// Left motor
#define AIN1 4
#define AIN2 3
#define APWM 2

// Right motor
#define BIN1 7
#define BIN2 6
#define BPWM 8.

#define STBY 5

float factor = 0.968;
int counter = 0;
// Encoder pins
#define ENCA1 21 // Right  // Motor A encoder channel A
#define ENCA2 20 // Right  // Motor A encoder channel B
#define ENCB1 19 // Left   // Motor B encoder channel A
#define ENCB2 18 // Left   // Motor B encoder channel B

#define bluetooth Serial2

#define PULSES_PER_REV 1857

bool newData = false;

int motor1Speed = 0;  // left motor target PWM
int motor2Speed = 0;  // right motor target PWM
volatile long encoderCountA = 0;
volatile long encoderCountB = 0;

void setup() {
  Serial.begin(115200);     // USB Serial
  bluetooth.begin(9600);    // Bluetooth Serial
  Timer1.initialize(50000); // 50 ms ISR
  Timer1.attachInterrupt(ISR_timerone);

  int motorPins[] = {AIN1, AIN2, APWM, BIN1, BIN2, BPWM, STBY};
  for (int i = 0; i < 7; i++) pinMode(motorPins[i], OUTPUT);
  digitalWrite(STBY, HIGH);
  pinMode(ENCA1, INPUT_PULLUP);
  pinMode(ENCA2, INPUT_PULLUP);
  pinMode(ENCB1, INPUT_PULLUP);
  pinMode(ENCB2, INPUT_PULLUP);

  // Attach interrupts
  attachInterrupt(digitalPinToInterrupt(ENCA1), encoderA_ISR, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCB1), encoderB_ISR, RISING);

}

void loop() {
  
  receiveData(Serial);
  receiveData(bluetooth);
}

void moveMotor(int in1, int in2, int pwm, bool forward, int speed) {
  digitalWrite(in1, forward ? HIGH : LOW);
  digitalWrite(in2, forward ? LOW : HIGH);
  analogWrite(pwm, speed);
}

// Interrupts (count pulses)
void encoderA_ISR() {
  encoderCountA++;
}
void encoderB_ISR() {
  encoderCountB++;
}

void ISR_timerone() {
 counter++;
 if (counter % 30 == 0)
 {
  factor = 1;
 }
 else
 {
  factor = 0.968;
 }

    noInterrupts();
    long countA = encoderCountA;
    long countB = encoderCountB;
    encoderCountA = 0;
    encoderCountB = 0;
    interrupts();


    float revA = (float)countA / PULSES_PER_REV;
    float revB = (float)countB / PULSES_PER_REV;

    float rpmA = revA * 20.0;
    float rpmB = revB * 20.0;
    // 50 ms → 20 Hz
    float revsPerSecA = revA * 20.0;
    float revsPerSecB = revB * 20.0;

    // wheel radius (example: 0.06721 m from your setup)
    float wheelRadius = 0.033605;

    // linear velocity in m/s
    float velA = revsPerSecA * 2.0 * PI * wheelRadius;
    float velB = revsPerSecB * 2.0 * PI * wheelRadius;

    if (motor1Speed < 0)
    {
      velA = velA * -1;
    }

    if ( motor2Speed<0 )
    {
      velB = velB * -1;
    }

    
    Serial.print(velB, 4);
    Serial.print(",");
    Serial.println(velA, 4);  // m/s
    // Serial.print(rpmA);
    // Serial.print(",");
    // Serial.println(rpmB);

}
void setSpeed(int leftSpeed, int rightSpeed) {
  // Left motor (B)
  if (leftSpeed > 0)
  {
    moveMotor(BIN1, BIN2, BPWM, false, leftSpeed);
  }
  else
  {
    moveMotor(BIN1, BIN2, BPWM, true, abs(leftSpeed));
  }

  // Right motor (A)
  if (rightSpeed > 0)
  {
    moveMotor(AIN1, AIN2, APWM, false, (rightSpeed * factor));
  }
  else
  {
    moveMotor(AIN1, AIN2, APWM, true, abs(rightSpeed));
  }
}

void receiveData(Stream &stream) {
  static char receivedChars[16];
  static byte ndx = 0;
  char rc;

  while (stream.available() > 0 && newData == false) {
    rc = stream.read();

    if (rc != '\n' && rc != '\r') {
      receivedChars[ndx++] = rc;
      if (ndx >= sizeof(receivedChars) - 1) ndx = sizeof(receivedChars) - 1;
    } else {
      receivedChars[ndx] = '\0';
      ndx = 0;
      newData = true;
    }
  }

  if (newData) {
    char *strtokIndx = strtok(receivedChars, ",");
    if (strtokIndx != NULL) {
      motor1Speed = atoi(strtokIndx);
      strtokIndx = strtok(NULL, ",");
      if (strtokIndx != NULL) {
        motor2Speed = atoi(strtokIndx);
        setSpeed(motor1Speed, motor2Speed);
      }
    }
    newData = false;
  }
}
