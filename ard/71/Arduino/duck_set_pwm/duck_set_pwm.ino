#include "TimerOne.h"

// Left
#define AIN1 4
#define AIN2 3
#define APWM 2

// Right
#define BIN1 7
#define BIN2 6
#define BPWM 8

#define STBY 5

// Encoder pins
#define ENCA1 21 // Right
#define ENCA2 20
#define ENCB1 19 // Left
#define ENCB2 18

#define PULSES_PER_REV 1857

#define bluetooth Serial2
bool newData = false;
int motor1Speed = 0;  // target RPM
int motor2Speed = 0;

volatile long encoderCountA = 0;
volatile long encoderCountB = 0;

int pwmLeft = 0;
int pwmRight = 0;

int targetLeftRPM = 0;
int targetRightRPM = 0;

// PID parameters (start simple)
float Kp = 2.0;  // proportional gain
// You can later add Ki, Kd if needed

void setup() {
  Serial.begin(115200);
  bluetooth.begin(9600);
  Timer1.initialize(50000);  // 1 sec
  int motorPins[] = {AIN1, AIN2, APWM, BIN1, BIN2, BPWM, STBY};
  for (int i = 0; i < 7; i++) pinMode(motorPins[i], OUTPUT);
  digitalWrite(STBY, HIGH);

  pinMode(ENCA1, INPUT_PULLUP);
  pinMode(ENCA2, INPUT_PULLUP);
  pinMode(ENCB1, INPUT_PULLUP);
  pinMode(ENCB2, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENCA1), encoderA_ISR, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCB1), encoderB_ISR, RISING);

  Timer1.attachInterrupt(ISR_timerone);
}

void ISR_timerone() {
    // Save and reset counts
    noInterrupts();
    long countA = encoderCountA;
    long countB = encoderCountB;
    encoderCountA = 0;
    encoderCountB = 0;
    interrupts();

    // Compute actual RPM
    float revA = (float)countA / PULSES_PER_REV;
    float revB = (float)countB / PULSES_PER_REV;
    float rpmA = revA * 60.0 / 0.05; // since ISR runs every 50ms
    float rpmB = revB * 60.0 / 0.05;

    // Compute error vs desired speed
    float errLeft  = targetLeftRPM  - rpmA;
    float errRight = targetRightRPM - rpmB;

    // Simple P-controller
    pwmLeft  = constrain(pwmLeft  + Kp * errLeft, 0, 255);
    pwmRight = constrain(pwmRight + Kp * errRight, 0, 255);

    // Apply motor PWM
    moveMotor(AIN1, AIN2, APWM, targetLeftRPM >= 0, pwmLeft);
    moveMotor(BIN1, BIN2, BPWM, targetRightRPM >= 0, pwmRight);

    // Optional: print RPM
    Serial.print("Left RPM: "); Serial.print(rpmA);
    Serial.print(" | Right RPM: "); Serial.println(rpmB);
}



void loop() {
  // Serial reset encoder command
  if (Serial.available() > 0) {
    char cmd = Serial.read();
    if (cmd == 'r' || cmd == 'R') {
      noInterrupts();
      encoderCountA = 0;
      encoderCountB = 0;
      interrupts();
      Serial.println("Encoders reset!");
    }
  }
  receiveData();
}

void moveMotor(int in1, int in2, int pwm, bool forward, int speed) {
  digitalWrite(in1, forward ? HIGH : LOW);
  digitalWrite(in2, forward ? LOW : HIGH);
  analogWrite(pwm, speed);
}

// Encoder ISRs
void encoderA_ISR() { encoderCountA++; }
void encoderB_ISR() { encoderCountB++; }

// Receive target RPM via bluetooth
void receiveData() {
  static byte ndx = 0;
  static char receivedChars[16];
  char rc;

  while (bluetooth.available() > 0 && !newData) {
    rc = bluetooth.read();
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
    motor1Speed = atoi(strtokIndx);
    strtokIndx = strtok(NULL, ",");
    motor2Speed = atoi(strtokIndx);
    newData = false;
    targetLeftRPM = motor1Speed;
    targetRightRPM = motor2Speed;
    
  }
}
