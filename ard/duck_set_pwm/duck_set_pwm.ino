#include "TimerOne.h"

// Left motor pins
#define AIN1 4
#define AIN2 3
#define APWM 2

// Right motor pins
#define BIN1 7
#define BIN2 6
#define BPWM 8

#define STBY 5

// Encoder pins
#define ENCA1 21 // Right motor encoder A
#define ENCA2 20 // Right motor encoder B
#define ENCB1 19 // Left  motor encoder A
#define ENCB2 18 // Left  motor encoder B

#define PULSES_PER_REV 1857

#define bluetooth Serial2
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

  attachInterrupt(digitalPinToInterrupt(ENCA1), encoderA_ISR, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCB1), encoderB_ISR, RISING);
}

// ===== Timer ISR (every 50 ms) =====
void ISR_timerone() {
  noInterrupts();
  long countA = encoderCountA;
  long countB = encoderCountB;
  encoderCountA = 0;
  encoderCountB = 0;
  interrupts();

  float revA = (float)countA / PULSES_PER_REV;
  float revB = (float)countB / PULSES_PER_REV;

  float rpmA = revA * 60.0 / 0.05; // per 50ms
  float rpmB = revB * 60.0 / 0.05;

  Serial.print("Left RPM: ");
  Serial.print(rpmA);
  Serial.print(" | Right RPM: ");
  Serial.println(rpmB);
}

// ===== Main loop =====
void loop() {
  // Accept commands from BOTH Serial and Bluetooth
  receiveData(Serial);
  receiveData(bluetooth);
}

// ===== Motor control =====
void moveMotor(int in1, int in2, int pwm, bool forward, int speed) {
  digitalWrite(in1, forward ? HIGH : LOW);
  digitalWrite(in2, forward ? LOW : HIGH);
  analogWrite(pwm, constrain(speed, 0, 255));
}

void setSpeed(int leftSpeed, int rightSpeed) {
  moveMotor(AIN1, AIN2, APWM, leftSpeed >= 0, abs(leftSpeed));
  moveMotor(BIN1, BIN2, BPWM, rightSpeed >= 0, abs(rightSpeed - (rightSpeed * 0.03)));
}

// ===== Encoder ISRs =====
void encoderA_ISR() { encoderCountA++; }
void encoderB_ISR() { encoderCountB++; }

// ===== Command parser (works for Serial + Bluetooth) =====
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
