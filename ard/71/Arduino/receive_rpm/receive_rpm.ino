#include <TimerOne.h>

// ----------------- MOTOR PINS -----------------
#define AIN1 3
#define AIN2 4
#define APWM 2   // Left motor PWM

#define BIN1 6
#define BIN2 7
#define BPWM 8   // Right motor PWM

#define STBY 5   // Standby pin

// ----------------- ENCODER PINS -----------------
#define LEFT_ENC_A  20
#define LEFT_ENC_B  21
#define RIGHT_ENC_A 18
#define RIGHT_ENC_B 19

// ----------------- VARIABLES -----------------
volatile long leftEncoderCount = 0;
volatile long rightEncoderCount = 0;

long prevLeftCount = 0;
long prevRightCount = 0;

float leftRPM = 0;
float rightRPM = 0;

float targetLeftRPM = 0;
float targetRightRPM = 0;

float pwmLeft = 0;
float pwmRight = 0;
///////////////////////
// ---- TUNE THESE ----
const float dt = 0.05f;          // 50 ms control period
const int   pulsesPerRev = 1857; // your 50rev→92883 counts gives ~1857
float Kp = 2.0f;
float Ki = 0.5f;
float Kd = 0.05f;
float Kv = 1.0f;                 // feed-forward gain (start at 0, try 0.8–1.5 later)
const float rpmDeadband = 2.0f;  // don’t correct tiny RPM errors
const float integLimit  = 200.0f; // clamp integral term (RPM*s)
const int   pwmLimit    = 255;
const int   pwmSlewPerStep = 50; // max PWM change per 50 ms step
///////////////////////
// PID params

float iLeft = 0, iRight = 0;
bool newData = false;

//float integralLeft = 0, integralRight = 0;
float prevErrLeft = 0, prevErrRight = 0;
int   pwmLeftCmd = 0, pwmRightCmd = 0;



// ----------------- ENCODER ISR -----------------
void leftEncoderISR() {
  int b = digitalRead(LEFT_ENC_B);
  if (b == HIGH) leftEncoderCount--;
  else leftEncoderCount++;
}

void rightEncoderISR() {
  int b = digitalRead(RIGHT_ENC_B);
  if (b == HIGH) rightEncoderCount++;
  else rightEncoderCount--;
}

// ----------------- MOTOR CONTROL -----------------
void setMotor(int pwm, int in1, int in2, int pwmPin) {
  if (pwm > 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    analogWrite(pwmPin, pwm);
  } else if (pwm < 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    analogWrite(pwmPin, -pwm);
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    analogWrite(pwmPin, 0);
  }
}

// Short-brake helper
inline void brakeMotor(int in1, int in2, int pwmPin) {
  digitalWrite(in1, HIGH);
  digitalWrite(in2, HIGH);   // short-brake
  analogWrite(pwmPin, 0);
}

// Signed motor set (same as you had)
void setMotorSigned(int pwm, int in1, int in2, int pwmPin) {
  pwm = constrain(pwm, -pwmLimit, pwmLimit);
  if (pwm > 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    analogWrite(pwmPin, pwm);
  } else if (pwm < 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    analogWrite(pwmPin, -pwm);
  } else {
    // hold (coast), braking handled elsewhere when target≈0
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    analogWrite(pwmPin, 0);
  }
}

// Slew limiter
inline int slewLimit(int target, int current, int maxStep) {
  if (target > current + maxStep)   return current + maxStep;
  if (target < current - maxStep)   return current - maxStep;
  return target;
}

void controlLoop() {
  // --- Measure RPM from encoder deltas over dt ---
  static long prevLeftCnt = 0, prevRightCnt = 0;

  long leftCnt  = leftEncoderCount;
  long rightCnt = rightEncoderCount;

  long dL = leftCnt  - prevLeftCnt;
  long dR = rightCnt - prevRightCnt;
  prevLeftCnt  = leftCnt;
  prevRightCnt = rightCnt;

  float leftRPM  = (dL / (float)pulsesPerRev) / dt * 60.0f;
  float rightRPM = (dR / (float)pulsesPerRev) / dt * 60.0f;

  // --- LEFT WHEEL ---
  float tgtL = targetLeftRPM;
  if (fabs(tgtL) < rpmDeadband) {
    // Stop behavior
    iLeft = 0; prevErrLeft = 0;
    pwmLeftCmd = 0;
    brakeMotor(AIN1, AIN2, APWM);
  } else {
    float errL = tgtL - leftRPM;
    if (fabs(errL) < rpmDeadband) errL = 0;

    // PID (direct form)
    iLeft += errL * dt;
    iLeft = constrain(iLeft, -integLimit, integLimit);
    float dErrL = (errL - prevErrLeft) / dt;
    float uL = Kp*errL + Ki*iLeft + Kd*dErrL + Kv*tgtL; // Kv*tgtL is optional feed-forward
    prevErrLeft = errL;

    // Clamp & slew
    int pwmTarget = (int)constrain(uL, -pwmLimit, pwmLimit);
    pwmLeftCmd = slewLimit(pwmTarget, pwmLeftCmd, pwmSlewPerStep);

    setMotorSigned(pwmLeftCmd, AIN1, AIN2, APWM);
  }

  // --- RIGHT WHEEL ---
  float tgtR = targetRightRPM;
  if (fabs(tgtR) < rpmDeadband) {
    iRight = 0; prevErrRight = 0;
    pwmRightCmd = 0;
    brakeMotor(BIN1, BIN2, BPWM);
  } else {
    float errR = tgtR - rightRPM;
    if (fabs(errR) < rpmDeadband) errR = 0;

    iRight += errR * dt;
    iRight = constrain(iRight, -integLimit, integLimit);
    float dErrR = (errR - prevErrRight) / dt;
    float uR = Kp*errR + Ki*iRight + Kd*dErrR + Kv*tgtR;
    prevErrRight = errR;

    int pwmTarget = (int)constrain(uR, -pwmLimit, pwmLimit);
    pwmRightCmd = slewLimit(pwmTarget, pwmRightCmd, pwmSlewPerStep);

    setMotorSigned(pwmRightCmd, BIN1, BIN2, BPWM);
  }

  // --- Debug (optional) ---
  Serial.print("Target L/R: ");
  Serial.print(targetLeftRPM); Serial.print(" / "); Serial.print(targetRightRPM);
  Serial.print(" | Actual L/R: ");
  Serial.print(leftRPM); Serial.print(" / "); Serial.println(rightRPM);
}
// ----------------- SETUP -----------------
void setup() {
  Serial.begin(57600);

  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(APWM, OUTPUT);

  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(BPWM, OUTPUT);

  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);

  pinMode(LEFT_ENC_A, INPUT_PULLUP);
  pinMode(LEFT_ENC_B, INPUT_PULLUP);
  pinMode(RIGHT_ENC_A, INPUT_PULLUP);
  pinMode(RIGHT_ENC_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(LEFT_ENC_A), leftEncoderISR, RISING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_A), rightEncoderISR, RISING);

  Timer1.initialize(50000);  // 50 ms
  Timer1.attachInterrupt(controlLoop);
}

// ----------------- LOOP -----------------
void loop() {
  receiveData(Serial);
  //receiveData(bluetooth);

}



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
      targetLeftRPM = atoi(strtokIndx);
      strtokIndx = strtok(NULL, ",");
      if (strtokIndx != NULL) {
        targetRightRPM = atoi(strtokIndx);
         
         
        //setSpeed(rpm_target1, rpm_target2);
      }
    }
    newData = false;
  }
}
