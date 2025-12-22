#include "TimerOne.h"

// --- Motor pins ---
// Left motor (B)
#define BIN1 7
#define BIN2 6
#define BPWM 8   // <-- fixed typo (no trailing dot)

// Right motor (A)
#define AIN1 4
#define AIN2 3
#define APWM 2

#define STBY 5

// Encoders
#define ENCA1 21 // Right motor encoder A (A-side)
#define ENCA2 20 // Right motor encoder B (unused here)
#define ENCB1 19 // Left  motor encoder A (B-side)
#define ENCB2 18 // Left  motor encoder B (unused here)

// Serial
#define BLUETOOTH Serial2

// --- Robot params ---
static const float WHEEL_RADIUS = 0.033605f; // meters
static const long  PULSES_PER_REV = 1857;    // per wheel

// Optional right-motor feedforward trim (keep your behavior)
float factor = 0.968f;
int   counter = 0;

// Commanded PWMs (from ROS)
volatile int motorLeftPWM  = 0;  // left motor target PWM  (-255..255)
volatile int motorRightPWM = 0;  // right motor target PWM (-255..255)

// Encoder pulse counts (accumulated between ISR periods)
volatile long encoderCountRight = 0; // A-side
volatile long encoderCountLeft  = 0; // B-side

// Timing for 100 Hz loop
volatile unsigned long last_us = 0; // microseconds at last ISR

// --- Forward decls ---
void setSpeed(int leftPWM, int rightPWM);
void receiveData(Stream &stream);

// --- ISRs for encoders (simple rising-edge counting) ---
void encoderA_ISR() { encoderCountRight++; }
void encoderB_ISR() { encoderCountLeft++;  }

// --- Setup ---
void setup() {
  Serial.begin(115200);      // USB to ROS
  BLUETOOTH.begin(9600);     // optional

  // Motor pins
  pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT); pinMode(APWM, OUTPUT);
  pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT); pinMode(BPWM, OUTPUT);
  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);

  // Encoders
  pinMode(ENCA1, INPUT_PULLUP);
  pinMode(ENCA2, INPUT_PULLUP);
  pinMode(ENCB1, INPUT_PULLUP);
  pinMode(ENCB2, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENCA1), encoderA_ISR, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCB1), encoderB_ISR, RISING);

  // Timer1 at 100 Hz (10,000 us)
  Timer1.initialize(10000);
  last_us = micros();
  Timer1.attachInterrupt(ISR_timerone);
}

void loop() {
  // Accept commands from USB or Bluetooth
  receiveData(Serial);
  receiveData(BLUETOOTH);
}

// --- Low-level motor drive ---
void moveMotor(int in1, int in2, int pwmPin, bool forward, int pwmVal) {
  digitalWrite(in1, forward ? HIGH : LOW);
  digitalWrite(in2, forward ? LOW  : HIGH);
  pwmVal = constrain(pwmVal, 0, 255);
  analogWrite(pwmPin, pwmVal);
}

// Apply signed PWM to both motors (left first, right second)
void setSpeed(int leftPWM, int rightPWM) {
  leftPWM  = constrain(leftPWM,  -255, 255);
  rightPWM = constrain(rightPWM, -255, 255);

  // Left motor (B)
  if (leftPWM >= 0) {
    moveMotor(BIN1, BIN2, BPWM, /*forward=*/false, leftPWM);
  } else {
    moveMotor(BIN1, BIN2, BPWM, /*forward=*/true,  -leftPWM);
  }

  // Right motor (A) with trim
  int rightPWMTrim = rightPWM >= 0 ? int(rightPWM * factor) : -int((-rightPWM) * factor);
  if (rightPWMTrim >= 0) {
    moveMotor(AIN1, AIN2, APWM, /*forward=*/false, rightPWMTrim);
  } else {
    moveMotor(AIN1, AIN2, APWM, /*forward=*/true,  -rightPWMTrim);
  }
}

// --- Timer ISR @ 100 Hz: compute speeds and stream to ROS ---
void ISR_timerone() {
  // Keep your periodic trim behavior (optional)
  counter++;
  factor = (counter % 30 == 0) ? 1.0f : 0.968f;

  // Compute dt from last tick (seconds)
  unsigned long now_us = micros();
  float dt = (now_us - last_us) / 1e6f;      // seconds
  // Protect against first run / overflow or weird dt
  if (dt <= 0.0f || dt > 0.2f) {             // >200 ms? skip (USB pause/etc.)
    last_us = now_us;
    return;
  }
  last_us = now_us;

  // Atomically copy & reset encoder counts
  noInterrupts();
  long cR = encoderCountRight;  // Right wheel (A-side)
  long cL = encoderCountLeft;   // Left  wheel (B-side)
  encoderCountRight = 0;
  encoderCountLeft  = 0;
  int  leftPWM  = motorLeftPWM;   // snapshot of commands for sign
  int  rightPWM = motorRightPWM;
  interrupts();

  // Convert pulses → revolutions in the last dt
  float revR = (float)cR / (float)PULSES_PER_REV;
  float revL = (float)cL / (float)PULSES_PER_REV;

  // Revs per second
  float rpsR = revR / dt;
  float rpsL = revL / dt;

  // Linear velocity (m/s) = rps * circumference
  const float CIRC = 2.0f * PI * WHEEL_RADIUS;
  float velR = rpsR * CIRC;
  float velL = rpsL * CIRC;

  // Apply commanded sign so reverse motion is negative
  if (rightPWM < 0) velR = -velR;
  if (leftPWM  < 0) velL = -velL;

  // --- Stream to ROS over USB as "v_l,v_r\n" (LEFT FIRST) ---
  // EXACT format expected by your ROS2 node when IS_FIRMWARE_SPEED_MPS = True
  Serial.print(velL, 4);
  Serial.print(",");
  Serial.println(velR, 4);
}

// --- Receive "left_pwm,right_pwm\n" from ROS ---
void receiveData(Stream &stream) {
  static char buf[32];
  static byte idx = 0;

  while (stream.available() > 0) {
    char ch = stream.read();

    if (ch == '\n' || ch == '\r') {
      if (idx == 0) continue;  // ignore empty lines
      buf[idx] = '\0';
      idx = 0;

      // Parse "L,R"
      char *tok = strtok(buf, ",");
      if (tok) {
        int l = atoi(tok);
        tok = strtok(NULL, ",");
        if (tok) {
          int r = atoi(tok);

          // Update globals (atomic-ish window)
          noInterrupts();
          motorLeftPWM  = constrain(l, -255, 255);
          motorRightPWM = constrain(r, -255, 255);
          interrupts();

          // Apply immediately
          setSpeed(motorLeftPWM, motorRightPWM);
        }
      }
    } else {
      if (idx < sizeof(buf) - 1) {
        buf[idx++] = ch;
      }
    }
  }
}
