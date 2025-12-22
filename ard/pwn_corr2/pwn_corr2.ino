// Left motor
#define AIN1 4
#define AIN2 3
#define APWM 2

// Right motor
#define BIN1 7
#define BIN2 6
#define BPWM 8

#define STBY 5

// Encoder pins
#define ENCA1 21 // Right  // Motor A encoder channel A
#define ENCA2 20 // Right  // Motor A encoder channel B
#define ENCB1 19 // Left   // Motor B encoder channel A
#define ENCB2 18 // Left   // Motor B encoder channel B

#define PULSES_PER_REV 1857
#define debug false  // for print statements

volatile long encoderCountA = 0;
volatile long encoderCountB = 0;

unsigned long ul_PreviousMillis = 0UL;
unsigned long ul_Interval = 50UL; // 50ms between corrections

int pwm_max = 255;
int cur_pwm1 = 0;
int cur_pwm2 = 0;
int cpr = 1857;
int rpm_max = 60;
int rpm_target1 = 0;
int rpm_target2 = 0;

float Kp = 2.0;
float Ki = 0.5;
float Kd = 0.1;

// PID state
float integral1 = 0, integral2 = 0;
float lastError1 = 0, lastError2 = 0;

#define bluetooth Serial2
bool newData = false;

// Interrupts (count pulses)
void encoderA_ISR() {
  encoderCountA++;
}

void encoderB_ISR() {
  encoderCountB++;
}

void setup() {
  Serial.begin(115200);
  bluetooth.begin(9600);

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
  unsigned long ul_CurrentMillis = millis();  
  if (ul_CurrentMillis - ul_PreviousMillis >= ul_Interval) {
    // Calculate RPMs
    float timeElapsed = (ul_CurrentMillis - ul_PreviousMillis) / 1000.0;
    float rpm1 = ((float)encoderCountA / (cpr * timeElapsed)) * 60.0;
    float rpm2 = ((float)encoderCountB / (cpr * timeElapsed)) * 60.0;

    // Reset counters immediately after reading
    encoderCountA = 0;
    encoderCountB = 0;
    
    // Update previous time
    ul_PreviousMillis = ul_CurrentMillis;

    int basePWM = 60;  // experiment with 50-80

    // Calculate PID for motor A (right)
    float error1 = rpm_target1 - rpm1;
    integral1 += error1 * timeElapsed;
    float derivative1 = (error1 - lastError1) / timeElapsed;
    float correction1 = Kp * error1 + Ki * integral1 + Kd * derivative1;
    cur_pwm1 = constrain(basePWM + correction1, 0, pwm_max);
    lastError1 = error1;

    // Calculate PID for motor B (left)
    float error2 = rpm_target2 - rpm2;
    integral2 += error2 * timeElapsed;
    float derivative2 = (error2 - lastError2) / timeElapsed;
    float correction2 = Kp * error2 + Ki * integral2 + Kd * derivative2;
    cur_pwm2 = constrain(basePWM + correction2, 0, pwm_max);
    lastError2 = error2;

    // Debug print
    if (debug) {
      Serial.print("Target1: ");
      Serial.print(rpm_target1);
      Serial.print(" Actual1: ");
      Serial.print(rpm1);
      Serial.print(" PWM1: ");
      Serial.print(cur_pwm1);
      
      Serial.print(" | Target2: ");
      Serial.print(rpm_target2);
      Serial.print(" Actual2: ");
      Serial.print(rpm2);
      Serial.print(" PWM2: ");
      Serial.println(cur_pwm2);
    }

    // Apply speeds with correct direction handling
    setSpeed(cur_pwm2, cur_pwm1, rpm_target2, rpm_target1);
  }

  // Handle serial commands
  receiveData(Serial);
  receiveData(bluetooth);
}

void moveMotor(int in1, int in2, int pwm, bool forward, int speed) {
  digitalWrite(in1, forward ? HIGH : LOW);
  digitalWrite(in2, forward ? LOW : HIGH);
  analogWrite(pwm, speed);
}

void setSpeed(int leftSpeed, int rightSpeed, int leftDirection, int rightDirection) {
  // Left motor (B)
  moveMotor(BIN1, BIN2, BPWM, leftDirection >= 0, leftSpeed);
  
  // Right motor (A)
  moveMotor(AIN1, AIN2, APWM, rightDirection >= 0, rightSpeed);
}

// Command parser (works for Serial + Bluetooth)
void receiveData(Stream &stream) {
  static char receivedChars[16];
  static byte ndx = 0;
  char rc;

  while (stream.available() > 0 && !newData) {
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
      rpm_target2 = atoi(strtokIndx); // Left motor
      strtokIndx = strtok(NULL, ",");
      if (strtokIndx != NULL) {
        rpm_target1 = atoi(strtokIndx); // Right motor
      }
    }
    newData = false;
  }
}
