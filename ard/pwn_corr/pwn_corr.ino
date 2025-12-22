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
#define ENCA1 21 // Right  // Motor A encoder channel A
#define ENCA2 20 // Right  // Motor A encoder channel B
#define ENCB1 19 // Left   // Motor B encoder channel A
#define ENCB2 18 // Left   // Motor B encoder channel B


#define PULSES_PER_REV 1857
#define debug false  // for print statements

volatile long encoderCountA = 0;
volatile long encoderCountB = 0;

unsigned long ul_PreviousMillis = 0UL;  // using millis rollver code from http://playground.arduino.cc/Code/TimingRollover
unsigned long ul_Interval = 50UL;       // 50ms this is delay between corrections - non blocking to allow interrupts to count encoder pulses

int pwm_max = 255;            // (pwm is 0-255)
int cur_pwm1 = 0;   // the speed we will run at - half speed (pwm is 0-255)
int cur_pwm2 = 0;   // the speed we will run at - half speed (pwm is 0-255)
int cpr = 1857;                // the click per rev for the encoder
int rpm_max = 60;            // the 12V open loop RPM
int rpm_target1 = 0;
int rpm_target2 = 0;


#define bluetooth Serial2
bool newData = false;
bool IsInForward = true;




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
  if (ul_CurrentMillis - ul_PreviousMillis > ul_Interval) {
    // the count can be used to find the counts per revolution
    if (debug) Serial.print("Count: ");
    //Serial.println(counter1);
    
    // these lines will show rpm, but zero the counter
    //Serial.print((float)ul_Interval/1000.0);
    int rpm1 = ((float)encoderCountA/((float)cpr * ((float)ul_Interval/1000.0))) * 60.0;
    int rpm2 = ((float)encoderCountB/((float)cpr * ((float)ul_Interval/1000.0))) * 60.0;
    if (debug) Serial.print("A: ");
    if (debug) Serial.print(encoderCountA);
    if (debug) Serial.print(", RPM: ");
    if (debug) Serial.print(rpm1);

    if (debug) Serial.print(",B: ");
    if (debug) Serial.print(encoderCountB);
    if (debug) Serial.print(", RPM: ");
    if (debug) Serial.print(rpm2);

    // kind of a messy P only correction
    // use the error * half the pwn range
    float error1 = ((float)abs(rpm_target1) - (float)rpm1);
    float error2 = ((float)abs(rpm_target2) - (float)rpm2);
    float correction1 = ((error1 / (float)rpm_max) / 2.0) * (float)pwm_max;
    float correction2 = ((error2 / (float)rpm_max) / 2.0) * (float)pwm_max;
    cur_pwm1 = min(255, max(0, cur_pwm1 + correction1));
    cur_pwm2 = min(255, max(0, cur_pwm2 + correction2));
    if (debug) Serial.print(", PWMA: ");
    if (debug) Serial.print(cur_pwm1);
    if (debug) Serial.print(", PWMB: ");
    if (debug) Serial.println(cur_pwm2);

    if(!debug) Serial.print(rpm1);
    if(!debug) Serial.print(",");
    if(!debug) Serial.println(rpm2);
    //analogWrite(pin_ENA, cur_pwm1);
    setSpeed(cur_pwm2,cur_pwm1);
    


    encoderCountA = 0;  // reset the counter for the next loop
    encoderCountB = 0;  // reset the counter for the next loop
    ul_PreviousMillis = millis();  // reset the timer for the next correction
  }
  receiveData(Serial);
  receiveData(bluetooth);

}




void moveMotor(int in1, int in2, int pwm, bool forward, int speed) {
  digitalWrite(in1, forward ? HIGH : LOW);
  digitalWrite(in2, forward ? LOW : HIGH);
  analogWrite(pwm, speed);
}




void setSpeed(int leftSpeed,int rightSpeed){




  // Run both motors backward at full speed
  if (rpm_target1 > 0 )
  {
  moveMotor(AIN1, AIN2, APWM, true, leftSpeed);
    
  }
  else
  {
  moveMotor(AIN1, AIN2, APWM, false, abs(leftSpeed));
  }

  if (rpm_target2 > 0)
  {
  moveMotor(BIN1, BIN2, BPWM, true, rightSpeed);
  }
  else
  { 
  moveMotor(BIN1, BIN2, BPWM, false, abs(rightSpeed));
  }
  
  
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
      rpm_target2 = atoi(strtokIndx);
      strtokIndx = strtok(NULL, ",");
      if (strtokIndx != NULL) {
        rpm_target1 = atoi(strtokIndx);
         
         
        //setSpeed(rpm_target1, rpm_target2);
      }
    }
    newData = false;
  }
}
