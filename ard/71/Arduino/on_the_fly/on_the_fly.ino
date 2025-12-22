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



void setup() {
  Serial.begin(115200);
  

  int motorPins[] = {AIN1, AIN2, APWM, BIN1, BIN2, BPWM, STBY};
  for (int i = 0; i < 7; i++) pinMode(motorPins[i], OUTPUT);
  digitalWrite(STBY, HIGH);
  setSpeed(0, 0);
  //setSpeed(255,247 );
}

void loop() {
  
 
}

void moveMotor(int in1, int in2, int pwm, bool forward, int speed) {
  digitalWrite(in1, forward ? HIGH : LOW);
  digitalWrite(in2, forward ? LOW : HIGH);
  analogWrite(pwm, speed);
}

void setSpeed(int leftSpeed, int rightSpeed) {
  // Left motor (B)
  moveMotor(BIN1, BIN2, BPWM, false, leftSpeed);
  
  // Right motor (A)
  moveMotor(AIN1, AIN2, APWM, false, rightSpeed);
}
