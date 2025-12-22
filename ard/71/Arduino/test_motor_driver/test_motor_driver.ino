// Motor A
#define PWMA 12
#define AIN2 11
#define AIN1 10
// Standby
#define STBY 9
// Motor B
#define BIN1 8
#define BIN2 7
#define PWMB 6


void setup() {
  Serial.begin(9600);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(STBY, OUTPUT);

  digitalWrite(STBY, HIGH); // Activate driver
  Serial.println("Motors should work now");
    // Motor A forward
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
  analogWrite(PWMA, 200);

  // Motor B backward
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);
  analogWrite(PWMB, 200);
  delay(2000);
  analogWrite(PWMA, 0);
  analogWrite(PWMB, 0);
}

void loop() {
  // Motor A forward
//  digitalWrite(AIN1, HIGH);
//  digitalWrite(AIN2, LOW);
//  analogWrite(PWMA, 200);
//
//  // Motor B backward
//  digitalWrite(BIN1, HIGH);
//  digitalWrite(BIN2, LOW);
//  analogWrite(PWMB, 200);

  delay(2000);

  // Stop motors
  analogWrite(PWMA, 0);
  analogWrite(PWMB, 0);

  delay(2000);
}
