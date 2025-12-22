// Motor AA
#define APWMA 12
#define AAIN2 11
#define AAIN1 10
// Standby
#define ASTBY 9
// Motor AB
#define ABIN1 8
#define ABIN2 7
#define APWMB 6

// Motor BA
#define BPWMA 4
#define BAIN2 19
#define BAIN1 18
// Standby
#define BSTBY 17
// Motor BB
#define BBIN1 16
#define BBIN2 15
#define BPWMB 14


void setup() {
  pinMode(AAIN1, OUTPUT);
  pinMode(AAIN2, OUTPUT);
  pinMode(APWMA, OUTPUT);
  pinMode(ABIN1, OUTPUT);
  pinMode(ABIN2, OUTPUT);
  pinMode(APWMB, OUTPUT);
  pinMode(ASTBY, OUTPUT);

  digitalWrite(ASTBY, HIGH); // Activate driver

  pinMode(BAIN1, OUTPUT);
  pinMode(BAIN2, OUTPUT);
  pinMode(BPWMA, OUTPUT);
  pinMode(BBIN1, OUTPUT);
  pinMode(BBIN2, OUTPUT);
  pinMode(BPWMB, OUTPUT);
  pinMode(BSTBY, OUTPUT);

  digitalWrite(BSTBY, HIGH); // Activate driver
}

void loop() {
  // Motor AA forward
  digitalWrite(AAIN1, HIGH);
  digitalWrite(AAIN2, LOW);
  analogWrite(APWMA, 200);

  // Motor AB backward
  digitalWrite(ABIN1, HIGH);
  digitalWrite(ABIN2, LOW);
  analogWrite(APWMB, 200);

  // Motor BA forward
  digitalWrite(BAIN1, HIGH);
  digitalWrite(BAIN2, LOW);
  analogWrite(BPWMA, 200);

  // Motor BB backward
  digitalWrite(BBIN1, HIGH);
  digitalWrite(BBIN2, LOW);
  analogWrite(BPWMB, 200);
  delay(2000);

  // Stop motors
  analogWrite(APWMA, 0);
  analogWrite(APWMB, 0);
  analogWrite(BPWMA, 0);
  analogWrite(BPWMB, 0);

  delay(2000);
}
