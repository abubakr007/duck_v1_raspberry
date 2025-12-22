volatile int pulseCount1 = 0;
volatile int pulseCount2 = 0;
unsigned long lastTime = 0;
float speed1 = 0; // in units per second
float speed2 = 0; // in units per second
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
  Serial.begin(115200);
  pinMode(3, INPUT); // D0 connected to pin 2
  pinMode(2, INPUT); // D0 connected to pin 2
  attachInterrupt(digitalPinToInterrupt(3), countPulse1, RISING);
  attachInterrupt(digitalPinToInterrupt(2), countPulse2, RISING);

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

  // Stop motors
  analogWrite(APWMA, 0);
  analogWrite(APWMB, 0);
  analogWrite(APWMA, 0);
  analogWrite(APWMB, 0);
  turnOnAllMotors();
}

void loop() {
  unsigned long currentTime = millis();

  // Every 1000ms (1 second), calculate speed
  if (currentTime - lastTime >= 1000) {
    noInterrupts();
    int count1 = pulseCount1;
    pulseCount1 = 0;
    int count2 = pulseCount2;
    pulseCount2 = 0;
    interrupts();

    // Suppose 1 pulse = 1 rotation and wheel circumference = 20cm
    float distancePerPulse = 0.20; // meters
    speed1 = count1 * distancePerPulse; // meters per second
    speed2 = count2 * distancePerPulse; // meters per second

    Serial.print("L:");
    Serial.print(speed1);
    Serial.print(" R:");
    Serial.println(speed2);

    lastTime = currentTime;
  }

  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    int sep = input.indexOf(' ');
    if (sep > 0) {
      int left = input.substring(2, sep).toInt();
      int right = input.substring(sep + 3).toInt();
      setLeftSpeed(constrain(left, -255, 255));
      setRightSpeed(constrain(right, -255, 255));
      
    }
  }
}

void turnOnRightMotors()
{
  digitalWrite(AAIN1, HIGH);
  digitalWrite(AAIN2, LOW);
  digitalWrite(ABIN1, HIGH);
  digitalWrite(ABIN2, LOW);
}

void turnOnLeftMotors()
{
  digitalWrite(BAIN1, HIGH);
  digitalWrite(BAIN2, LOW);
  digitalWrite(BBIN1, HIGH);
  digitalWrite(BBIN2, LOW);
}

void turnOnAllMotors()
{
  turnOnRightMotors();
  turnOnLeftMotors();
}
void setLeftSpeed(int speed)
{
  if (speed >= 0) {
    digitalWrite(AAIN1, HIGH);
    digitalWrite(AAIN2, LOW);
    digitalWrite(ABIN1, HIGH);
    digitalWrite(ABIN2, LOW);
  } else {
    digitalWrite(AAIN1, LOW);
    digitalWrite(AAIN2, HIGH);
    digitalWrite(ABIN1, LOW);
    digitalWrite(ABIN2, HIGH);
    speed = -speed;
  }
  // Motor AA
  analogWrite(APWMA, speed);
  // Motor AB 
  analogWrite(APWMB, speed);
}

void setRightSpeed(int speed)
{
  if (speed >= 0) {
    digitalWrite(BAIN1, HIGH);
    digitalWrite(BAIN2, LOW);
    digitalWrite(BBIN1, HIGH);
    digitalWrite(BBIN2, LOW);
  } else {
    digitalWrite(BAIN1, LOW);
    digitalWrite(BAIN2, HIGH);
    digitalWrite(BBIN1, LOW);
    digitalWrite(BBIN2, HIGH);
    speed = -speed;
  }
  // Motor BA
  analogWrite(BPWMA, speed);
  // Motor BB 
  analogWrite(BPWMB, speed);
}

void stopLeft()
{
  // Stop motors
  analogWrite(APWMA, 0);
  analogWrite(APWMB, 0);

}

void stopRight()
{
  analogWrite(BPWMA, 0);
  analogWrite(BPWMB, 0);
}


void countPulse1() {
  pulseCount1++;
}

void countPulse2() {
  pulseCount2++;
}
