//#include "TimerOne.h"
#include <SoftwareSerial.h>
#include <NewPing.h>

#define TRIG_PIN 23
#define ECHO_PIN 25
#define MAX_DIST 200 // in cm

NewPing sonar(TRIG_PIN, ECHO_PIN, MAX_DIST);

// Define the SoftwareSerial pins for HC-06
#define BT_RX_PIN 17  // Connect to HC-06 TX
#define BT_TX_PIN 16  // Connect to HC-06 RX
// Create SoftwareSerial instance for Bluetooth
//SoftwareSerial bluetooth(BT_RX_PIN, BT_TX_PIN);
#define bluetooth Serial2


volatile int pulseCountLeftRear = 0;
volatile int pulseCountLeftFront = 0;
volatile int pulseCountRightRear = 0;
volatile int pulseCountRightFront = 0;
float speedLeftRear = 0; // in units per second
float speedLeftFront = 0; // in units per second
float speedRightRear = 0; // in units per second
float speedRightFront = 0; // in units per second
float diskslots = 20;  // Change to match value of encoder disk
unsigned long lastTime = 0;

#define LeftRearSpeedPin 19
#define LeftFrontSpeedPin 21
#define RightRearSpeedPin 18
#define RightFrontSpeedPin 20

// Motor A pins
#define LeftRearIN1 40
#define LeftRearIN2 42
#define LeftRearPWM 2

// Motor B pins
#define LeftFrontIN1 36
#define LeftFrontIN2 34
#define LeftFrontPWM 3

// Motor C pins (2nd TB6612)
#define RightRearIN1 28
#define RightRearIN2 30
#define RightRearPWM 4

// Motor D pins
#define RightFrontIN1 24
#define RightFrontIN2 22
#define RightFrontPWM 5
// Standby pin
#define LeftSTBY 38
#define RightSTBY 26

// Motor control variables
int motor1Speed = 0;
int motor2Speed = 0;
bool newData = false;

void setup() {
  Serial.begin(115200);
  //Timer1.initialize(1000000); // set timer for 1sec
  // Start Bluetooth serial communication
  bluetooth.begin(9600);
  // Set all control pins as output
  int motorPins[] = {LeftRearIN1, LeftRearIN2, LeftRearPWM, LeftFrontIN1, LeftFrontIN2, LeftFrontPWM, RightRearIN1, RightRearIN2, RightRearPWM, RightFrontIN1, RightFrontIN2, RightFrontPWM, LeftSTBY,RightSTBY};
  for (int i = 0; i < 13; i++) pinMode(motorPins[i], OUTPUT);
  
  // Enable motors
  digitalWrite(LeftSTBY, HIGH);
  digitalWrite(RightSTBY, HIGH);

  pinMode(LeftRearSpeedPin, INPUT); // D0 connected to pin 2
  pinMode(LeftFrontSpeedPin, INPUT); // D0 connected to pin 2
  pinMode(RightRearSpeedPin, INPUT); // D0 connected to pin 2
  pinMode(RightFrontSpeedPin, INPUT); // D0 connected to pin 2

  attachInterrupt(digitalPinToInterrupt(LeftRearSpeedPin), LeftRearCountPulse, RISING);
  attachInterrupt(digitalPinToInterrupt(LeftFrontSpeedPin), LeftFrontCountPulse, RISING);
  attachInterrupt(digitalPinToInterrupt(RightRearSpeedPin), RightRearCountPulse, RISING);
  attachInterrupt(digitalPinToInterrupt(RightFrontSpeedPin), RightFrontCountPulse, RISING);
  
  
  //move_all_forward();
  //Timer1.attachInterrupt( ISR_timerone ); // Enable the timer

  move_all_forward();
  delay(2000);
  stopMotors();
  
}

// TimerOne ISR
void ISR_timerone()
{
  //Timer1.detachInterrupt();  // Stop the timer
  Serial.print("LR: "); 
  bluetooth.print("LR: ");
  float countLeftRear = (pulseCountLeftRear / diskslots) * 60.00;  // calculate RPM for Motor 1
  Serial.print(countLeftRear);  
  bluetooth.print(countLeftRear);
  //Serial.print(" RPM - "); 
  pulseCountLeftRear = 0;  //  reset counter to zero
  Serial.print(" ,LF:"); 
  bluetooth.print(" ,LF:");
  float countLeftFront = (pulseCountLeftFront / diskslots) * 60.00;  // calculate RPM for Motor 2
  Serial.print(countLeftFront);  
  bluetooth.print(countLeftFront);
  //Serial.print(" RPM"); 
  pulseCountLeftFront = 0;
  Serial.print(" ,RR:"); 
  bluetooth.print(" ,RR:");
  float countRightRear = (pulseCountRightRear / diskslots) * 60.00;  // calculate RPM for Motor 1
  Serial.print(countRightRear);  
  bluetooth.print(countRightRear);
  //Serial.print(" RPM - "); 
  pulseCountRightRear = 0;  //  reset counter to zero
  Serial.print(" ,RF:"); 
  bluetooth.print(" ,RF:");
  float countRightFront = (pulseCountRightFront / diskslots) * 60.00;  // calculate RPM for Motor 2
  Serial.println(countRightFront);  
  bluetooth.println(countRightFront);
  //bluetooth.write("\n");
  pulseCountRightFront = 0;
  //Serial.println(" RPM"); 

  //counter2 = 0;  //  reset counter to zero
  //Timer1.attachInterrupt( ISR_timerone );  // Enable the timer
}

void LeftRearCountPulse() {
  pulseCountLeftRear++;
}
void LeftFrontCountPulse() {
  pulseCountLeftFront++;
}
void RightRearCountPulse() {
  pulseCountRightRear++;
}
void RightFrontCountPulse() {
  pulseCountRightFront++;
}

void loop() {
  // int distance = sonar.ping_cm();
  // Serial.print("distance: ");
  // Serial.println(distance);
  receiveData();
  
}

void receiveData() {
  static byte ndx = 0;
  static char receivedChars[16]; // Buffer to hold incoming data
  char rc;
  
  while (bluetooth.available() > 0 && newData == false) {
    
    rc = bluetooth.read();
    
    if (rc != '\n' && rc != '\r') {
      receivedChars[ndx] = rc;
      ndx++;
      if (ndx >= sizeof(receivedChars) - 1) {
        ndx = sizeof(receivedChars) - 1; // Prevent buffer overflow
      }
    }
    else {
      receivedChars[ndx] = '\0'; // Terminate the string
      ndx = 0;
      newData = true;
    }
  }
  
  if (newData) {
    // Parse the data (format: "255,255")
    char * strtokIndx; // Used by strtok() for parsing
    
    strtokIndx = strtok(receivedChars, ","); // Get first part
    motor1Speed = atoi(strtokIndx); // Convert to integer
    
    strtokIndx = strtok(NULL, ","); // Get second part
    motor2Speed = atoi(strtokIndx);
    
    // Debug output
    Serial.print("Motor 1: ");
    Serial.print(motor1Speed);
    Serial.print(", Motor 2: ");
    Serial.println(motor2Speed);
    
    newData = false;
    setSpeed(motor1Speed,motor2Speed);
    //delay(400);
  }
}
void move_all_forward(){
  // Run both motors backward at full speed
  moveMotor(LeftRearIN2,LeftRearIN1 , LeftRearPWM, true, 255);
    moveMotor(LeftFrontIN1, LeftFrontIN2, LeftFrontPWM, true, 255);

    moveMotor(RightRearIN2, RightRearIN1, RightRearPWM, true, 255);
    moveMotor(RightFrontIN1, RightFrontIN2, RightFrontPWM, true, 255);
    
  
}

void setSpeed(int leftSpeed,int rightSpeed){
  // Run both motors backward at full speed
  if (leftSpeed > 0 )
  {
    moveMotor(LeftRearIN2,LeftRearIN1 , LeftRearPWM, true, leftSpeed);
    moveMotor(LeftFrontIN1, LeftFrontIN2, LeftFrontPWM, true, leftSpeed);
  }
  else
  {

moveMotor(LeftRearIN1,LeftRearIN2 , LeftRearPWM, true, leftSpeed);
    moveMotor(LeftFrontIN2, LeftFrontIN1, LeftFrontPWM, true, leftSpeed);
  }

  if (rightSpeed > 0)
  {

    moveMotor(RightRearIN2, RightRearIN1, RightRearPWM, true, rightSpeed);
    moveMotor(RightFrontIN1, RightFrontIN2, RightFrontPWM, true, rightSpeed);
  }
  else
  {
    moveMotor(RightRearIN1, RightRearIN2, RightRearPWM, true, rightSpeed);
    moveMotor(RightFrontIN2,RightFrontIN1 , RightFrontPWM, true, rightSpeed);

  }
  
  
}


// Function to move a motor
void moveMotor(int in1, int in2, int pwm, bool forward, int speed) {
  digitalWrite(in1, forward ? HIGH : LOW);
  digitalWrite(in2, forward ? LOW : HIGH);
  analogWrite(pwm, speed);
}

// Function to stop both motors
void stopMotors() {
  analogWrite(LeftRearPWM, 0);
  analogWrite(LeftFrontPWM, 0);

  analogWrite(RightRearPWM, 0);
  analogWrite(RightFrontPWM, 0);

}
