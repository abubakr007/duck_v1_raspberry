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

// Controller gains
float Kp                  = 10;     // Proportional gain
float Ki                  = 20;     // Integral gain

// Control law variables
float Ts                  = 1000;   // Sample time in microseconds. Recalculated on each sample.
float leftReference       = 0;
float rightReference      = 0;
float leftError           = 0;
float rightError          = 0;
float leftControllaw      = 0;
float rightControllaw     = 0;
float leftIntegrat        = 0;
float rightIntegrat       = 0;

// Encoder variables
volatile long leftEncoderval    = 0;
volatile long rightEncoderval   = 0;
float leftMspeed          = 0;
float rightMspeed         = 0;

// Previous sample data
float leftPrevintegrat    = 0;
float rightPrevintegrat   = 0;
float leftPreverror       = 0;
float rightPreverror      = 0;
unsigned long prevtime    = 0;
long leftPrevencoderval   = 0;
long rightPrevencoderval  = 0;

// Calculated PWM
unsigned char leftPwm;
unsigned char rightPwm;

// Serial port variables
int availablebytes        = 0;
char readreference[10]    = {'0','0','0','0','0','0','0','0','0','0'};

void setup()
{
  // Attach interrupts for encoders
  attachInterrupt(digitalPinToInterrupt(LEFT_ENC_A), leftEncoderInt, RISING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_A), rightEncoderInt, RISING);
  
  // Set encoder B pins as inputs
  pinMode(LEFT_ENC_B, INPUT);
  pinMode(RIGHT_ENC_B, INPUT);
  
  // Set motor control pins as outputs
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(APWM, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(BPWM, OUTPUT);
  pinMode(STBY, OUTPUT);
  
  // Initialize motor directions (both forward)
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);
  
  // Enable motors
  digitalWrite(STBY, HIGH);
  
  Serial.begin(115200);
}

void loop()
{
  availablebytes = Serial.available();
  if(availablebytes > 0)
  {
    // Clear buffer
    for(int i = 0; i < 10; i++) {
      readreference[i] = '0';
    }
    
    Serial.readBytes(readreference, 10);
    // Parse two references separated by comma or space
    // Format expected: "left,right" or "left right"
    String input = String(readreference);
    int separatorIndex = input.indexOf(',');
    if(separatorIndex == -1) {
      separatorIndex = input.indexOf(' ');
    }
    
    if(separatorIndex != -1) {
      leftReference = input.substring(0, separatorIndex).toFloat();
      rightReference = input.substring(separatorIndex + 1).toFloat();
    } else {
      // If no separator found, use same reference for both motors
      leftReference = rightReference = atof(readreference);
    }
    
    // Reset encoders
    leftEncoderval = 0;
    rightEncoderval = 0;
    leftPrevencoderval = 0;
    rightPrevencoderval = 0;
  }
  
  // Measuring motor speeds
  leftMspeed = (float)(leftEncoderval - leftPrevencoderval);
  leftPrevencoderval = leftEncoderval;
  rightMspeed = (float)(rightEncoderval - rightPrevencoderval);
  rightPrevencoderval = rightEncoderval;
  
  // Convert to rounds/sec (assuming 2000 pulses per revolution)
  leftMspeed = leftMspeed * 1000 / (2 * Ts);
  rightMspeed = rightMspeed * 1000 / (2 * Ts);
  
  // Calculate errors
  leftError = leftReference - leftMspeed;
  rightError = rightReference - rightMspeed;
  
  // Calculate integrals
  leftIntegrat = leftPrevintegrat + (leftPreverror / 1000000) * Ts;
  rightIntegrat = rightPrevintegrat + (rightPreverror / 1000000) * Ts;
  leftPrevintegrat = leftIntegrat;
  rightPrevintegrat = rightIntegrat;
  leftPreverror = leftError;
  rightPreverror = rightError;
  
  // Calculate control laws
  leftControllaw = leftError * Kp + Ki * leftIntegrat;
  rightControllaw = rightError * Kp + Ki * rightIntegrat;
  
  // Apply saturation
  if(leftControllaw > 12) leftControllaw = 12;
  if(leftControllaw < -12) leftControllaw = -12;
  if(rightControllaw > 12) rightControllaw = 12;
  if(rightControllaw < -12) rightControllaw = -12;
  
  // Set left motor direction
  if(leftControllaw > 0)
  {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
  }
  else if(leftControllaw < 0)
  {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
  }
  
  // Set right motor direction
  if(rightControllaw > 0)
  {
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
  }
  else if(rightControllaw < 0)
  {
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
  }
  
  // Update motor speeds
  Serial.print("Left: ");
  Serial.print(leftMspeed);
  Serial.print(", Right: ");
  Serial.println(rightMspeed);
  
  // Calculate and apply PWM
  leftPwm = (abs(leftControllaw) / 12) * 255;
  rightPwm = (abs(rightControllaw) / 12) * 255;
  analogWrite(APWM, leftPwm);
  analogWrite(BPWM, rightPwm);
  
  // Measure sample time
  Ts = micros() - prevtime;
  prevtime = micros();
}

void leftEncoderInt()
{
  if(digitalRead(LEFT_ENC_B) == HIGH) {
    leftEncoderval++;
  } else {
    leftEncoderval--;
  }
}

void rightEncoderInt()
{
  if(digitalRead(RIGHT_ENC_B) == HIGH) {
    rightEncoderval++;
  } else {
    rightEncoderval--;
  }
}
