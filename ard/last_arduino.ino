#include <PID_v1.h>

// ----- Bluetooth (NEW) -----
#define bluetooth Serial2

// TB6612FNG Motor Driver Connection PINs
#define TB6612_enA 9
#define TB6612_enB 8
#define TB6612_in4 7
#define TB6612_in3 6
#define TB6612_in2 4
#define TB6612_in1 3

#define STBY 5

// Wheel Encoders Connection PINs
#define right_encoder_phaseA 20
#define right_encoder_phaseB 21
#define left_encoder_phaseA 18
#define left_encoder_phaseB 19

// Encoders
volatile unsigned int right_encoder_counter = 0;
volatile unsigned int left_encoder_counter  = 0;
volatile char right_wheel_sign = 'p';
volatile char left_wheel_sign  = 'p';
unsigned long last_millis = 0;
const unsigned long interval = 100;

// PID
double right_wheel_cmd_vel = 0.0;
double left_wheel_cmd_vel  = 0.0;
double right_wheel_meas_vel = 0.0;
double left_wheel_meas_vel  = 0.0;
double right_wheel_cmd = 0.0;
double left_wheel_cmd  = 0.0;
double Kp_r = 20.0, Ki_r = 60.0, Kd_r = 0.2;
double Kp_l = 22.0, Ki_l = 65.0, Kd_l = 0.2;
PID rightMotor(&right_wheel_meas_vel, &right_wheel_cmd, &right_wheel_cmd_vel, Kp_r, Ki_r, Kd_r, DIRECT);
PID leftMotor (&left_wheel_meas_vel,  &left_wheel_cmd,  &left_wheel_cmd_vel,  Kp_l, Ki_l, Kd_l, DIRECT);

// Command buffer
String commandBuffer = "";
const int MAX_BUFFER_SIZE = 64;
bool sendFeedback = false;
char last_r_sign = 'p';
char last_l_sign = 'p';

void setup() {
  // TB6612FNG pins
  pinMode(TB6612_enA, OUTPUT);
  pinMode(TB6612_enB, OUTPUT);
  pinMode(TB6612_in1, OUTPUT);
  pinMode(TB6612_in2, OUTPUT);
  pinMode(TB6612_in3, OUTPUT);
  pinMode(TB6612_in4, OUTPUT);
  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);
  // Default direction
  digitalWrite(TB6612_in1, HIGH);
  digitalWrite(TB6612_in2, LOW);
  digitalWrite(TB6612_in3, HIGH);
  digitalWrite(TB6612_in4, LOW);

  rightMotor.SetMode(AUTOMATIC);
  leftMotor.SetMode(AUTOMATIC);
  // Match PID sample time to the 100ms control loop interval
  rightMotor.SetSampleTime(interval);
  leftMotor.SetSampleTime(interval);

  Serial.begin(115200);
  Serial.setTimeout(10);
  bluetooth.begin(9600);

  // Encoders
  pinMode(right_encoder_phaseB, INPUT);
  pinMode(left_encoder_phaseB, INPUT);
  attachInterrupt(digitalPinToInterrupt(right_encoder_phaseA), rightEncoderCallback, RISING);
  attachInterrupt(digitalPinToInterrupt(left_encoder_phaseA),  leftEncoderCallback,  RISING);

  commandBuffer.reserve(MAX_BUFFER_SIZE);
}

void loop() {
  // Process incoming serial data (non-blocking)
  processSerialCommands();

  // Encoder + control tick @100ms
  unsigned long current_millis = millis();
  if (current_millis - last_millis >= interval) {
    // Disable interrupts briefly to safely read encoder counters and signs together
    noInterrupts();
    unsigned int right_count = right_encoder_counter;
    unsigned int left_count = left_encoder_counter;
    char r_sign = right_wheel_sign;
    char l_sign = left_wheel_sign;
    right_encoder_counter = 0;
    left_encoder_counter = 0;
    interrupts();

    // Calculate velocities (rad/s)
    right_wheel_meas_vel = (10 * right_count * (60.0/1859.41)) * 0.10472;
    left_wheel_meas_vel  = (10 * left_count  * (60.0/1864.87)) * 0.10472;

    // Run PID
    rightMotor.Compute();
    leftMotor.Compute();

    if (right_wheel_cmd_vel == 0.0) right_wheel_cmd = 0.0;
    if (left_wheel_cmd_vel  == 0.0) left_wheel_cmd  = 0.0;

    // Apply motor commands
    analogWrite(TB6612_enA, right_wheel_cmd);
    analogWrite(TB6612_enB, left_wheel_cmd);
    bluetooth.print("right_wheel_cmd:");
    bluetooth.print(right_wheel_cmd);
    bluetooth.print("left_wheel_cmd:");
    bluetooth.println(left_wheel_cmd);

    // Store signs captured atomically for feedback
    last_r_sign = r_sign;
    last_l_sign = l_sign;
    sendFeedback = true;
    last_millis = current_millis;
  }

  // Send feedback AFTER processing any pending commands
  // This prevents corruption from interleaved read/write
  if (sendFeedback && Serial.availableForWrite() > 50) {
    String encoder_read = "r" + String(last_r_sign) + String(right_wheel_meas_vel, 2)
                        + ",l" + String(last_l_sign) + String(left_wheel_meas_vel, 2) + ",";
    Serial.println(encoder_read);
    sendFeedback = false;
  }
}

void processSerialCommands() {
  // Process multiple commands per loop to clear buffer quickly
  int commandsProcessed = 0;
  while (Serial.available() > 0 && commandsProcessed < 5) {
    char chr = Serial.read();
    //bluetooth.write(chr);
    
    // Add to buffer (skip newlines/carriage returns)
    if (chr != '\n' && chr != '\r') {
      commandBuffer += chr;
      
      // Prevent overflow
      if (commandBuffer.length() > MAX_BUFFER_SIZE) {
        commandBuffer = "";
      }
    }
    
    // Process complete command (ends with ,,)
    if (commandBuffer.endsWith(",,")) {
      parseCommand(commandBuffer);
      commandBuffer = "";
      commandsProcessed++;
    }
  }
}

void parseCommand(String cmd) {
  // Expected format: "rnXX.XX,lnXX.XX,,"
  // Remove trailing ,,
  cmd.replace(",,", "");
  
  int firstComma = cmd.indexOf(',');
  if (firstComma == -1) return;
  
  // Extract right wheel command
  String rightCmd = cmd.substring(0, firstComma);
  if (rightCmd.length() >= 3 && rightCmd.charAt(0) == 'r') {
    char dir = rightCmd.charAt(1);
    String value = rightCmd.substring(2);
    
    // Validate value is numeric
    if (isValidNumber(value)) {
      // Set direction
      if (dir == 'p') {
        digitalWrite(TB6612_in1, HIGH);
        digitalWrite(TB6612_in2, LOW);
      } else if (dir == 'n') {
        digitalWrite(TB6612_in1, LOW);
        digitalWrite(TB6612_in2, HIGH);
      }

      right_wheel_cmd_vel = value.toFloat();
    }
  }

  // Extract left wheel command
  String leftCmd = cmd.substring(firstComma + 1);
  if (leftCmd.length() >= 3 && leftCmd.charAt(0) == 'l') {
    char dir = leftCmd.charAt(1);
    String value = leftCmd.substring(2);

    // Validate value is numeric
    if (isValidNumber(value)) {
      // Set direction
      if (dir == 'p') {
        digitalWrite(TB6612_in3, HIGH);
        digitalWrite(TB6612_in4, LOW);
      } else if (dir == 'n') {
        digitalWrite(TB6612_in3, LOW);
        digitalWrite(TB6612_in4, HIGH);
      }
      
      left_wheel_cmd_vel = value.toFloat();
    }
  }
}

bool isValidNumber(String str) {
  if (str.length() == 0) return false;
  for (unsigned int i = 0; i < str.length(); i++) {
    char c = str.charAt(i);
    if (c != '.' && c != '-' && !isDigit(c)) {
      return false;
    }
  }
  return true;
}

void rightEncoderCallback() {
  right_wheel_sign = (digitalRead(right_encoder_phaseB) == HIGH) ? 'p' : 'n';
  right_encoder_counter++;
}

void leftEncoderCallback() {
  left_wheel_sign = (digitalRead(left_encoder_phaseB) == HIGH) ? 'n' : 'p';
  left_encoder_counter++;
}
