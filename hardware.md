
Hardware:
Raspberry Pi 5
Arduino Mega 2560

Dual Motor Driver Module 1A TB6612FNG for Arduino Microcontroller

Onewer Dc Motor Wheel, DC 12V Encoder Motor Shaft DC Gear Motor Dc Gear Motor Encoder Motor Set for Robot for Printer(60RPM)

the wheel radius : 3.35 cm
wheel separation : 17 cm
wheel thickness : 2.7 cm

the Arduino Mega 2560 is connected to Raspberry Pi 5 via USB on port /dev/ttyACM0 at 115200 baud (8N1)

Slamtec RPLiDAR C1 laser scanner connected via USB on port /dev/ttyUSB0 at 460800 baud

MPU6050 IMU connected via I2C bus 1 (/dev/i2c-1) at address 0x68

Robot base dimensions: 25.7 cm x 14 cm x 5 cm
Front caster wheel radius: 1.5 cm

Motor driver pin configuration (TB6612FNG):
  enA (Right motor PWM) : Pin 9
  enB (Left motor PWM)  : Pin 8
  IN1 (Right dir A)     : Pin 3
  IN2 (Right dir B)     : Pin 4
  IN3 (Left dir A)      : Pin 6
  IN4 (Left dir B)      : Pin 7
  STBY (Standby enable) : Pin 5

Encoder pins:
  Right Phase A (interrupt) : Pin 20
  Right Phase B (direction) : Pin 21
  Left Phase A (interrupt)  : Pin 18
  Left Phase B (direction)  : Pin 19
  Right wheel PPR (calibrated) : 1859.41
  Left wheel PPR (calibrated)  : 1864.87

Bluetooth module on Serial2 at 9600 baud (debug output)
