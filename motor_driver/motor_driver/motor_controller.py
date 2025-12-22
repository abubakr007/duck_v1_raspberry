from gpiozero import Motor, PWMOutputDevice
import time

def main(args=None):
    # Left motor (Motor A)
    left_motor = Motor(forward=26, backward=27)  # IN1, IN2
    left_pwm = PWMOutputDevice(25)               # ENA

    # Right motor (Motor B)
    right_motor = Motor(forward=22, backward=23) # IN3, IN4
    right_pwm = PWMOutputDevice(17)              # ENB

    # Forward at half speed
    left_motor.forward()
    right_motor.forward()
    left_pwm.value = 1
    right_pwm.value = 1
    time.sleep(3)

    # Backward at half speed
    left_motor.backward()
    right_motor.backward()
    left_pwm.value = 1
    right_pwm.value = 1
    time.sleep(3)

    # Stop motors
    left_motor.stop()
    right_motor.stop()
    left_pwm.value = 0.0
    right_pwm.value = 0.0


# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import Twist
# import platform

# #is_raspberry_pi = platform.system() == 'Linux' and 'raspberrypi' in platform.uname().nodename.lower()
# is_raspberry_pi = platform.system() == 'Linux' and 'raspberrypi' in platform.uname().node.lower()


# if is_raspberry_pi:
#     import RPi.GPIO as GPIO

#     ENA = 25
#     IN1 = 26
#     IN2 = 27

#     GPIO.setmode(GPIO.BCM)
#     GPIO.setup(ENA, GPIO.OUT)
#     GPIO.setup(IN1, GPIO.OUT)
#     GPIO.setup(IN2, GPIO.OUT)
#     pwm = GPIO.PWM(ENA, 1000)
#     pwm.start(0)
# else:
#     print("WARNING: Not running on Raspberry Pi. GPIO will not be used.")

# class MotorController(Node):
#     def __init__(self):
#         super().__init__('motor_controller')
#         self.subscription = self.create_subscription(
#             Twist,
#             '/cmd_vel',
#             self.listener_callback,
#             10)
    
#     def listener_callback(self, msg):
#         speed = msg.linear.x

#         if is_raspberry_pi:
#             if speed > 0:
#                 GPIO.output(IN1, GPIO.HIGH)
#                 GPIO.output(IN2, GPIO.LOW)
#             elif speed < 0:
#                 GPIO.output(IN1, GPIO.LOW)
#                 GPIO.output(IN2, GPIO.HIGH)
#             else:
#                 GPIO.output(IN1, GPIO.LOW)
#                 GPIO.output(IN2, GPIO.LOW)

#             pwm.ChangeDutyCycle(min(abs(speed) * 100, 100))
#         else:
#             print(f"[SIMULATION] Speed: {speed}")

# def main(args=None):
#     rclpy.init(args=args)
#     node = MotorController()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

#     if is_raspberry_pi:
#         GPIO.cleanup()

if __name__ == '__main__':
    main()
