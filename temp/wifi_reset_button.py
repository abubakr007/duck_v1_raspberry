from gpiozero import Button, LED
import time
from signal import pause
import subprocess

button = Button(17, pull_up=True, bounce_time=0.2)
led = LED(27)
def restart_ssh():
    print("Button pressed! Restarting SSH service...")
    subprocess.run(
        ["sudo", "systemctl", "restart", "ssh"],
        check=False
    )
    print("SSH restart command sent")
    led.on()
    time.sleep(2)
    led.off()

button.when_pressed = restart_ssh

pause()
