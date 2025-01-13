import RPi.GPIO as GPIO
import time

class Servo(object):
    def __init__(self, channel, offset=0):
        """
        channel: GPIO pin number (BCM numbering)
        offset: correction offset in degrees
        """
        self.channel = channel  # GPIO pin number
        self.offset = offset
        self._min_pulse = 500   # Min pulse length (0 degrees)
        self._max_pulse = 2500  # Max pulse length (180 degrees)
        self._frequency = 50    # 50Hz for servos

    def setup(self):
        """Initialize GPIO and PWM"""
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.channel, GPIO.OUT)
        self.pwm = GPIO.PWM(self.channel, self._frequency)
        self.pwm.start(0)

    def _angle_to_duty_cycle(self, angle):
        """Convert angle to duty cycle"""
        # Ensure angle is within bounds
        angle = max(0, min(180, angle))
        # Add offset
        angle = angle + self.offset
        # Convert to duty cycle (2-12% is typical for servos)
        duty = angle / 18.0 + 2
        return duty

    def write(self, angle):
        """
        Set servo angle
        angle: 0-180 degrees
        """
        duty = self._angle_to_duty_cycle(angle)
        self.pwm.ChangeDutyCycle(duty)
        # Small delay to allow servo to move
        time.sleep(0.1)

    def stop(self):
        """Clean up GPIO"""
        self.pwm.stop()
        GPIO.cleanup(self.channel)
