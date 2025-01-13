from autorc.nodes import Node
from autorc.config import config
from autorc.utils import range_map

import RPi.GPIO as GPIO
# from autorc.picar3 import PCA9685
# from autorc.picar3.front_wheels import Front_Wheels
# from autorc.picar3.back_wheels import Back_Wheels


class Engine(Node):
    def __init__(self, context, process_rate=60, **kwargs):
        # Error in process_loop, no output, no run
        super(Engine, self).__init__(context, process_rate=process_rate,
                                     **kwargs)
        # Test car
        self.steering = None
        self.throttle = None
        self.motorPinRPWM = 5
        self.motorPinLPWM = 6
        self.motorSpeedPin29 = None
        self.motorSpeedPin31 = None


    def start_up(self):
        GPIO.setmode(GPIO.BCM)
        #Motor Init
        GPIO.setup(self.motorPinRPWM, GPIO.OUT)
        GPIO.setup(self.motorPinLPWM, GPIO.OUT)
        self.motorSpeedPin29 = GPIO.PWM(self.motorPinRPWM, 1000)
        self.motorSpeedPin31 = GPIO.PWM(self.motorPinLPWM, 1000)
  
        # pwm = PCA9685.PWM(bus_number=1)
        # pwm.setup()
        # pwm.frequency = 60
        # self.pwm = pwm
        # self.fw = Front_Wheels()
        # self.bw = Back_Wheels()


    def shutdown(self):
        pass
        # TODO shutdown pwm

    def forward(self, speed):
        print(f"Going forward at {speed}%")
        if(speed > 25 and speed < 100):
            speed = 25
            print(f"Speed override: Going forward at {speed}%")

        GPIO.output(self.motorPinLPWM, False)
        self.motorSpeedPin29.ChangeDutyCycle(speed)

    def backward(self, speed):
        print(f"Going backward at {speed}%")
        if (speed > 25 and speed < 100):
            speed = 25
            print(f"Speed override: Going backward at {speed}%")
        GPIO.output(self.motorPinRPWM, True)
        self.motorSpeedPin31.ChangeDutyCycle(speed)

    def stop(self):
        print("Motor stopped")
        GPIO.output(self.motorPinRPWM, False)
        GPIO.output(self.motorPinLPWM, False)

    def process_loop(self, *args):
        # if self.input_updated(('pilot/steering', )):
        #     steering_percent = self.context.get('pilot/steering', 0)
        #     steering = range_map(
        #         steering_percent, -1, 1, 70, 110, int_only=True)
        #     self.fw.turn(steering)
        # if self.input_updated(('user/steering', )):
        #     steering_percent = self.context.get('user/steering', 0)
        #     steering = range_map(
        #         steering_percent, -1, 1, 70, 110, int_only=True)
        #     self.fw.turn(steering)
        if self.input_updated(('user/throttle', )):
            throttle_percent = self.context.get('user/throttle', 0)
            throttle = range_map(
                abs(throttle_percent), 0, 1, 50, int_only=True)
            self.bw.speed = throttle
            if throttle_percent > 0:
                self.forward(throttle)
            elif throttle_percent < 0:
                self.backward(throttle)
            else:
                self.stop()
