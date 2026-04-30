i;port warnings
import RPi.GPIO as GPIO
from time import sleep
GPIO.setwarnings(False)

class Motor:

    def __init__(self, in1=5, in2=6, en_a=24, in3=13, in4=19, en_b=24, start_duty=100, frequency=1000):

        self.in1 = in1  # Right motor input 1
        self.in2 = in2  # Right motor input 2
        self.en_a = en_a  # Right motor enable (PWM)

        self.in3 = in3  # Left motor input 1
        self.in4 = in4  # Left motor input 2
        self.en_b = en_b  # Left motor enable (PWM)

        GPIO.setmode(GPIO.BCM)

        for pin in (self.in1, self.in2, self.en_a, self.in3, self.in4, self.en_b):
            GPIO.setup(pin, GPIO.OUT)
            GPIO.output(pin, GPIO.LOW)

        self.left_pwm = GPIO.PWM(self.en_a, 100)
        self.right_pwm = GPIO.PWM(self.en_b, 100)
        self.left_pwm.start(start_duty)
        self.right_pwm.start(start_duty)

    def _set_speeds(self, rduty, lduty):
        self.left_pwm.ChangeDutyCycle(lduty)
        self.right_pwm.ChangeDutyCycle(rduty)

    def move_forward(self, rduty, lduty):
        self._set_speeds(rduty, lduty)
        GPIO.output(self.in1, GPIO.LOW)
        GPIO.output(self.in2, GPIO.HIGH)
        GPIO.output(self.in3, GPIO.HIGH)
        GPIO.output(self.in4, GPIO.LOW)

        print("Forward")
    def move_backward(self, rduty, lduty):
        self._set_speeds(rduty, lduty)
        GPIO.output(self.in1, GPIO.HIGH)
        GPIO.output(self.in2, GPIO.LOW)
        GPIO.output(self.in3, GPIO.LOW)
        GPIO.output(self.in4, GPIO.HIGH)
        print("Backward")

    def move_right(self, rduty, lduty):
        self._set_speeds(rduty, lduty)
        GPIO.output(self.in1, GPIO.LOW)
        GPIO.output(self.in2, GPIO.HIGH)
        GPIO.output(self.in3, GPIO.LOW)
        GPIO.output(self.in4, GPIO.LOW)
        print("Right")

    def move_left(self, rduty, lduty):
        self._set_speeds(rduty, lduty)
        GPIO.output(self.in1, GPIO.LOW)
        GPIO.output(self.in2, GPIO.LOW)
        GPIO.output(self.in3, GPIO.LOW)
        GPIO.output(self.in4, GPIO.HIGH)
        print("Left")

    def stop(self):
        GPIO.output(self.in1, GPIO.LOW)
        GPIO.output(self.in2, GPIO.LOW)
        GPIO.output(self.in4, GPIO.LOW)
        GPIO.output(self.in3, GPIO.LOW)
        print("Stopped")

    def cleanup(self):
        self.stop()
        self.left_pwm.stop()
        self.right_pwm.stop()
        GPIO.cleanup()
