import warnings
import RPi.GPIO as GPIO
from time import sleep

GPIO.setwarnings(False)
# number is in mhz
# en_a = left
# en_b = right

class Motor:
    # Pin definitions

    def __init__(self, in1=13, in2=19, en_a=5, in3=26, in4=21, en_b=6, start_duty=75, frequency=1000):
        # Assign instance pins (avoid accidental duplicates)
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

        # Initialize PWM
        self.left_pwm = GPIO.PWM(self.en_a, frequency)
        self.right_pwm = GPIO.PWM(self.en_b, frequency)
        self.left_pwm.start(start_duty)
        self.right_pwm.start(start_duty)

    def _set_speeds(self, rfreq, lfreq):
        self.left_pwm.ChangeFrequency(lfreq)
        self.right_pwm.ChangeFrequency(rfreq)

    def move_forward(self, rfreq, lfreq):
        self._set_speeds(rfreq, lfreq)
        GPIO.output(self.in2, GPIO.HIGH)
        GPIO.output(self.in1, GPIO.LOW)
        GPIO.output(self.in3, GPIO.HIGH)
        GPIO.output(self.in4, GPIO.LOW)

        print("Forward")   
    def move_backward(self, rfreq, lfreq):
        self._set_speeds(rfreq, lfreq)
        GPIO.output(self.in1, GPIO.LOW)
        GPIO.output(self.in2, GPIO.HIGH)
        GPIO.output(self.in4, GPIO.LOW)
        GPIO.output(self.in3, GPIO.HIGH)
        print("Backward")

    def move_right(self, rfreq, lfreq):
        self._set_speeds(rfreq, lfreq)
        GPIO.output(self.in1, GPIO.LOW)
        GPIO.output(self.in2, GPIO.HIGH)
        GPIO.output(self.in4, GPIO.LOW)
        GPIO.output(self.in3, GPIO.LOW)
        print("Right")

    def move_left(self, rfreq, lfreq):
        self._set_speeds(rfreq, lfreq)
        GPIO.output(self.in1, GPIO.HIGH)
        GPIO.output(self.in2, GPIO.LOW)
        GPIO.output(self.in4, GPIO.HIGH)
        GPIO.output(self.in3, GPIO.HIGH)
        print("Left")

    def stop(self):
        self.left_pwm.stop()
        self.right_pwm.stop()
        print("Stopped")

    def cleanup(self):
        self.stop()
        self.left_pwm.stop()
        self.right_pwm.stop()
        GPIO.cleanup()



