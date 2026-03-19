import warnings
import RPi.GPIO as GPIO
from time import sleep

GPIO.setwarnings(False)
# number is in mhz
# en_a = left
# en_b = right
class Motor:
    # Pin definitions
    in1 = 17  # Right Motor
    in2 = 27
    en_a = 4
    in3 = 5   # Left Motor
    in4 = 6
    en_b = 13

    def __init__(self, start_duty=75, frequency=500):
        GPIO.setmode(GPIO.BCM)

        # Setup all pins as output
        for pin in [self.in1, self.in2, self.en_a, self.in3, self.in4, self.en_b]:
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
        GPIO.output(self.in1, GPIO.HIGH)
        GPIO.output(self.in2, GPIO.LOW)
        GPIO.output(self.in4, GPIO.HIGH)
        GPIO.output(self.in3, GPIO.LOW)
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
        self.left_pwm.ChangeFrequency(0)
        self.right_pwm.ChangeFrequency(0)
        print("Stopped")

    def cleanup(self):
        self.stop()
        self.left_pwm.stop()
        self.right_pwm.stop()
        GPIO.cleanup()


# Wrap main content in a try block so we can  catch the user pressing CTRL-C and run the
# GPIO cleanup function. This will also prevent the user seeing lots of unnecessary error messages.
try:
# Create Infinite loop to read user input
#
    movement = ""
    freq_right = 0
    freq_left = 0
    motor = Motor()
    while(True):
        # Get user Input
        user_input = input()
        try:
            movement = user_input.split()[0]
            freq_right = int(user_input.split()[1])
            freq_left = int(user_input.split()[2])
        except Exception as e:
            continue

      # To see users input
      # print(user_input)

        if movement == 'w':
            motor.move_forward(freq_right, freq_left)
        elif movement == 's':
            motor.move_backward(freq_right, freq_left)
        elif movement == 'd':
            motor.move_right(freq_right, freq_left)
        elif movement == 'a':
            motor.move_left(freq_right, freq_left)
        elif movement == 'c':
            motor.stop()

# If user press CTRL-C
except KeyboardInterrupt:
  # Reset GPIO settings
  GPIO.cleanup()
  print("GPIO Clean up")

