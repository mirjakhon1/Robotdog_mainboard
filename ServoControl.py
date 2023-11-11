from Constants import Constants
from adafruit_servokit import ServoKit  # https://circuitpython.readthedocs.io/projects/servokit/en/latest/
import time


class ServoControl:
    def __init__(self):
        self.constants = Constants()
        self.pca = ServoKit(channels=16)
        number_of_servos = 12
        min_imp = self.constants.servoMin
        max_imp = self.constants.servoMax
        for i in range(number_of_servos):
            self.pca.servo[i].set_pulse_width_range(min_imp, max_imp)
        self.MIN_ANG = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        self.MAX_ANG = [180, 180, 180, 180, 180, 180, 180, 180, 180, 180, 180, 180]

    def turn_to_angle(self, angle, servo_number):
        angle = round(angle)
        if angle < 0:
            print("Angle is a negative number, angle not changed")
        elif angle > 180:
            print("Servo angle out of range, angle not changed")
        else:
            self.pca.servo[servo_number].angle = angle
      
