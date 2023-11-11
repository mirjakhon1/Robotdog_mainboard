import math
from Constants import Constants


class InverseKinematics:
    def __init__(self):
        self.constant = Constants()
        self.length_1 = self.constant.length_1

        self.angle_in_range = False

    def get_angles(self, y, z, leg_array):

        angle_tetha = 0
        phi = 0
        alpha = 0
        beta = 0
        # default position for z
        z = z + 155
        
        # diagonal leg error offset
        if leg_array == self.constant.front_left_leg or leg_array == self.constant.back_right_leg:
            z = z - 20

        l_hyp = math.sqrt(y * y + z * z)
        beta_rad = math.acos((2 * self.length_1 * self.length_1 - l_hyp * l_hyp) / (2 * self.length_1 * self.length_1))
        beta = beta_rad * 180 / math.pi

        if y > 0:
            phi = beta / 2
            angle_tetha = math.atan(z / y) * 180 / math.pi
            alpha = phi + angle_tetha
        elif y < 0:
            phi = beta / 2
            angle_tetha = math.atan(z / math.fabs(y)) * 180 / math.pi
            alpha = 180 + phi - angle_tetha
        elif y == 0:
            phi = (180 - (180 - beta)) / 2
            alpha = 90 + phi

        # mirroring angles for other 2 legs in diagonal
        if leg_array == self.constant.front_right_leg or leg_array == self.constant.back_left_leg:
            alpha = 180 - alpha
            beta = 180 - beta

        self._check_angle_limits(leg_array, alpha, beta)

        return alpha, beta, self.angle_in_range

    def _check_angle_limits(self, leg_array, alpha, beta):
        # assigning angle limits of specific leg to angle_limits variable
        angle_limits = []
        if leg_array[0] == 0:
            angle_limits = self.constant.servoAngleLimitsLeg1
        elif leg_array[0] == 3:
            angle_limits = self.constant.servoAngleLimitsLeg2
        elif leg_array[0] == 6:
            angle_limits = self.constant.servoAngleLimitsLeg3
        elif leg_array[0] == 9:
            angle_limits = self.constant.servoAngleLimitsLeg4

        # setting boundaries for alpha, beta, gamma
        if (not angle_limits[2] < alpha < angle_limits[3]) or \
                (not angle_limits[4] < beta < angle_limits[5]):
            if alpha < angle_limits[2]:
                alpha = angle_limits[2]
                print("Alpha angle boundary limit! (min angle reached)")
            elif alpha > angle_limits[3]:
                alpha = angle_limits[3]
                print("Alpha angle boundary limit! (max angle reached)")

            if beta < angle_limits[4]:
                beta = angle_limits[4]
                print("Beta angle boundary limit! (min angle reached)")
            elif beta > angle_limits[5]:
                beta = angle_limits[5]
                print("Beta angle boundary limit! (max angle reached)")

            self.angle_in_range = False

        else:
            self.angle_in_range = True

