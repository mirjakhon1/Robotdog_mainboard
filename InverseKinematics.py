import math
from Constants import Constants


class InverseKinematics:
    def __init__(self):
        self.constant = Constants()
        self.length_1 = self.constant.length_1

        self.angle_in_range = False

    def get_angles(self, x, y, z, leg_array):
        
        s = self.constant.s
        angle_tetha = 0
        phi = 0
        alpha = 0
        beta = 0
        gamma = 0
        
        # default position for z
        z = z + 155
        
        # diagonal legs error offset
        if leg_array == self.constant.front_left_leg or leg_array == self.constant.back_right_leg:
            z = z - 20
        
        z_old = z
        ###############inverse kinematics######################
        if x<0:
            x = x - s
            g = math.sqrt(z*z + x*x)
            z = math.sqrt(g*g - s*s)            # z_new is set
            epsilon_1 = math.atan(z / math.fabs(x)) * 180 / math.pi
            epsilon_2 = math.acos((z*z + g*g - s*s) / (2*z*g)) * 180 / math.pi
            gamma = 90 - epsilon_1 - epsilon_2 
            gamma = 90 - gamma
        elif x>s:
            x = x - s
            g = math.sqrt(z*z + x*x)
            z = math.sqrt(g*g - s*s)            # z_new is set
            epsilon_1 = math.acos((s*s + g*g - z*z) / (2 * s * g)) * 180 / math.pi
            epsilon_2 = math.asin(x / g) * 180 / math.pi
            gamma = 90 - (epsilon_1 - epsilon_2)
            gamma = 90 + gamma
        elif 0<x<s:
            x = s - x
            g = math.sqrt(x*x + z*z)
            z = math.sqrt(g*g - s*s)            # z_new is set
            epsilon_1 = math.asin(x / g) * 180 / math.pi
            epsilon_2 = math.asin(z / g) * 180 / math.pi
            gamma = 90 - epsilon_1 - epsilon_2
            gamma = 90 + gamma 
        else:
            gamma = 90
            
       
        # invert z because of the orientation of z axis
        # if new z is increased by 10, this will decrease it by 10 instead
        if z > z_old:
            z = z + (z_old - z) * 2
        elif z < z_old:
            z = z + (z_old - z) * 2
            
        
        # gamma offset because 90 degrees is not a right angle
        if leg_array == self.constant.front_left_leg:
            gamma = gamma - 5           # 5 degree offset
        elif leg_array == self.constant.front_right_leg:
            gamma = gamma               # no offset
        elif leg_array == self.constant.back_left_leg:
            gamma = gamma + 5   # old value -5
        elif leg_array == self.constant.back_right_leg:
            gamma = gamma + 10  # old value -10
        
        # mirroring angles for shoulder angle 
        if leg_array == self.constant.back_right_leg or leg_array == self.constant.front_right_leg:
            gamma = 180 - gamma
         
         
        print(f"Z = {z}")
        #print(f"epsilon_1 = {epsilon_1}")
        #print(f"epsilon_2 = {epsilon_2}")
        #print()
        
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
            gamma = 180 - gamma
            

        self._check_angle_limits(leg_array, alpha, beta, gamma)

        return alpha, beta, gamma, self.angle_in_range

    def _check_angle_limits(self, leg_array, alpha, beta, gamma):
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
        if (not angle_limits[0] < gamma < angle_limits[1]) or \
            (not angle_limits[2] < alpha < angle_limits[3]) or (not angle_limits[4] < beta < angle_limits[5]):
                    
            if gamma < angle_limits[0]:
                gamma = angle_limits[0]
                print("Gamma angle boundary limit! (min angle reached)")
            elif gamma > angle_limits[1]:
                gamma = angle_limits[1]
                print("Gamma angle boundary limit! (max angle reached)")
                
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

