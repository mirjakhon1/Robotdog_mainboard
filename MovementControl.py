import numpy as np
from Constants import Constants
from scipy.interpolate import interp1d
from InverseKinematics import InverseKinematics
from ServoControl import ServoControl
import time

def map_value(value, from_min, from_max, to_min, to_max):
        # Calculate the ratio of the value's position in the original range
        ratio = (value - from_min) / (from_max - from_min)

        # Map the ratio to the target range
        mapped_value = to_min + (to_max - to_min) * ratio

        # Make sure the result is within the target range
        return round(mapped_value, 2)
        
        
class MovementControl:
    def __init__(self):
        self.inverse_kin = InverseKinematics()
        self.constant = Constants()
        self.servo_control = ServoControl()

        # set of moving legs, used to switch legs each step
        self.moving_legs = [self.constant.front_left_leg, self.constant.back_right_leg]
        self.retracting_legs = [self.constant.front_right_leg, self.constant.back_left_leg]
        
    def take_step_2(self,  step_size_passed, direction, turn_angle):
        #print("Taking a step")

        # num_steps defines how fine the movement is along the arc
        num_steps = 15

        # data set for an arc, used in spline to create an interpolation
        y_values_traj = np.array([0, 10, 20, 30, 40, 50, 60, 70, 80, 90])
        z_values_traj = np.array([0., 15, 30, 46, 62, 77, 100, 67, 33, 0.])
        #spline = CubicSpline(y_values_traj, z_values_traj)
        linear_interp = interp1d(y_values_traj, z_values_traj, kind="linear")

        # creating set of evenly spaced y values (list) to feed it to spline which will return z values
        #y_values_mapped = np.linspace(y_values_traj[0], y_values_traj[-1], num_steps)
        #z_values_mapped = spline(y_values_mapped)
        y_values_mapped = np.linspace(0, 90, num_steps)
        z_values_mapped = linear_interp(y_values_mapped)

        # save moving leg set to switch it every step (every cycle)
        if self.moving_legs[0] == self.constant.front_left_leg:
            self.moving_legs = [self.constant.front_right_leg, self.constant.back_left_leg]
            self.retracting_legs = [self.constant.front_left_leg, self.constant.back_right_leg]
            
        else:
            self.moving_legs = [self.constant.front_left_leg, self.constant.back_right_leg]
            self.retracting_legs = [self.constant.front_right_leg, self.constant.back_left_leg]
            
        for y, z in zip(y_values_mapped, z_values_mapped):
            
            
            if (self.moving_legs[0] == self.constant.front_left_leg and direction == -1) or \
                (self.moving_legs[0] == self.constant.front_right_leg and direction == 1):
                step_size = step_size_passed / turn_angle
                step_height = 25 / turn_angle + 5
            else: 
                step_size = step_size_passed
                step_height = 25
            
            y_calc = map_value(y, y_values_mapped[0], y_values_mapped[-1], 0, step_size)
            # 100 is max of z value, negative 40 because z axis is inverted
            z_calc = map_value(z, 0, 100, -30, step_height) # netive z is downwards
            self._move_dir(0, y_calc, z_calc, self.moving_legs[0])
            
            if (self.moving_legs[1] == self.constant.back_left_leg and direction == -1) or \
                (self.moving_legs[1] == self.constant.back_right_leg and direction == 1):
                step_size = step_size_passed / turn_angle
                step_height = 25 / turn_angle + 5
            else: 
                step_size = step_size_passed
                step_height = 25
                
            # inverting y values for the other moving leg in diagonal
            y_calc = map_value(y, y_values_mapped[0], y_values_mapped[-1], 0, -step_size)
            z_calc = map_value(z, 0, 100, -30, step_height) # netive z is downwards
            self._move_dir(0, y_calc, z_calc, self.moving_legs[1])
            
            
            if (self.retracting_legs[0] == self.constant.front_left_leg and direction == -1) or \
                (self.retracting_legs[0] == self.constant.front_right_leg and direction == 1):
                step_size = step_size_passed / turn_angle
            else: 
                step_size = step_size_passed
            # retracting other 2 legs in diagonal
            y_calc = map_value(y, y_values_mapped[0], y_values_mapped[-1], step_size, 0)
            z_calc = map_value(z, z_values_mapped[0], 100, -30, -35)
            self._move_dir(0, y_calc, z_calc, self.retracting_legs[0]) 
            
            
            if (self.retracting_legs[1] == self.constant.back_left_leg and direction == -1) or \
                (self.retracting_legs[1] == self.constant.back_right_leg and direction == 1):
                step_size = step_size_passed / turn_angle
            else: 
                step_size = step_size_passed
                
            # inverting y values for the other retracting leg in diagonal
            y_calc = map_value(y, y_values_mapped[0], y_values_mapped[-1], -step_size, 0)
            self._move_dir(0, y_calc, z_calc, self.retracting_legs[1])
            #time.sleep(0.5)
    
    # not complete
    def turn_left(self, turn_step_size):
        print("Turning left...")
        
        num_steps = 10
        # data set for an arc, used in spline to create an interpolation
        x_values_traj = np.array([0, 10, 20, 30, 40, 50, 60, 70, 80, 90])
        z_values_traj = np.array([0., 34.20, 64.27, 86.60, 98.48, 98.48, 86.60, 64.27, 34.20, 0.])
        spline = CubicSpline(x_values_traj, z_values_traj)

        # creating set of evenly spaced y values (list) to feed it to spline which will return z values
        x_values_mapped = np.linspace(x_values_traj[0], x_values_traj[-1], num_steps)
        z_values_mapped = spline(x_values_mapped)
        
        # save moving leg set to switch it every step (every cycle)
        if self.moving_legs[0] == self.constant.front_left_leg:
            self.moving_legs = [self.constant.front_right_leg, self.constant.back_left_leg]
            self.retracting_legs = [self.constant.front_left_leg, self.constant.back_right_leg]
            
        else:
            self.moving_legs = [self.constant.front_left_leg, self.constant.back_right_leg]
            self.retracting_legs = [self.constant.front_right_leg, self.constant.back_left_leg]
        
        for x, z in zip(x_values_mapped, z_values_mapped):
            x_calc = map_value(x, x_values_mapped[0], x_values_mapped[-1], 0, turn_step_size)
            # 100 is max of z value, negative 40 because z axis is inverted
            z_calc = map_value(z, 0, 100, -30, 25) # negative z is downwards
            self._move_dir(x_calc, 0, z_calc, self.moving_legs[0])

            # moving other leg in diagonal
            x_calc = map_value(x, x_values_mapped[0], x_values_mapped[-1], 0, turn_step_size)
            self._move_dir(x_calc, 0, z_calc, self.moving_legs[1])
            
            # retracting other 2 legs in diagonal
            x_calc = map_value(x, x_values_mapped[0], x_values_mapped[-1], turn_step_size, 0)
            z_calc = map_value(z, z_values_mapped[0], 100, -30, -40)
            self._move_dir(x_calc, 0, z_calc, self.retracting_legs[0]) 

            # retracting other leg in diagonal
            x_calc = map_value(x, x_values_mapped[0], x_values_mapped[-1], turn_step_size, 0)
            self._move_dir(x_calc, 0, z_calc, self.retracting_legs[1])
        

    def do_pose_1(self):
        self._move_dir(0, 0, 30, self.constant.back_right_leg)
        self._move_dir(0, 0, 30, self.constant.back_left_leg)
        self._move_dir(0, 0, -50, self.constant.front_right_leg)
        self._move_dir(0, 0, -50, self.constant.front_left_leg)
        
    def _move_dir(self, x, y, z, leg_array):
        # calls inverse kinematics and move the leg
        alpha, beta, gamma, angle_limit_check = self.inverse_kin.get_angles(x, y, z, leg_array)
        #print(f"Alpha {alpha}")
        #print(f"Beta {beta}")
        #print(f"Gamma {gamma}")
        #print()



        if angle_limit_check:
            self.servo_control.turn_to_angle(gamma, leg_array[0])  # adjust for gamma
            self.servo_control.turn_to_angle(alpha, leg_array[1])
            self.servo_control.turn_to_angle(beta, leg_array[2])
        else:
            print("Angles are out of range!!! -> angle_limit_check = false. ")

    
    
