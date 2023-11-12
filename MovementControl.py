import numpy as np
from Constants import Constants
from scipy.interpolate import CubicSpline
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

    def take_step(self,  step_size):
        print("Taking a step")

        # Define the time step and the number of steps // add time.sleep(time_step) to slow down movement
        execution_time = 1
        time_step = 0.1  # Adjust this value as needed for faster execution
        # num_steps defines how fine the movement is along the arc
        num_steps = int(execution_time / time_step)

        # data set for an arc, used in spline to create an interpolation
        y_values_traj = np.array([0, 10, 20, 30, 40, 50, 60, 70, 80, 90])
        z_values_traj = np.array([0., 34.20, 64.27, 86.60, 98.48, 98.48, 86.60, 64.27, 34.20, 0.])
        spline = CubicSpline(y_values_traj, z_values_traj)

        # creating set of evenly spaced y values (list) to feed it to spline which will return z values
        y_values_mapped = np.linspace(y_values_traj[0], y_values_traj[-1], num_steps)
        z_values_mapped = spline(y_values_mapped)

        # save moving leg set to switch it every step (every cycle)
        if self.moving_legs[0] == self.constant.front_left_leg:
            self.moving_legs = [self.constant.front_right_leg, self.constant.back_left_leg]
            self.retracting_legs = [self.constant.front_left_leg, self.constant.back_right_leg]
            
        else:
            self.moving_legs = [self.constant.front_left_leg, self.constant.back_right_leg]
            self.retracting_legs = [self.constant.front_right_leg, self.constant.back_left_leg]
            
        for y, z in zip(y_values_mapped, z_values_mapped):
            y_calc = map_value(y, y_values_mapped[0], y_values_mapped[-1], 0, step_size)
            # 100 is max of z value, negative 40 because z axis is inverted
            z_calc = map_value(z, 0, 100, -30, 25) # netive z is downwards
            self._move_dir(y_calc, z_calc, self.moving_legs[0])

            # inverting y values for the other moving leg in diagonal
            y_calc = map_value(y, y_values_mapped[0], y_values_mapped[-1], 0, -step_size)

            self._move_dir(y_calc, z_calc, self.moving_legs[1])
            
            # retracting other 2 legs in diagonal
            y_calc = map_value(y, y_values_mapped[0], y_values_mapped[-1], step_size, 0)
            z_calc = map_value(z, z_values_mapped[0], 100, -30, -40)
            self._move_dir(y_calc, z_calc, self.retracting_legs[0]) 

            # inverting y values for the other retracting leg in diagonal
            y_calc = map_value(y, y_values_mapped[0], y_values_mapped[-1], -step_size, 0)
            self._move_dir(y_calc, z_calc, self.retracting_legs[1])

    def _move_dir(self, y, z, leg_array):
        # calls inverse kinematics and move the leg
        alpha, beta, angle_limit_check = self.inverse_kin.get_angles(y, z, leg_array)
        #print(f"Alpha {alpha}")
        #print(f"Beta {beta}")

        if angle_limit_check:
            #self.servo_control.turn_to_angle(90, leg_array[0])  # adjust for gamma
            self.servo_control.turn_to_angle(alpha, leg_array[1])
            self.servo_control.turn_to_angle(beta, leg_array[2])
        else:
            print("Angles are out of range!!! -> angle_limit_check = false. ")

    
    
