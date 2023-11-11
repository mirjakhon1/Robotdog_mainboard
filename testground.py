import numpy as np
import time

class YourRobotClass:
    def __init__(self, step_size, execution_time):
        self.step_size = step_size
        self.execution_time = execution_time

        # Define the time step and the number of steps
        self.time_step = 0.01  # Adjust this value as needed for faster execution
        self.num_steps = int(self.execution_time / self.time_step)

        # Value for an arc, used for interpolation
        self.y_values_traj = np.array([-100., -77.77, -55.55, -33.33, -11.11, 11.11, 33.33, 55.55, 77.77, 100.])
        self.z_values_traj = np.array([0., 34.20, 64.27, 86.60, 98.48, 98.48, 86.60, 64.27, 34.20, 0.])

    def take_step(self):
        print("Taking a step")

        # Calculate step sizes for y and z
        y_step = self.step_size / self.num_steps
        z_step = -40 / self.num_steps  # Adjust this value as needed

        for _ in range(self.num_steps):
            y = self.y_values_traj[0]
            z = 0

            for i in range(len(self.y_values_traj)):
                if y >= self.step_size / 2:
                    break
                y = min(y + y_step, self.y_values_traj[i])
                z = self.z_values_traj[i]

            self.move_dir(y, z, self.constant.front_left_leg)

            # Sleep to control the speed of the movement
            time.sleep(self.time_step)

# Example usage:
robot = YourRobotClass(step_size=10, execution_time=2.0)  # Adjust step_size and execution_time as needed
robot.take_step()
