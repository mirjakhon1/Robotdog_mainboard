#!/usr/bin/env python
from MovementControl import MovementControl
import time
from Constants import Constants
from ServoControl import ServoControl

move_control = MovementControl()
constants = Constants()
servo_control = ServoControl()

def checkServoAngle():
    value = int(input("Enter a value: "))
    print(f"Received value from user input: {value}")
    servo_control.turn_to_angle(value, 5)
    #move_control._move_dir(value, 0, constants.front_right_leg)
    time.sleep(0.05)

# function main
def main():
    
    # 90 degree angle of shoulder servos
    servo_control.turn_to_angle(85, 0) # 85
    servo_control.turn_to_angle(90, 3)
    servo_control.turn_to_angle(85, 6)
    servo_control.turn_to_angle(80, 9)
    
    while False:
        servo_control.turn_to_angle(90, 1)
        servo_control.turn_to_angle(90, 2)
        servo_control.turn_to_angle(90, 4)
        servo_control.turn_to_angle(90, 5)
        servo_control.turn_to_angle(90, 7)
        servo_control.turn_to_angle(90, 8)
        servo_control.turn_to_angle(90, 10)
        servo_control.turn_to_angle(90, 11)
        
    while True:
        move_control.take_step(30)
        time.sleep(0.1)
        #move_control._move_dir(0, -30, constants.back_right_leg)
        #move_control._move_dir(0, -30, constants.back_left_leg)
        #move_control._move_dir(0, -30, constants.front_right_leg)
        #move_control._move_dir(0, -30, constants.front_left_leg)
        
    while False:
        
        for x in range(-65, 65, 10):
            move_control._move_dir(x, 0, constants.front_right_leg)
            time.sleep(0.2)
        for x in range(65, -65, -10):
            move_control._move_dir(x, 0, constants.front_right_leg)
            time.sleep(0.2)
        
        

if __name__ == '__main__':
    main()



