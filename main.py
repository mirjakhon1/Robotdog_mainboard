#!/usr/bin/env python
from MovementControl import MovementControl
import time
from Constants import Constants
from ServoControl import ServoControl

import RPi.GPIO as GPIO
from lib_nrf24 import NRF24
import spidev
import threading

# SPI communication
GPIO.setmode(GPIO.BCM)

pipes = [[0xE8, 0xE8, 0xF0, 0xF0, 0xE1], [0xF0, 0xF0, 0xF0, 0xF0, 0xE1]]

radio = NRF24(GPIO, spidev.SpiDev())
radio.begin(1, 17)

radio.setPayloadSize(32)
radio.setChannel(0x30)
radio.setDataRate(NRF24.BR_1MBPS)
radio.setPALevel(NRF24.PA_MIN)

radio.setAutoAck(True)
radio.enableDynamicPayloads()
radio.enableAckPayload()

radio.openReadingPipe(1, pipes[1])
radio.printDetails()
radio.startListening()


move_control = MovementControl()
constants = Constants()
servo_control = ServoControl()

def map_value(value, from_min, from_max, to_min, to_max):
        # Calculate the ratio of the value's position in the original range
        ratio = (value - from_min) / (from_max - from_min)

        # Map the ratio to the target range
        mapped_value = to_min + (to_max - to_min) * ratio

        # Make sure the result is within the target range
        return round(mapped_value, 2)
        
def checkServoAngle():
    value = int(input("Enter a value: "))
    print(f"Received value from user input: {value}")
    servo_control.turn_to_angle(value, 5)
    #move_control._move_dir(value, 0, constants.front_right_leg)
    time.sleep(0.05)

def default_position():
    # default position
    move_control._move_dir(0, 0, 0, constants.back_right_leg)
    move_control._move_dir(0, 0, 0, constants.back_left_leg)
    move_control._move_dir(0, 0, 0, constants.front_right_leg)
    move_control._move_dir(0, 0, 0, constants.front_left_leg)


data_list = []
start_walking = False
do_pose = False
received_data = ""
step_size = 24

# speed variables
default_speed = 0.2
current_speed = default_speed
max_speed = 0.051
min_speed = 0.69
    
default_turn_speed = 2.5
current_turn_speed = default_turn_speed
max_turn_speed = 4.49
min_turn_speed = 1.51
turn_direction = 0              # -1 left, 0 forward, 1 right

# function main
def main():
    default_position()

    
    def radio_listener():
        global received_data, start_walking, data_list, step_size, do_pose
        global current_speed, max_speed, min_speed, current_turn_speed, max_turn_speed, min_turn_speed
        bounce_time = 1.0       # button press bounce time
        old_time = time.time()
        
        while True:
            if radio.available(0):
                receivedMessage = []
                radio.read(receivedMessage, radio.getDynamicPayloadSize())
                #print("Recevied: {}".format(receivedMessage))
                for n in receivedMessage:
                    if (n >= 32 and n <= 126):
                        received_data += chr(n)
                #print("Our received message decodes to: {}".format(received_data))
                #print()
                
                # direction, turn_direction, speed_change, turn_speed_change
                data_list = received_data.split(",")
                received_data = ""
                
                turn_direction = int(data_list[1])

                # walk backwards?
                if int(data_list[0]) == -1:
                    step_size = -abs(step_size)
                else:
                    step_size = abs(step_size)
                
                if (time.time() > old_time + bounce_time):
                    # adjust the speed and turn speed
                    if int(data_list[2]) == 0 and current_speed > max_speed:
                        current_speed -= 0.05
                    elif int(data_list[3]) == 0 and current_speed < min_speed:
                        current_speed += 0.05
                    elif int(data_list[4]) == 0 and current_turn_speed < max_turn_speed:
                        current_turn_speed += 0.5
                    elif int(data_list[5]) == 0 and current_turn_speed > min_turn_speed:
                        current_turn_speed -= 0.5
                        # posing condition
                    elif int(data_list[6]) == 0:
                        do_pose = True
                    old_time = time.time()
                    
            if data_list and radio.available(0):
                if int(data_list[0]) != 0:
                    start_walking = True
                else:
                    start_walking = False
            else:
                start_walking = False
            
            time.sleep(0.1)
    
    
    def control_robot():
        
        global start_walking, step_size, turn_direction, current_turn_speed, current_speed, do_pose
        
        
        while True:
            if start_walking:
                print("walking")
                print(f"step_size: {step_size}")
                print(f"turn_direction: {turn_direction}")
                print(f"current_speed: {current_speed}")
                print(f"Turn speed: {current_turn_speed}")
                print()
                move_control.take_step_2(step_size, turn_direction, current_turn_speed)
                time.sleep(current_speed)
        
            else:
                default_position()
            time.sleep(0.1)
            
            # posing
            if not start_walking and do_pose:
                move_control.do_pose_1()
                time.sleep(2)
                do_pose = False
    
    
    radio_thread = threading.Thread(target=radio_listener)
    radio_thread.start()
    
    task_thread = threading.Thread(target=control_robot)
    task_thread.start()
    
        
    
    while False:
        
        if radio.available(0):	
            	
            receivedMessage = []
            radio.read(receivedMessage, radio.getDynamicPayloadSize())
            #print("Recevied: {}".format(receivedMessage))
		
            # Translating our received Message into unicode characters
            received_data = ""
		
            for n in receivedMessage:
                if (n >= 32 and n <= 126):
                    received_data += chr(n)
            print("Our received message decodes to: {}".format(received_data))
        
            # direction, turn_direction, speed_change, turn_speed_change
            data_list = received_data.split(",")
        
        if data_list and radio.available(0):
            if int(data_list[0]) != 0:
                start_walking = True
            else:
                start_walking = False
        else:
            start_walking = False
        
        if start_walking:
            turn_direction = int(data_list[1])
            
            # walk backwards?
            if int(data_list[0]) == -1:
                step_size = -abs(step_size)
            else:
                step_size = abs(step_size)
            
            # adjust the speed and turn speed
            if int(data_list[2]) == 0 and current_speed > max_speed:
                current_speed -= 0.05
            elif int(data_list[3]) == 0 and current_speed < min_speed:
                current_speed += 0.05
            elif int(data_list[4]) == 0 and current_turn_speed < max_turn_speed:
                current_turn_speed += 0.5
            elif int(data_list[5]) == 0 and current_turn_speed > min_turn_speed:
                current_turn_speed -= 0.5
            
            move_control.take_step_2(step_size, turn_direction, current_turn_speed)
            time.sleep(current_speed)
            
        else:
            default_position()
        

        
    while False:
        servo_control.turn_to_angle(90, 1)
        servo_control.turn_to_angle(90, 2)
        servo_control.turn_to_angle(90, 4)
        servo_control.turn_to_angle(90, 5)
        servo_control.turn_to_angle(90, 7)
        servo_control.turn_to_angle(90, 8)
        servo_control.turn_to_angle(90, 10)
        servo_control.turn_to_angle(90, 11)
    






if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        GPIO.cleanup()



