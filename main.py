#!/usr/bin/env python
from MovementControl import MovementControl
import time
from Constants import Constants
from ServoControl import ServoControl

import RPi.GPIO as GPIO
from lib_nrf24 import NRF24
import spidev


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

# function main
def main():
    step = True
    default_position()
    
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
    
    step_size = 30
    data_list = []
    start_walking = False
    
    while True:
        
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
    
    while False:
        
        for x in range(-70, 70, 1):
            move_control._move_dir(0, x, -30, constants.back_left_leg)
            time.sleep(0.05)
            print(f"x === {x}")
        for x in range(70, -70, -1):
            move_control._move_dir(0, x, -30, constants.back_left_leg)
            time.sleep(0.05)
            print(f"x = {x}")
 

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        GPIO.cleanup()



