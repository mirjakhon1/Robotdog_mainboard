
class Constants:
    servoMin = 944      # angle - 0
    servoMax = 2222     # angle - 180
    servoMiddle = 1500  # to adjust
    degree_1 = 1.46     # to adjust
    length_1 = 109      # leg length
    s = 58.5            # shoulder offset length, previously 41.5
    
    # leg indices
    front_left_leg = [0, 1, 2]
    front_right_leg = [3, 4, 5]
    back_left_leg = [6, 7, 8]
    back_right_leg = [9, 10, 11]

    # servo boundaries in degrees
    # servo0min, servo0max, servo1min, servo1max ...
    servoAngleLimitsLeg1 = [55, 128, 10, 170, 42, 120]
    servoAngleLimitsLeg2 = [37, 121, 10, 170, 64, 144]
    servoAngleLimitsLeg3 = [20, 118, 10, 170, 60, 138]
    servoAngleLimitsLeg4 = [50, 150, 10, 170, 40, 114]
