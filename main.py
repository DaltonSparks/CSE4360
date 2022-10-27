#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
import math as mat
import math

# Create your objects here.
ev3 = EV3Brick()

#connected motors to ports
LM = Motor(Port.A)
RM = Motor(Port.B)
#G = GyroSensor(Port.s1)
#speed variables
# 16x10 12 inch tiles
#move speed and time 
MS = ((305/(85*math.pi))*360)/2
MT = 1480*2
#turn speed and time
TS = 150 #((305/(85*math.pi))*360)/2
print(TS)
TT = 1165 #890

def left():
    time.sleep(2)
    LM.run_time(-TS,TT,Stop.HOLD,False)
    RM.run_time(TS,TT,Stop.HOLD,True)

def right():
    time.sleep(2)
    LM.run_time(TS,TT,Stop.HOLD,False)
    RM.run_time(-TS,TT,Stop.HOLD,True)

def forward():
    time.sleep(2)
    LM.run_time(MS,MT,Stop.HOLD,False)
    RM.run_time(MS,MT,Stop.HOLD,True)

def hforward():
    time.sleep(2)
    LM.run_time(MS,MT/2,Stop.HOLD,False)
    RM.run_time(MS,MT/2,Stop.HOLD,True)

def backward():
    time.sleep(2)
    LM.run_time(-MS,MT,Stop.HOLD,False)
    RM.run_time(-MS,MT,Stop.HOLD,True)

def beauty_print(arr):
    for row in arr:
        print(*row, sep="\t")

# implementing path finding algorithm for the robot
num_obstacles = 13
obstacles = [
(0.61, 2.743),(0.915, 2.743),(1.219, 2.743),(1.829, 1.219),
(1.829, 1.524),( 1.829, 1.829), (1.829, 2.134),(2.743, 0.305),
(2.743, 0.61),(2.743, 0.915),(2.743, 2.743),(3.048, 2.743),
(3.353, 2.743),
(-1,-1),(-1,-1),(-1,-1),(-1,-1),(-1,-1),(-1,-1),(-1,-1),(-1,-1),(-1,-1),
(-1,-1),(-1,-1),(-1,-1),
]
start = (0.305, 1.219)
goal = (3.658, 1.829)

block_length = 0.305
max_block_len_ind = 16
max_block_wid_ind = 10
total_blocks = max_block_len_ind * max_block_wid_ind
init_potential = total_blocks + 5
obstacle_potential = 500
config_space = [[init_potential for i in range(max_block_len_ind)] for j in range(max_block_wid_ind)]
for i in range(num_obstacles):
    len_obs, wid_obs = obstacles[i]
    len_block, wid_block = round(len_obs/block_length), round(wid_obs/block_length)
    for w in range(2):
        for l in range(2):
            new_w = max_block_wid_ind - (wid_block - w) - 1
            new_l = len_block - l
            if new_l >= 0 and new_l < max_block_len_ind and new_w >= 0 and new_w < max_block_wid_ind:
                config_space[new_w][new_l] = obstacle_potential

start_block = round(start[0]/block_length), round(start[1]/block_length)
goal_block = round(goal[0]/block_length), round(goal[1]/block_length)


def path_finder(obstacles, start, goal):
    goal_obj = (max_block_wid_ind - goal[1] - 2, goal[0] - 1, 0) # (width_index, length_index, 0)
    q = []
    q.append(goal_obj)
    visited_nodes = set()
    while q:
        obj = q.pop(0)
        config_space[obj[0]][obj[1]] = obj[2]
        visited_nodes.add((obj[0], obj[1]))
        for w in range(-1, 2, 1):
            for l in range(-1, 2, 1):
                new_w = obj[0] + w
                new_l = obj[1] + l
                new_p = obj[2] + 1
                if abs(w) == abs(l):
                    continue
                if new_w >= 0 and new_l >= 0 and new_w < max_block_wid_ind and new_l < max_block_len_ind:
                    if (new_w, new_l) not in visited_nodes and config_space[new_w][new_l] != obstacle_potential and config_space[new_w][new_l] > new_p:
                        q.append((new_w, new_l, new_p))
    return 

path_finder(obstacles, start_block, goal_block)

beauty_print(config_space)

def instructions():
    final = 1
    i = 0
    j = 0
    direction = "SOUTH"
    while final:
        if(config_space[i][j]>config_space[i+1][j]):
            if(direction == "SOUTH"):
                print("Move Forward")
                forward()
            elif(direction == "EAST"):
                print("Turn right move forward")
                right()
                forward()
                direction = "SOUTH"
            elif(direction == "WEST"):
                print("Turn left move forward")
                left()
                forward()
                direction = "SOUTH"
            elif(direction == "NORTH"):
                print("Turn right twice move forward")
                right()
                right()
                forward()
                direction = "SOUTH"
            print("moving to:")
            print(config_space[i+1][j])
            final = config_space[i+1][j]
            i += 1
        elif(config_space[i][j]>config_space[i-1][j]):
            if(direction == "SOUTH"):
                print("Turn right twice move forward")
                right()
                right()
                forward()
                direction = "NORTH"
            elif(direction == "EAST"):
                print("Turn right move forward")
                right()
                forward()
                direction = "NORTH"
            elif(direction == "WEST"):
                print("Turn left move forward")
                left()
                forward()
                direction = "NORTH"
            elif(direction == "NORTH"):
                forward()
                print("Move Forward")
            print("moving to:")
            print(config_space[i-1][j])
            final = config_space[i-1][j]
            i -= 1
        elif(config_space[i][j]>config_space[i][j+1]):
            if(direction == "SOUTH"):
                print("Turn left move forward")
                left()
                forward()
                direction = "EAST"
            elif(direction == "EAST"):
                print("Move Forward")
                forward()
            elif(direction == "WEST"):
                print("Turn right twice move forward")
                right()
                right()
                forward()
                direction = "EAST"
            elif(direction == "NORTH"):
                print("Turn right move forward")
                right()
                forward()
                direction = "EAST"
            print("moving to:")
            print(config_space[i][j+1])
            final = config_space[i][j+1]
            j += 1
        elif(config_space[i][j]>config_space[i+1][j-1]):
            if(direction == "SOUTH"):
                print("Turn right move forward")
                right()
                forward()
                direction = "WEST"
            elif(direction == "EAST"):
                print("Turn right twice move forward")
                right()
                right()
                forward()
                direction = "WEST"
            elif(direction == "WEST"):
                print("Move Forward")
                forward()
            elif(direction == "NORTH"):
                print("Turn left move forward")
                left()
                forward()
                direction = "WEST"
            print("moving to:")
            print(config_space[i+1][j-1])
            final = config_space[i][j-1]
            j -= 1



instructions()


#construct workspace with all paths and obstacles 

#implement a function to calculate path to take and movements


# Write your program here.
#ev3.speaker.beep()
