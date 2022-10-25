#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
import math as math
import time
#define MAX_OBSTACLES 25 /* maximum number of obstacles */
num_obstacles = 13 #number of obstacles
obstaclegrid = [
(0.61, 2.743),(0.915, 2.743),(1.219, 2.743),(1.829, 1.219),
(1.829, 1.524),( 1.829, 1.829), (1.829, 2.134),(2.743, 0.305),
(2.743, 0.61),(2.743, 0.915),(2.743, 2.743),(3.048, 2.743),
(3.353, 2.743),
(-1,-1),(-1,-1),(-1,-1),(-1,-1),(-1,-1),(-1,-1),(-1,-1),(-1,-1),(-1,-1),
(-1,-1),(-1,-1),(-1,-1),
]
start = (0.305, 1.219)
goal = (3.658, 1.829)


# Create your objects here.
ev3 = EV3Brick()

#connected motors to ports
LM = Motor(Port.A)
RM = Motor(Port.B)
G = GyroSensor(Port.s1)
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
    LM.run_time(-TS,TT,Stop.HOLD,False)
    RM.run_time(TS,TT,Stop.HOLD,True)

def right():
    LM.run_time(TS,TT,Stop.HOLD,False)
    RM.run_time(-TS,TT,Stop.HOLD,True)

def forward():
    LM.run_time(MS,MT,Stop.HOLD,False)
    RM.run_time(MS,MT,Stop.HOLD,True)

def backward():
    LM.run_time(-MS,MT,Stop.HOLD,False)
    RM.run_time(-MS,MT,Stop.HOLD,True)
c = 0
while(c<5):

#testing movement
    time.sleep(1)
    left()
#left()
#time.sleep(1)
#right()
#right()
    time.sleep(1)
    forward()
#backward()
    time.sleep(1)
    left()
    time.sleep(1)
    forward()
    time.sleep(1)
    left()
    time.sleep(1)
    forward()
    time.sleep(1)
    left()
    time.sleep(1)
    forward()
    c+=1


#construct workspace with all paths and obstacles 

#implement a function to calculate path to take and movements


# Write your program here.
#ev3.speaker.beep()
