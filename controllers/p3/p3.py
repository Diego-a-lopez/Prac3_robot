 
from controller import Robot
from controller import PositionSensor
from controller import Camera

import heapq
import numpy as np
robot = Robot()

WHEEL_SPEED = 3.0
TIME_STEP = 64

#init devices
##camera = robot.getDevice("camera")
##camera.enable(TIME_STEP)
leftWheel = robot.getDevice('left wheel motor')
rightWheel = robot.getDevice('right wheel motor') 
encoderL = robot.getDevice("left wheel sensor")
encoderR = robot.getDevice("right wheel sensor")
encoderL.enable(TIME_STEP)
encoderR.enable(TIME_STEP)

#ground sensors

frontLeftSensor = robot.getDevice('ground front left infrared sensor')
frontRigthSensor = robot.getDevice('ground front right infrared sensor')
leftSensor = robot.getDevice('ground left infrared sensor')
rightSensor = robot.getDevice('ground right infrared sensor')

frontLeftSensor.enable(TIME_STEP)
frontRigthSensor.enable(TIME_STEP)
leftSensor.enable(TIME_STEP)
rigthSensor.enable(TIME_STEP)

posL = 0.0
posR = 0.0


#actions
FORWARD = 0
LEFT = 1
RIGHT = 2


states = ['Default','leftLeft', 'leftRight']

DEFAULT = 0
FROMLEFT = 1
FROMRIGHT = 2

size = (3,3) #size of q matrix

Q = np.zeros(size, dtype=float ) #initialize empty matrix

def advanceForward():
    pass
def turnLeft():
    pass
def turnRight():
    pass
def getState():
    if ((frontRigthSensor.getValue() > 750 ) and (leftSensor.getValue() < 500)):
        return 2
    elif((frontLeftSensor.getValue() > 750) and (rightSensor.getValue() < 500)):
        return 1
    else:
        return 0
def reward():
    reward = 0
    if (rigthSensor.getValue() > 750 ):
        reward += 1
    elif (frontRigthSensor.getValue() < 500 ):
        reward -= 1
    if (frontRigthSensor.getValue() > 750 ):
        reward += 1
    elif (frontRigthSensor.getValue() < 500 ):
        reward -= 1    
        
    return reward
def getNextAction():
    return 0;
def main():
    if (getNextAction== FOWARD)
    