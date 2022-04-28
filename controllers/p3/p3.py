 
from controller import Robot
from controller import PositionSensor
from controller import Camera

import heapq
import numpy as np
robot = Robot()

WHEEL_SPEED = 3.0
TIME_STEP = 64

#init devices
camera = robot.getDevice("camera")
camera.enable(TIME_STEP)
leftWheel = robot.getDevice('left wheel motor')
rightWheel = robot.getDevice('right wheel motor') 
encoderL = robot.getDevice("left wheel sensor")
encoderR = robot.getDevice("right wheel sensor")
encoderL.enable(TIME_STEP)
encoderR.enable(TIME_STEP)
posL = 0.0
posR = 0.0