from controller import Robot
from controller import PositionSensor
from controller import Camera
from controller import DistanceSensor
from controller import LED 
from controller import Motor
import math
import numpy as np

WHEEL_SPEED = 47.6
TIME_STEP = 64
robot = Robot()

#init devices
camera = robot.getDevice("camera")
camera.enable(TIME_STEP)
leftWheel = robot.getDevice('left wheel motor')
rightWheel = robot.getDevice('right wheel motor')
encoderL = robot.getDevice('left wheel sensor')
encoderR = robot.getDevice('right wheel sensor')
front_l_led = robot.getDevice('front left led')
front_r_led = robot.getDevice('front right led')
rear_led = robot.getDevice('rear led')
INFINITY = math.inf

encoderL.enable(TIME_STEP)
encoderR.enable(TIME_STEP)

ultrasonic_sensor_names = [ "left ultrasonic sensor", "front left ultrasonic sensor", "front ultrasonic sensor", "front right ultrasonic sensor",
  "right ultrasonic sensor"]
infrared_sensors_names = ["rear left infrared sensor", "left infrared sensor", "front left infrared sensor", "front infrared sensor",
  "front right infrared sensor", "right infrared sensor", "rear right infrared sensor", "rear infrared sensor",
  "ground left infrared sensor", "ground front left infrared sensor", "ground front right infrared sensor",
  "ground right infrared sensor"]

def main():
  ultrasonic_devices = []
  infrared_devices = []

  robot.step(TIME_STEP)
  for x in ultrasonic_sensor_names:
    ultrasonic_devices += (robot.getDevice(ultrasonic_sensor_names[x]))
    ultrasonic_devices[x].enable() 
    
  for y in infrared_sensors_names:
    infrared_devices += (robot.getDevice(infrared_sensors_names[y]))
    infrared_devices[y].enable()

  leftWheel.setPosition(inf)
  rightWheel.setPosition(inf)
  leftWheel.setVelocity(0)
  rightWheel.setVelocity(0)

  last_display_second = 0

  while(robot.step(TIME_STEP)!= -1):
    display_second = robot.getTime()
    if(display_second != last_display_second):
      last_display_second = display_second

      print("time = ",display_second," [s] \n")
      for i in range (5):
        print("- ultrasonic sensors(",ultrasonic_sensor_names[i],") = ", ultrasonic_devices[i].getValue(),"[m]")
      for i in range (12):
        print("- infrared sensors(",infrared_sensor_names[i],") = ", infrared_devices[i].getValue(),"[m]")

    front_l_led.set(np.random.rand(0xFFFFFF))
    front_r_led.set(np.random.rand(0xFFFFFF))
    rear_led.set(np.random.rand(0xFFFFFF))

    #Obstacle avoidance algorithm

    speed_offset = 0.2 * (WHEEL_SPEED - 0.03 * infrared_sensors[3].getValue())
    speed_delta = 0.03 * infrared_sensors[2].getValue() - 0.03 * infrared_sensors[4].getValue()
    leftWheel.setVelocity(speed_offset + speed_delta)
    rightWheel.setVelocity(speed_offset - speed_delta)

main() 





