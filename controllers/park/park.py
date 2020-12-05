# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, Camera, Motor, DistanceSensor
import numpy as np
import cv2 as cv
import math
import reeds_shepp

from vehicle import Driver
 
# create the Robot instance.
#robot = Robot()
robot = Driver()
front_camera = robot.getCamera("front_camera")
rear_camera = robot.getCamera("rear_camera")
lidar = robot.getLidar("Sick LMS 291")


timestep = int(robot.getBasicTimeStep())/1e3
 
carleng = 3.37
 
dt = timestep  # [s] time difference
L = 2.87528  # [m] Wheel base of vehicle

lidar.enable(30)
        
steer = 0
distSinceObs = 0
bigGap = 0
accumDist = 0
obstacles = []
obstacles.append(0)
spotFound = False
mode = 'Looking'
spotWidth = 2.5
curSpeed = 0
show_animation = True


class State(object):
    """
    Class representing the state of a vehicle.
    :param x: (float) x-coordinate
    :param y: (float) y-coordinate
    :param yaw: (float) yaw angle
    :param v: (float) speed
    """

    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        """Instantiate the object."""
        super(State, self).__init__()
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v

    def update(self, curV, delta):
        """
        Update the state of the vehicle.
        Stanley Control uses bicycle model.
        :param acceleration: (float) Acceleration
        :param delta: (float) Steering
        """
        # delta = np.clip(delta, -max_steer, max_steer)
        self.v = curV

        self.x += self.v * np.cos(self.yaw) * dt
        self.y += self.v * np.sin(self.yaw) * dt
        self.yaw += self.v / L * np.tan(-delta) * dt

while robot.step() != -1:

    print('Current speed: {}' .format(curSpeed))
    rangeData = lidar.getRangeImage()
    toTheRight = rangeData[-1]
    if 25*.99 < (robot.getCurrentSpeed()) < 25*1.0 and mode == 'Looking':
        if toTheRight < 10:
            distSinceObs = 0
            obstacles.append(1)
        elif toTheRight < 70 and obstacles[-1] == 1:
            distSinceObs = 0
            obstacles.append(1)
        else:
            distSinceObs += (curSpeed)*timestep
            if distSinceObs > bigGap:
                bigGap = distSinceObs
            obstacles.append(0)
    elif not spotFound:
        obstacles = []
        obstacles.append(1)
    
    if bigGap > 5.8 and obstacles[-1] == 1:
        if (mode == 'Looking' or mode == 'Stopping') and curSpeed > 0:
            accumDist += curSpeed*timestep
            robot.setCruisingSpeed(0)
            robot.setBrakeIntensity(1)
            mode = 'Stopping'
        elif curSpeed == 0 and mode == 'Stopping':
            print(accumDist)
            print(toTheRight)
            mode = 'Parking'
            errors = 0.25
            state = State(x=3*bigGap/4 + accumDist - 3.37, y=toTheRight + spotWidth/1.75, yaw=0, v=0.0)
            if abs(state.x-state.y)/state.x < 0.1:
                constraint = np.min([state.x,state.y])/2
                constInd = np.argmin([state.x,state.y])
                turnAng = np.arctan(L/constraint)
            else:
                mode = 'Fixing'
    else:
        print(obstacles)
        print(distSinceObs)
        robot.setCruisingSpeed(25)
        robot.setSteeringAngle(steer*np.pi/180)
        
    if mode == 'Fixing':
        print(state.x,state.y)
        if state.x < state.y:
            di = 0
            target_speed = 2
            robot.setCruisingSpeed(target_speed)
            robot.setSteeringAngle(di)
            state.update(curSpeed,di)
        else:
            constraint = np.min([state.x,state.y])/2
            constInd = np.argmin([state.x,state.y])
            turnAng = np.arctan(L/constraint)
            mode = 'Parking'
        
    if mode == 'Parking' or mode == 'Finishing':
        middleAvg = np.mean(rangeData[87:94])
        if state.x > 0.15 and mode == 'Parking' and middleAvg > bigGap - carleng*1.01:
            target_speed = 2
            xandy = [state.x,state.y]
            if xandy[constInd] <= 1*constraint:
                di = -turnAng
            else:          
                di = turnAng
                
            state.update(curSpeed,di)
            print('Current X: {}' .format(state.x))
            print('Current Y: {}' .format(state.y))
            print('Current Angle: {}' .format(state.yaw))
            robot.setCruisingSpeed(-1*target_speed)
            robot.setSteeringAngle(di)
            
        elif middleAvg > (bigGap - carleng)/2.75:
            if mode == state.yaw > 0:
                di = -np.pi/5.5
            else:
                di = np.pi/5.5
            target_speed = 2
            state.update(curSpeed,di)
            print('Current X: {}' .format(state.x))
            print('Current Y: {}' .format(state.y))
            print('Current Angle: {}' .format(state.yaw))
            robot.setCruisingSpeed(target_speed)
            robot.setSteeringAngle(di)
            mode = 'Finishing'
            
        elif np.abs(state.v) > 0:
            di = 0
            target_speed = 0
            robot.setBrakeIntensity(1)
            state.update(curSpeed,di)
            print('Current X: {}' .format(state.x))
            print('Current Y: {}' .format(state.y))
            print('Current Angle: {}' .format(state.yaw))
            robot.setCruisingSpeed(target_speed)
            robot.setSteeringAngle(di)
        else:
            mode = 'Finished'
            
    
    curSpeed = robot.getCurrentSpeed()*1000/3600
    if curSpeed != curSpeed:
        curSpeed = 0
    print(mode)
       
    pass