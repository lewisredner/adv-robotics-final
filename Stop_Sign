"""av_challenge_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, Camera, Motor, DistanceSensor
import numpy as np
import cv2 as cv
import math
import time
from PIL import Image

from vehicle import Driver
 
# create the Robot instance.
#robot = Robot()
robot = Driver()
front_camera = robot.getCamera("front_camera")
rear_camera = robot.getCamera("rear_camera")
lidar = robot.getLidar("Sick LMS 291")

#for att in dir(robot):
#    print(att,getattr(robot,att))
# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())
#print(timestep)
#print(dir(robot))

# You should insert a getDevice-like function in order to get the
#instance of a device of the robot. Something like:
 # motor = robot.getMotor('motorname')
 # ds = robot.getDistanceSensor('dsname')
 # ds.enable(timestep)


front_camera.enable(30)
rear_camera.enable(30)

def find_stop(img):
    # crop image to just the left part of the image
    img_cropped = img[10:30,0:20]
    # extract just the red components
    img_red = img_cropped[:,:,2]
    #cv.imwrite('cropped_view.png',img_red)
    # get number of red pixels
    num_red_pixels = sum(img_red[img_red>140])
    num_total_pixels = img_red.shape[0]*img_red.shape[1]
    red_percentage = num_red_pixels/num_total_pixels
    # test out the red percentage required empirically
    if red_percentage > 1.5:
        return 1
    return 0
    
# Main loop:
# set up the initial velocity and direction
steer = 0
robot.setSteeringAngle(steer)
robot.setCruisingSpeed(25)

# enter the while loop
while robot.step() != -1:
    # capture an image from the camera
    img_f = front_camera.getImage()
    img_cv = np.frombuffer(img_f, np.uint8).reshape((front_camera.getHeight(), front_camera.getWidth(), 4))
    # see whether we need to stop
    stop = find_stop(img_cv)
    # if stop was 1, then we have to stop soon
    if(stop):
        print('stopped')
        # set cruising speed to 0 to stop
        robot.setCruisingSpeed(0)
        # iterate for a while to stay stopped
        for i in range(100):
            robot.step()
        # reset the cruise speed
        robot.setCruisingSpeed(25)
     


    

    pass
