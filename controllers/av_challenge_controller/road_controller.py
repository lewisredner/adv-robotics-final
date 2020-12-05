"""av_challenge_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, Camera, Motor, DistanceSensor
import numpy as np
import cv2 as cv
import math

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
print(timestep)
#print(dir(robot))

# You should insert a getDevice-like function in order to get the
#instance of a device of the robot. Something like:
 # motor = robot.getMotor('motorname')
 # ds = robot.getDistanceSensor('dsname')
 # ds.enable(timestep)


front_camera.enable(15)
rear_camera.enable(30)

def process_front(img1,shadow):

    

    img = np.zeros([img1.shape[0], img1.shape[1], img1.shape[2]-1],np.uint8)

    img[:,:,1] = img1[:,:,1]

    img_o = img
    #print(cv.mean(img))
    img = img[35*8:46*8,199:824 ]
    #print(cv.mean(img))
    dim = img.shape
    
    img = img[30:88,:]
    print(cv.mean(img))

    shadow = 0 
    
    imgray = cv.cvtColor(img,cv.COLOR_BGR2GRAY)
    #43 71 for green
    if(cv.mean(img)[1]>43 or cv.mean(img_o)[1]>71):
    #55 for green
        ret,thresh = cv.threshold(imgray,55,255,0)
    else:
        shadow =1
        imgray = imgray[:,200:426]
        #20 for green
        ret,thresh = cv.threshold(imgray,20,255,0)
        #ret2,thresh = cv.threshold(imgray,0,255,cv.THRESH_BINARY+cv.THRESH_OTSU)
       # thresh = cv.adaptiveThreshold(imgray,255,cv.ADAPTIVE_THRESH_GAUSSIAN_C,\
        #     cv.THRESH_BINARY,1401,5) 

             
 
    dthresh = thresh.shape  

    # cv.namedWindow("main", cv.WINDOW_NORMAL)
    # cv.imshow('main', thresh)
    # cv.waitKey()
    imgl = thresh[:,0:int(dthresh[1]/2)]
    imgr = thresh[:,int(dthresh[1]/2)+1:dthresh[1]]
    dr = imgl.shape
    ptot = dr[0]*dr[1]
    pxr = ptot-cv.countNonZero(imgr)
    pxl = ptot-cv.countNonZero(imgl)
    #print(pxl-pxr)
    return((pxr-pxl,shadow))
    


# Main loop:
# - perform simulation steps until Webots is stopping the controller
steer = 0
kp = 4.5
shadow_state = 0
while robot.step() != -1:
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()
    img_f = front_camera.getImage()
    img_cv = np.frombuffer(img_f, np.uint8).reshape((front_camera.getHeight(), front_camera.getWidth(), 4))
    #front_camera.saveImage("stop.png",100)

    nav_con = process_front(img_cv,shadow_state)

    weight = nav_con[0]
    shadow_state = nav_con[1]

    steer = kp*(weight/25000)
    if(abs(weight)>500):
        robot.setCruisingSpeed(35)
    else:
        robot.setCruisingSpeed(50)
    if(shadow_state==1):
        robot.setCruisingSpeed(25)

    print(weight)
    print(steer)


    robot.setSteeringAngle(steer)

    pass


    
    