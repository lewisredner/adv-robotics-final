"""av_challenge_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, Camera, Motor, DistanceSensor
import numpy as np
import cv2 as cv
import math
import matplotlib.pyplot as plt

from vehicle import Driver
 
# create the Robot instance.
#robot = Robot()
robot = Driver()
front_camera = robot.getCamera("front_camera")
rear_camera = robot.getCamera("rear_camera")
lidar = robot.getLidar("Sick LMS 291")

MAKEPLOT = False # TODO next need to add plotting for things other than speed

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


front_camera.enable(30)
rear_camera.enable(30)

'''
def make_plot():
    y = np.random.random()
    plt.scatter(4, y)
    plt.pause(0.05)
    print("iterate")
    plt.show()
'''
plt.style.use('ggplot')

def live_plotter(x_vec, y1_data, change, title):
    pause_time=0.1
    if change == []:
        plt.ion()
        fig = plt.figure(figsize=(13,6))
        ax = fig.add_subplot(111)
        change, = ax.plot(x_vec, y1_data,'-o',alpha=0.8)        
        plt.ylabel('Y Label')
        plt.title(title)
        plt.show()
    
    change.set_ydata(y1_data)

    if np.min(y1_data)<=change.axes.get_ylim()[0] or np.max(y1_data)>=change.axes.get_ylim()[1]:
        plt.ylim([np.min(y1_data)-np.std(y1_data),np.max(y1_data)+np.std(y1_data)])

    plt.pause(pause_time)

    return change

def process_front(img):
    #Return slope and midpoint 
    print(img.shape)        
    #Crop image
    #Preprocessing in general could be improved alot               
    #img = img[42*2:64*2,48*2:80*2]
    # im = cv.imread('testimg.png')
    imgray = cv.cvtColor(img,cv.COLOR_BGR2GRAY)
    ret,thresh = cv.threshold(imgray,150,255,0)
    
    scale_percent = 300 # percent of original size
    width = int(thresh.shape[1] * scale_percent / 100)
    height = int(thresh.shape[0] * scale_percent / 100)
    dim = (width, height)
    # resize image
    thresh = cv.resize(thresh, dim, interpolation = cv.INTER_AREA)
     
    
    # cv.namedWindow("main", cv.WINDOW_NORMAL)
    # cv.imshow('main', thresh)
    # cv.waitKey()
    # image, contours = cv.findContours(thresh,cv.RETR_TREE,cv.CHAIN_APPROX_SIMPLE)
    # img = cv.drawContours(im, contours, -1, (0,255,0), 3)
    contours, hierarchy = cv.findContours(thresh,cv.RETR_TREE,cv.CHAIN_APPROX_SIMPLE)
    cxs = np.zeros(25)
    cys = np.zeros(25)
    valid = [0,0]
    count = 0
#print(len(contours))
    if(len(contours)>1):
    	for c in range(0,len(contours)):
    		
    		M = cv.moments(contours[c])
    
    		if M["m00"] != 0:
    			cxs[c] = M["m10"] / M["m00"]
    			cys[c] = M["m01"] / M["m00"]
    			if(count<2):
    				valid[count] = c
    				count+=1
    
    else:
    	return -1      #no match                                
    	
    slope = math.atan2((cys[valid[1]]-cys[valid[0]]),(cxs[valid[1]]-cxs[valid[0]]))*(180/math.pi)+90
    xm,ym = thresh.shape
    # print(cxs[valid[1]])
    # print(cxs[valid[0]])
    midx = (cxs[valid[1]]+cxs[valid[0]])/2 -ym/2

    #uncomment this to see line drawn on image for each step
    # cv.line(img,(int(cxs[valid[0]]),int(cys[valid[0]])),(int(cxs[valid[1]]),int(cys[valid[1]])),(255,0,0),1)
    # cv.namedWindow("main", cv.WINDOW_NORMAL)
    # cv.imshow('main', img)
    # cv.waitKey()

    print(slope)
    print(midx)
    return((slope,midx))
# Main loop:
# - perform simulation steps until Webots is stopping the controller
steer = 0
# for plot
if MAKEPLOT is True:
    size = 100 
    x_vec = np.linspace(0, 1, size)
    y_vec = np.linspace(0, 0, size)
    change = []

while robot.step() != -1:
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()
    img_f = front_camera.getImage()
    img_cv = np.frombuffer(img_f, np.uint8).reshape((front_camera.getHeight(), front_camera.getWidth(), 4))
    #img_cv = copyImage(img_f)
    # cv.namedWindow("main", cv.WINDOW_NORMAL)
    # cv.imshow('main', img_cv)
    # cv.waitKey()

    # live plotting
    if MAKEPLOT is True:
        rand_val = np.random.randn(1)
        y_vec[-1] = float(robot.getCurrentSpeed())
        change = live_plotter(x_vec,y_vec,change, "Live Tesla Speed")
        y_vec = np.append(y_vec[1:],0.0)


    nav_con = process_front(img_cv)
    #TODO: implement PID. I was too lazy last night
    if(nav_con!=-1):
        if(abs(nav_con[0])>10):
            steer = nav_con[0]*(math.pi/180)*.15
            steer = steer*(1-(abs(nav_con[1])/50))
        else:
            steer = 0
    print(steer)
    robot.setSteeringAngle(steer)


    front_camera.saveImage("testimg.png",100)
    robot.setCruisingSpeed(25)
    #robot.setSteeringAngle(0)
    # Process sensor data here.

    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)
    pass


    
    