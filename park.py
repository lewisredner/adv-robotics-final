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

#for att in dir(robot):
#    print(att,getattr(robot,att))
# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())/1e3
#print(dir(robot))

# You should insert a getDevice-like function in order to get the
#instance of a device of the robot. Something like:
 # motor = robot.getMotor('motorname')
 # ds = robot.getDistanceSensor('dsname')
 # ds.enable(timestep)
 
carleng = 4.75
 
k = 0.5  # control gain
Kp = 1.0  # speed proportional gain
dt = 0.1  # [s] time difference
L = carleng/5  # [m] Wheel base of vehicle
max_steer = np.radians(30.0)  # [rad] max steering angle

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

    def update(self, acceleration, delta):
        """
        Update the state of the vehicle.
        Stanley Control uses bicycle model.
        :param acceleration: (float) Acceleration
        :param delta: (float) Steering
        """
        delta = np.clip(delta, -max_steer, max_steer)

        self.x += self.v * np.cos(self.yaw) * dt
        self.y += self.v * np.sin(self.yaw) * dt
        self.yaw += self.v / L * np.tan(delta) * dt
        self.yaw = normalize_angle(self.yaw)
        self.v += acceleration * dt


def pid_control(target, current):
    """
    Proportional control for the speed.
    :param target: (float)
    :param current: (float)
    :return: (float)
    """
    return Kp * (target - current)

def stanley_control(state, cx, cy, cyaw, last_target_idx):
    """
    Stanley steering control.
    :param state: (State object)
    :param cx: ([float])
    :param cy: ([float])
    :param cyaw: ([float])
    :param last_target_idx: (int)
    :return: (float, int)
    """
    current_target_idx, error_front_axle = calc_target_index(state, cx, cy)

    if last_target_idx >= current_target_idx:
        current_target_idx = last_target_idx

    # theta_e corrects the heading error
    theta_e = normalize_angle(cyaw[current_target_idx] - state.yaw)
    # theta_d corrects the cross track error
    theta_d = np.arctan2(k * error_front_axle, state.v)
    # Steering control
    delta = theta_e + theta_d

    return delta, current_target_idx


def normalize_angle(angle):
    """
    Normalize an angle to [-pi, pi].
    :param angle: (float)
    :return: (float) Angle in radian in [-pi, pi]
    """
    while angle > np.pi:
        angle -= 2.0 * np.pi

    while angle < -np.pi:
        angle += 2.0 * np.pi

    return angle


def calc_target_index(state, cx, cy):
    """
    Compute index in the trajectory list of the target.
    :param state: (State object)
    :param cx: [float]
    :param cy: [float]
    :return: (int, float)
    """
    # Calc front axle position
#     pdb.set_trace()
    fx = state.x + L * np.cos(state.yaw)
    fy = state.y + L * np.sin(state.yaw)

    # Search nearest point index
    dx = [fx - icx for icx in cx]
    dy = [fy - icy for icy in cy]
    d = np.hypot(dx, dy)
    target_idx = np.argmin(d)

    # Project RMS error onto front axle vector
    front_axle_vec = [-np.cos(state.yaw + np.pi / 2),
                      -np.sin(state.yaw + np.pi / 2)]
    error_front_axle = np.dot([dx[target_idx], dy[target_idx]], front_axle_vec)

    return target_idx, error_front_axle

lidar.enable(30)
        
# Main loop:
# - perform simulation steps until Webots is stopping the controller
rho = 5.8
step_size = 0.01

steer = 0
distSinceObs = 0
bigGap = 0
accumDist = 0
obstacles = []
obstacles.append(0)
spotFound = False
mode = 'Looking'
oldSpeed = 0
while robot.step() != -1:
    curSpeed = robot.getCurrentSpeed()*1000/3600

    rangeData = lidar.getRangeImage()
    toTheRight = rangeData[-1]
    if 25*.99 < (robot.getCurrentSpeed()) < 25*1.0:
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
    
    if bigGap > 7 and obstacles[-1] == 1:
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
            spotwidth = 2.75
            planleng = distSinceObs - 2*errors
            goal = [accumDist + planleng/2,toTheRight + spotwidth/2]
            path = reeds_shepp.path_sample((carleng/2,0,0), (goal[0],goal[1],0) , rho, step_size)
            xs = np.array([q[0] for q in path])
            ys = np.array([q[1] for q in path])
            thetas = np.array([q[2] for q in path])
            
            target_speed = 2.5  # [m/s]

            max_simulation_time = 200.0
            
            state = State(x=0, y=0.0, yaw=0, v=0.0)
            
            last_idx = len(xs) - 1
            time = 0.0
            x = [state.x]
            y = [state.y]
            yaw = [state.yaw]
            v = [state.v]
            t = [0.0]
            target_idx, _ = calc_target_index(state, xs, ys)
            dt = timestep
            
            sign = []
            for i in range(len(xs)-1):
                if xs[i+1] > xs[i]:
                    sign.append(1)
                else:
                    sign.append(-1)
            sign.append(-1)
            
            robot.setBrakeIntensity(1)
    else:
        print(obstacles)
        print(distSinceObs)
        robot.setCruisingSpeed(25)
        robot.setSteeringAngle(steer*np.pi/180)
        
    if mode == 'Parking':
        ai = pid_control(sign[target_idx]*target_speed, state.v)
        di, target_idx = stanley_control(state, xs, ys, thetas, target_idx)
        state.update(ai, di)
    
        time += dt
    
        robot.setCruisingSpeed(-1*sign[target_idx]*target_speed)
        robot.setSteeringAngle(di)
        x.append(state.x)
        y.append(state.y)
        yaw.append(state.yaw)
        v.append(state.v)
        t.append(time)
        
        if np.hypot(goal[0]-state.x,goal[1]-state.y) < 0.5:
            mode = 'Parked'
            break
            
        
        print('Steering angle: {}' .format(di))
        print('Speed: {}' .format(curSpeed))
    
    print(mode)
    
    # print(robot.getCurrentSpeed()*timestep)
    
    

    oldSpeed = curSpeed
    
    #robot.setSteeringAngle(0)
    # Process sensor data here.

    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)
    pass