from controller import Robot, Camera, Motor, Node, Supervisor
from occupancy_map import generate_occupancy_map
from planner import Planner
from utils import get_pos_at_grid_index, get_grid_index_at_pos
import logging

import numpy as np
import math

CURRENT_TIME = 0

class CCPPController(Supervisor):
    
    def __init__(self):
        super(CCPPController, self).__init__()
        self.timeStep = 64  # set the control time step

        # get device tags
        self.kinectCamera = self.getDevice("kinect color")
        self.kinectCamera.enable(64)
        self.kinectRange = self.getDevice("kinect range")
        self.kinectRange.enable(64)
        # right motor
        self.right_motor = self.getDevice("right wheel motor")
        self.right_motor.setPosition(float('inf'))
        # left motor
        self.left_motor = self.getDevice("left wheel motor")
        self.left_motor.setPosition(float('inf'))

        # number of cells on row / column to divide world into
        # assumes: square world
        self.dim = 20

        # get map_size
        self.map_size = self.getFromDef("ARENA").getField('floorSize').getSFVec2f()[0]

        self.safe_map_size = self.map_size - 0.4

        self.oc_map = generate_occupancy_map(self.safe_map_size, self.dim, self)

        self.planner = Planner(self.dim)

        new_occ, times, way_indices = self.planner.generate_compatible_plan(self.oc_map, [], (0,0))

        way_points = [get_pos_at_grid_index(i, j, self.safe_map_size, self.dim) for i, j in way_indices]

        print(self.oc_map)
                
        # self.planner.visualize_plan(new_occ, [], (times, way_indices))

        # ------------ CONTROLLER PARMS BEGIN -------------------#
        
        # Control numbers
        self.E = 0   # Cummulative error
        self.old_e = 0  # Previous error
        
        self.Kp = 1
        self.Ki = 0.01
        self.Kd = 0.01
        
        self.straightV = 3
        #self.desiredV = 3
        self.turningV = 0
        
        self.arrive_distance = 0.01

        # ------------ CONTROLLER PARMS END -------------------#
        
        # self.goal_positions = [[0,0], [1,0], [0,0], [1,0]]
        self.goal_positions = way_points
        
        if self.getSupervisor():
            self.robot = self.getSelf()
          
    def control_step(self, goal):
        #inspired by: https://github.com/BurakDmb/DifferentialDrivePathTracking/blob/master/main.py
        
        # print("------")
        # Difference in x and y
        d_x = goal[0] - self.robot.getField('translation').getSFVec3f()[0]
        # print("dx: ", d_x)
        d_y = goal[1] - self.robot.getField('translation').getSFVec3f()[1]
        # print("dy: ", d_y)
        
        if self.is_arrived(d_x,d_y):
            if len(self.goal_positions) > 1:
                self.goal_positions.pop(0)
            # print("arrived")
            return 0,0

        # Angle from robot to goal
        g_theta = np.arctan2(d_y, d_x)
        # print("theta: ", g_theta)
        
        # Error between the goal angle and robot angle
        alpha = g_theta - self.robot.getField('rotation').getSFRotation()[3]
        #alpha = g_theta - math.radians(90)
        # print("alpha: ", alpha)
        # print("sin: ", np.sin(alpha))
        # print("cos: ", np.cos(alpha))
        e = np.arctan2(np.sin(alpha), np.cos(alpha))
        # print("e: ", e)
        #print(e)
        e_P = e
        e_I = self.E + e
        while (abs(e_I) > 2*math.pi):
            e_I = np.sign(e_I)*(abs(e_I)-2*math.pi)
        self.E = e_I
        # TO DO: if self.E is greater than 2*pi, set it to 0 + difference. (Same thing for negative)
        e_D = e - self.old_e
        self.old_e = e
        
        # This PID controller only calculates the angular
        # velocity with constant speed of v
        # The value of v can be specified by giving in parameter or
        # using the pre-defined value defined above.
        # print("integral term: ", e_I)
        # print("derrivative term: ", e_D)
        w = self.Kp*e_P + self.Ki*e_I + self.Kd*e_D
        w = round(np.arctan2(np.sin(w), np.cos(w)), 3)
        # print("w: ", w)
        if abs(w) > 0.2:
            v = self.turningV
            #print("turning")
        else:
            v = self.straightV
            #print("straight")
        
        return v-w, v+w
        
    def is_arrived(self, dx, dy):
        difference = np.array([dx, dy])

        distance_err = difference @ difference.T
        if distance_err < self.arrive_distance:
            return True
        else:
            return False
        
    def run(self):
        global CURRENT_TIME
        # main control loop: perform simulation steps of 32 milliseconds
        # and leave the loop when the simulation is over
        while self.step(self.timeStep) != -1:
            CURRENT_TIME += self.timeStep
            if self.getSupervisor():
                left_speed, right_speed = self.control_step(self.goal_positions[0])
                                
                self.left_motor.setVelocity(left_speed)
                self.right_motor.setVelocity(right_speed)
                

# main Python program
controller = CCPPController()
controller.run()