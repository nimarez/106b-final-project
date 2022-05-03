from controller import Robot, Camera, Motor, Node, Supervisor
from collections import namedtuple

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
        #self.right_motor.setVelocity(6.67)
        # left motor
        self.left_motor = self.getDevice("left wheel motor")
        self.left_motor.setPosition(float('inf'))
        #self.left_motor.setVelocity(6.67)
        
        # number of cells on row / column to divide world into
        # assumes: square world
        self.diff = 20
        
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
        
        self.goal_positions = [[0,0], [1,0], [0,0], [1,0]]
        
        root_node = self.getRoot()
        
        #Box = namedtuple('Box', ['translation', 'rotation', 'size'])
        
        if self.getSupervisor():
            self.robot = self.getSelf()
        #    arena = self.getFromDef("ARENA")
        #    size, _ = arena.getField("floorSize").getSFVec2f() # Assume square
        #    boxes_field = self.getFromDef("STATIC").getField("children")
        #    boxes = []
        #    for i in range(boxes_field.getCount()):
        #        box_obj = boxes_field.getMFNode(i)
        #        trans = box_obj.getField("translation").getSFVec3f()
        #        rot = box_obj.getField("rotation").getSFRotation()
        #        size = box_obj.getField("size").getSFVec3f()
        #        boxes.append(Box(trans, rot, size))
            
        #time_arr = [0, 1280, 2560, 3840, 5120, 6400]
        #poss_arr = [(1.49, 0), (1.7, 0.5), (1, 0.5), (0.6, 0.5), (1, 1), (1.7, 1.3)]
        
        self.generate_occupancy_map(root_node)
    
    
    def generate_occupancy_map(self, root_node):
        # Takes in root node of the world
        # Finds all cube like things and their bounding boxes
        # Returns an int 2D array (1 or 0) corresponding to whether an object is in that location
        # Discretizes with the grid size equal to the robot size
        
        # empty occupancy map (0 = free, 1 = occupied)
        oc_map = np.zeros(shape=(self.diff,self.diff))
        
        # get the group containing all walls
        group_node = self.getFromDef("STATIC")
        # get the field of the group containg the children (=wall) nodes
        children_field = group_node.getField('children')
        # how many children does it have?
        nb_of_walls = children_field.getCount()
        
        for i in range(nb_of_walls):
            wall = children_field.getMFNode(i)
            
            position = wall.getPosition()
            size = wall.getField('children').getMFNode(0).getField('geometry').getSFNode().getField('size').getSFVec3f()
            
            self.add_rectangle_to_occupancy(size, position, oc_map, 10, self.diff)
            
        #print(oc_map)
    
    
    def add_rectangle_to_occupancy(self, size, position, oc_map, map_size, diff):
        # for now assume:
            # retangles are not rotated other than 90 degrees
            # only rectangles
            
        # start with top left corner
        size_in_x = size[0]
        size_in_y = size[1]
        current_x = position[0] - size_in_x / 2
        current_y = position[1] - size_in_y / 2
        
        
        step_value = map_size/diff
        
        while current_x < position[0] + size_in_x / 2:
            while current_y < position[1] + size_in_y / 2:
                grid = self.get_grid_index_at_pos(current_x, current_y, map_size, diff)
                oc_map[grid[0]][grid[1]] = 1
                current_y += step_value
            current_x += step_value
            current_y = position[1] - size_in_y / 2
        return
        
    def get_grid_index_at_pos(self, x, y, map_size, diff):
        return (math.floor(((x + map_size / 2)/map_size)*diff), math.floor(((y + map_size / 2)/map_size)*diff))
    
    def control_step(self, goal):
        #inspired by: https://github.com/BurakDmb/DifferentialDrivePathTracking/blob/master/main.py
        
        print("------")
        # Difference in x and y
        d_x = goal[0] - self.robot.getField('translation').getSFVec3f()[0]
        print("dx: ", d_x)
        d_y = goal[1] - self.robot.getField('translation').getSFVec3f()[1]
        print("dy: ", d_y)
        
        if self.is_arrived(d_x,d_y):
            if len(self.goal_positions) > 1:
                self.goal_positions.pop(0)
            print("arrived")
            return 0,0

        # Angle from robot to goal
        g_theta = np.arctan2(d_y, d_x)
        print("theta: ", g_theta)
        
        # Error between the goal angle and robot angle
        alpha = g_theta - self.robot.getField('rotation').getSFRotation()[3]
        #alpha = g_theta - math.radians(90)
        print("alpha: ", alpha)
        print("sin: ", np.sin(alpha))
        print("cos: ", np.cos(alpha))
        e = np.arctan2(np.sin(alpha), np.cos(alpha))
        print("e: ", e)
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
        print("integral term: ", e_I)
        print("derrivative term: ", e_D)
        w = self.Kp*e_P + self.Ki*e_I + self.Kd*e_D
        w = np.arctan2(np.sin(w), np.cos(w))
        print("w: ", w)
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
    
            pass

# main Python program
controller = CCPPController()
controller.run()