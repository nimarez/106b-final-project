from controller import Robot, Camera, Motor, Node, Supervisor
from occupancy_map import generate_occupancy_map
from dynamic_objects import Projectile, Human
from planner import Planner
from utils import get_pos_at_grid_index, get_grid_index_at_pos, get_vel_in_world_co_per_second, get_vel_in_grid_per_second
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

        self.safe_map_size = self.map_size - 0.5
        
        self.oc_map = generate_occupancy_map(self.safe_map_size, self.dim, self)
        
        # ------------ HUMAN PARMS BEGIN -----------------#
        human_start = (0, 9)
        human_goal = (9, 0)
        speed = 1
        
        self.human = Human(human_start, human_goal, speed)
        times, human_way_indices = self.human.get_traj(self.oc_map)
        
        human_way_points = [get_pos_at_grid_index(i, j, self.safe_map_size, self.dim) for i, j in human_way_indices]
        
        # cyclical_path = human_way_points + list(reversed(human_way_points))[1:]
        
        distraction = 200
        for _ in range(distraction):
            human_way_points.append((10, 10))
            human_way_points.append((10, 9))

        
        s = ",".join([f"{x} {y}" for x, y in human_way_points])
        arg = f"--trajectory={s}"
        start_x, start_y = human_way_points[0]
        self.getFromDef("HUMAN").getField('translation').setSFVec3f([start_x, start_y, 0])
        self.getFromDef("HUMAN").getField('controllerArgs').setMFString(0, arg)
        transformed_speed = get_vel_in_world_co_per_second(speed, self.safe_map_size, self.dim)
        speed_arg = f"--speed={transformed_speed}"
        self.getFromDef("HUMAN").getField('controllerArgs').setMFString(1, speed_arg)
        
        # ------------ HUMAN PARMS END -------------------#

                
        # ------------ CONTROLLER PARMS BEGIN -------------------#
        
        # Control numbers
        self.E = 0   # Cummulative error
        self.old_e = 0  # Previous error
        
        self.Kp = 1.5
        self.Ki = 0.01
        self.Kd = 0.01
        
        self.straightV = 4
        #self.desiredV = 3
        self.turningV = 0
        
        self.arrive_distance = 0.01

        # ------------ CONTROLLER PARMS END -------------------#
        
        # Convert angular velocity to world velocity then to squares/sec
        # NOTE: Assumes these params are correct
        wheel_rad = 0.033
        world_size = 5
        square_velocity = self.straightV * wheel_rad / 2*3.1415 * self.dim / world_size
        turn_time = 2/square_velocity # Just random approximation
        self.planner = Planner(self.dim, square_velocity, turn_time)

        succes, new_occ, tree, times, way_indices = self.planner.generate_compatible_plan(self.oc_map, [self.human], (0,0))
        self.oc_map = new_occ
        way_points = [get_pos_at_grid_index(i, j, self.safe_map_size, self.dim) for i, j in way_indices]
        
        print(way_points)

        
        # self.start_pos = way_points[0]
        self.goal_positions = way_points
        #self.goal_positions = [[0,0], [1,0], [1,1], [0,1],[0,0], [1,0], [1,1], [0,1],[0,0], [1,0], [1,1], [0,1]]
        #self.goal_positions = [[0,0], [1,0], [0,0], [1,0]]
        
        if self.getSupervisor():
            self.robot = self.getSelf()
        
        # To replan after 10 sec
        self.replanned = False
          
    def control_step(self, goal):
        #inspired by: https://github.com/BurakDmb/DifferentialDrivePathTracking/blob/master/main.py
        
        #print("------")
        # Difference in x and y
        d_x = round(goal[0] - self.robot.getField('translation').getSFVec3f()[0], 4)
        #print("dx: ", d_x)
        d_y = round(goal[1] - self.robot.getField('translation').getSFVec3f()[1], 4)
        #print("dy: ", d_y)
        
        if self.is_arrived(d_x,d_y):
            if len(self.goal_positions) > 1:
                self.goal_positions.pop(0)
            # print("arrived")
            return 0,0

        # Angle from robot to goal
        g_theta = np.arctan2(d_y, d_x)
        #print("theta: ", g_theta)
        
        # Error between the goal angle and robot angle
        curr_rotation = self.robot.getField('rotation').getSFRotation()
        if (curr_rotation[2] > 0):
            z_rot = curr_rotation[3]
        else:
            z_rot = -curr_rotation[3]
        alpha = g_theta - z_rot
        #print("alpha: ", alpha)
        #print("sin: ", np.sin(alpha))
        #print("cos: ", np.cos(alpha))
        e = np.arctan2(np.sin(alpha), np.cos(alpha))
        #print("e: ", e)
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
        w = np.arctan2(np.sin(w), np.cos(w))
        # print("w: ", w)
        if abs(w) > 0.1:
            v = self.turningV
            # print("turning")
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
                #if CURRENT_TIME >= 30*1000 and self.replanned == False:
                #    # replan!
                #    print("REPLAN")
                #    current_pos = get_grid_index_at_pos(self.robot.getField('translation').getSFVec3f()[0], self.robot.getField('translation').getSFVec3f()[1])
                #    current_time = 
                #   succes, new_occ, tree, times, way_indices = self.planner.generate_compatible_plan(self.oc_map, [self.human], (0,0), current_pos, )
                #    way_points = [get_pos_at_grid_index(i, j, self.safe_map_size, self.dim) for i, j in way_indices]
                #    print(way_points)
                #    self.replanned = True
                
                #print(self.getFromDef("TURTLEBOT").getField("translation").getSFVec3f())
                                
                self.left_motor.setVelocity(left_speed)
                self.right_motor.setVelocity(right_speed)
                

# main Python program
controller = CCPPController()
controller.run()