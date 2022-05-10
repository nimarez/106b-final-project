#!/usr/bin/env python3
import rospy
import math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy as np
from planner import Planner
from utils import get_pos_at_grid_index

class Controller():
    def __init__(self):
        # number of cells on row / column to divide world into
        # assumes: square world
        self.dim = 6
        self.map_size = 1.5
        # Control numbers
        self.E = 0   # Cummulative error
        self.old_e = 0  # Previous error
        
        self.w_scale = 1.5
        self.Kp = self.w_scale*1.5
        self.Ki = self.w_scale*0.0
        self.Kd = self.w_scale*0.1
        
        self.straightV = 4
        #self.desiredV = 3
        self.turningV = 0
        
        self.arrive_distance = 0.4

        # ------------ CONTROLLER PARMS END -------------------#
        
        # Convert angular velocity to world velocity then to squares/sec
        # NOTE: Assumes these params are correct
        square_velocity = self.straightV * self.dim / self.map_size
        turn_time = 2/square_velocity # Just random approximation
        self.planner = Planner(self.dim, square_velocity, turn_time)

        self.oc_map = np.zeros((self.dim, self.dim))

        succes, new_occ, tree, times, way_indices = self.planner.generate_compatible_plan(self.oc_map, [], (0,0))
        self.planner.visualize_plan(new_occ, [], (times, way_indices), t_step=1000)
        way_points = [(i * self.map_size / self.dim, j * self.map_size / self.dim) for i, j in way_indices]
        self.goal_positions = way_points

        rospy.init_node("controller", anonymous=False)
        rospy.on_shutdown(self.shutdown)
        self.cmd_vel = rospy.Publisher("/mobile_base/commands/velocity", Twist, queue_size=10)
        self.initial = None
        self.odom = rospy.Subscriber("/odom", Odometry, self.callback, queue_size=10)

    def callback(self, data):
        if self.initial is None:
            self.initial = data
        self.orientation = data

    def start(self):
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.initial is not None:
                rel_x = self.orientation.pose.pose.position.x - self.initial.pose.pose.position.x
                rel_y = self.orientation.pose.pose.position.y - self.initial.pose.pose.position.y
                rel_theta = np.pi*(self.orientation.pose.pose.orientation.z)# - self.initial.pose.pose.orientation.z)

                print("Attempting to go to", self.goal_positions[0])
                print("At", rel_x, rel_y, rel_theta)
                
                v1, v2 = self.control_step(self.goal_positions[0], rel_x, rel_y, rel_theta)
                print("Temp", v1, v2)
                move_cmd = self.vel_to_twist(v1, v2)
                #print("Moving with", move_cmd)
                self.cmd_vel.publish(move_cmd)
            r.sleep()

    def vel_to_twist(self, left_vel, right_vel):
        # http://planning.cs.uiuc.edu/node659.html
        # https://kobuki.readthedocs.io/en/release-1.0.x/conversions.html
        r = 0.035
        L = 0.230 # TODO Could be wrong
        move_cmd = Twist()
        # TODO: Scaling factor might be wrong
        move_cmd.linear.x = r/2*(right_vel + left_vel)
        move_cmd.angular.z = r/L*(right_vel - left_vel)
        return move_cmd
          
    def control_step(self, goal, x, y, theta):
        #inspired by: https://github.com/BurakDmb/DifferentialDrivePathTracking/blob/master/main.py
        
        #print("------")
        # Difference in x and y
        d_x = round(goal[0] - x, 4)
        #print("dx: ", d_x)
        d_y = round(goal[1] - y, 4)
        #print("dy: ", d_y)
        
        if self.is_arrived(d_x,d_y):
            if len(self.goal_positions) > 1:
                self.goal_positions.pop(0)
            # print("arrived")
            return 0,0

        # Angle from robot to goal
        g_theta = np.arctan2(d_y, d_x)
        print("target theta: ", g_theta)
        
        # Error between the goal angle and robot angle
        # https://stackoverflow.com/questions/28036652/finding-the-shortest-distance-between-two-angles
        diff = ( g_theta - theta + np.pi ) % (2*np.pi) - np.pi
        alpha = (diff + 2*np.pi) if (diff < -np.pi) else diff
        print("alpha: ", alpha)
        #print("sin: ", np.sin(alpha))
        #print("cos: ", np.cos(alpha))
        e = np.arctan2(np.sin(alpha), np.cos(alpha))
        print("e: ", e)
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
        print("p term: ", e_P)
        print("integral term: ", e_I)
        print("derrivative term: ", e_D)
        w = self.Kp*e_P + self.Ki*e_I + self.Kd*e_D
        #w = self.w_scale*np.arctan2(np.sin(w), np.cos(w))
        print("w: ", w)
        t1 = 0.4
        t2 = 0.1
        if abs(w) > t1:
            v = self.turningV
            # print("turning")
        elif abs(w) > t2:
            scaling = (abs(w) - t2) / (t1 - t2)
            v = self.turningV*(1 - scaling) + self.straightV*scaling
        else:
            v = self.straightV
            #print("straight")
        
        return v-w, v+w
        
    def is_arrived(self, dx, dy):
        difference = np.array([dx, dy])

        distance_err = np.linalg.norm(difference)
        if distance_err < self.arrive_distance:
            return True
        else:
            return False
        
    def shutdown(self):
        self.cmd_vel.publish(Twist())
        rospy.sleep(1)
 
if __name__ == '__main__':
   Controller().start()
