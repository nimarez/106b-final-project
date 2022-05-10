import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import np

class Controller():
    def __init__(self):
        # number of cells on row / column to divide world into
        # assumes: square world
        self.dim = 6
        self.map_size = 2
        # Control numbers
        self.E = 0   # Cummulative error
        self.old_e = 0  # Previous error
        
        self.Kp = 1.5
        self.Ki = 0.01
        self.Kd = 0.01
        
        self.straightV = 0.5
        #self.desiredV = 3
        self.turningV = 0
        
        self.arrive_distance = 0.05

        # ------------ CONTROLLER PARMS END -------------------#
        
        # Convert angular velocity to world velocity then to squares/sec
        # NOTE: Assumes these params are correct
        world_size = 5
        square_velocity = self.straightV * self.dim / world_size
        turn_time = 2/square_velocity # Just random approximation
        self.planner = Planner(self.dim, square_velocity, turn_time)

        self.oc_map = np.zeros((self.dim, self.dim))

        succes, new_occ, tree, times, way_indices = self.planner.generate_compatible_plan(self.oc_map, [], (0,0))
        way_points = [get_pos_at_grid_index(i, j, self.safe_map_size, self.dim) for i, j in way_indices]
        self.goal_positions = way_points

        rospy.init_node("controller", anonymous=False)
        rospy.on_shutdown(self.shutdown)
        self.cmd_vel = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
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
                rel_x = self.orientation.pose.x - self.initial.pose.x
                rel_y = self.orientation.pose.y - self.initial.pose.y
                rel_theta = np.pi*(self.orientation.twist.z - self.initial.twist.z)
                
                v1, v2 = self.control_step(self.goal_positions[0], rel_x, rel_y, rel_theta)
                move_cmd = vel_to_twist(v1, v2)
                self.cmd_vel.publish(move_cmd)
            r.sleep()

    def vel_to_twist(left_vel, right_vel):
        move_cmd = Twist()
        move_cmd.linear.x = 0.2
        move_cmd.angular.z = 0
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
        #print("theta: ", g_theta)
        
        # Error between the goal angle and robot angle
        # https://stackoverflow.com/questions/28036652/finding-the-shortest-distance-between-two-angles
        diff = ( g_theta - theta + np.pi ) % (2*np.pi) - np.pi
        alpha = (diff + 2*np.pi) if (diff < -np.pi) else diff
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

        distance_err = np.linalg.norm(difference)
        if distance_err < self.arrive_distance:
            return True
        else:
            return False
        
# main Python program
controller = CCPPController()
controller.run()
        
    def shutdown(self):
        self.cmd_vel.publish(Twist())
        rospy.sleep(1)
 
if __name__ == '__main__':
   Controller().start()
