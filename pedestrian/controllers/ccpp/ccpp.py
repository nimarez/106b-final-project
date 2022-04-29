from controller import Robot, Camera, Motor, Node, Supervisor
from collections import namedtuple

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
        
        Box = namedtuple('Box', ['translation', 'rotation', 'size'])
        
        if self.getSupervisor():
            self.robot = self.getSelf()
            arena = self.getFromDef("ARENA")
            size, _ = arena.getField("floorSize").getSFVec2f() # Assume square
            boxes_field = self.getFromDef("STATIC").getField("children")
            boxes = []
            for i in range(boxes_field.getCount()):
                box_obj = boxes_field.getMFNode(i)
                trans = box_obj.getField("translation").getSFVec3f()
                rot = box_obj.getField("rotation").getSFRotation()
                size = box_obj.getField("size").getSFVec3f()
                boxes.append(Box(trans, rot, size))
            
    time_arr = [0, 1280, 2560, 3840, 5120, 6400]
    poss_arr = [(1.49, 0), (1.7, 0.5), (1, 0.5), (0.6, 0.5), (1, 1), (1.7, 1.3)]
    
    def turutlebot_movement():
        # move the turtlebot
        
        
        
        
    def run(self):
        global CURRENT_TIME
        # main control loop: perform simulation steps of 32 milliseconds
        # and leave the loop when the simulation is over
        while self.step(self.timeStep) != -1:
            CURRENT_TIME += self.timeStep
            if self.getSupervisor():
                position = self.robot.getPosition()
                if (4.95 < position[0] ** 2 + position[1] ** 2 < 5.05):
                    self.left_motor.setVelocity(-self.left_motor.getVelocity())
                    self.right_motor.setVelocity(-self.right_motor.getVelocity())
    
            pass

# main Python program
controller = CCPPController()
controller.run()