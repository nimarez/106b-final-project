from controller import Robot, Camera, Motor


class CCPPController(Robot):
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
        self.right_motor.setVelocity(6.67)
        # left motor
        self.left_motor = self.getDevice("left wheel motor")
        self.left_motor.setPosition(float('inf'))
        self.left_motor.setVelocity(6.67)


    def run(self):
        # main control loop: perform simulation steps of 32 milliseconds
        # and leave the loop when the simulation is over
        while self.step(self.timeStep) != -1:
            pass

# main Python program
controller = CCPPController()
controller.run()