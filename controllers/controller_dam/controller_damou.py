"""my_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot ,Motor , DistanceSensor

class Motors ():

    def __init__(self,robot):
        super().__init__()
        self.lwheel : Motor = self.getDevice('left whell')
        self.rwheel : Motor = self.getDevice('right wheel')
    
    def turn_left(self):
        self.lwheel.setPosition(-10.0)
        self.rwheel.setPosition(0)

    def strait(self):
        self.lwheel.setPosition(10.0)
        self.rwheel.setPosition(10.0)

class monRobot(Robot):
    
    def __init__(self):
        super().__init__()
        self.motors=Motors(robot)

        self.ps0:DistanceSensor = self.getDevice('ps0')
        self.ps7:DistanceSensor = self.getDevice('ps7')
        self.ps0.enable()
        self.ps7.enable()


    def run(self):
        self.motors.strait()
        print(self.ps0.getValue())
        if self.ps0.getValue() >= 0.5 and self.ps7.getValue() >= 0.5:
            self.motors.turn_left()
        pass





# create the Robot instance.
robot = monRobot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getDevice('motorname')
#  ds = robot.getDevice('dsname')
#  ds.enable(timestep)

# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:

    robot.run() # on avance et on esquive les murs


    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()

    # Process sensor data here.

    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)
    pass

# Enter here exit cleanup code.
