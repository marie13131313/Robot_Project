"""my_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot ,Motor , DistanceSensor, Compass

class Motors ():

    def __init__(self,robot:Robot):
        super().__init__()
        self.lwheel : Motor = robot.getDevice('left wheel')
        self.rwheel : Motor = robot.getDevice('right wheel')
    
    def turn_left(self):
        self.lwheel.setPosition(-10.0)
        self.rwheel.setPosition(0)

    def strait(self):
        self.lwheel.setPosition(10.0)
        self.rwheel.setPosition(10.0)

class monRobot(Robot):
    
    def __init__(self):
        super().__init__()
        self.timestep = int(self.getBasicTimeStep())
        self.motors=Motors(self)


        # self.ps0:DistanceSensor = self.getDevice('ps0')
        # self.ps7:DistanceSensor = self.getDevice('ps7')
        # self.ps0.enable(self.timestep)
        # self.ps7.enable(self.timestep)
        self.compass:Compass = self.getDevice('compass')
        self.compass.enable(self.timestep)


    def run(self):
        self.motors.strait()
        # print(self.ps0.getValue())
        # if self.ps0.getValue() >= 0.5 and self.ps7.getValue() >= 0.5:
        #     self.motors.turn_left()
        # pass

        print(self.compass.getValues())

    


# create the Robot instance.
robot = monRobot()

# get the time step of the current world.


# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getDevice('motorname')
#  ds = robot.getDevice('dsname')
#  ds.enable(timestep)

# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(robot.timestep) != -1:

    robot.run() # on avance et on esquive les murs


    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()

    # Process sensor data here.

    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)
    pass

# Enter here exit cleanup code.

