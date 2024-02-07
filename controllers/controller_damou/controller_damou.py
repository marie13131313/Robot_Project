"""my_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot ,Motor , DistanceSensor, Compass
import math

class Motors ():

    def __init__(self,robot:Robot):
        super().__init__()
        self.lwheel : Motor = robot.getDevice('left wheel')
        self.rwheel : Motor = robot.getDevice('right wheel')
        self.lwheel.setPosition(float("inf"))
        self.rwheel.setPosition(float("inf"))

    def turn_left(self, Value):
        self.lwheel.setVelocity(-Value)
        self.rwheel.setVelocity(Value)

    def turn_right(self, Value):
        self.lwheel.setVelocity(Value)
        self.rwheel.setVelocity(-Value)

    def forward(self, Value):
        self.lwheel.setVelocity(Value)
        self.rwheel.setVelocity(Value)

    def backward(self, Value):
        self.lwheel.setVelocity(Value)
        self.rwheel.setVelocity(Value)

class monRobot(Robot):
    
    def __init__(self):
        super().__init__()
        self.timestep = int(self.getBasicTimeStep())
        self.motors=Motors(self)


        self.so2:DistanceSensor = self.getDevice('so2')
        self.so5:DistanceSensor = self.getDevice('so5')
        self.so2.enable(self.timestep)
        self.so5.enable(self.timestep)
        self.compass:Compass = self.getDevice('compass')
        self.compass.enable(self.timestep)


    def run(self):
       
        self.motors.forward(8)
        
        print(self.so2.getValue())
        if (self.so2.getValue() <= 200 and self.so2.getValue() != 0) or (self.so5.getValue() <= 200 and self.so5.getValue() != 0):
            self.motors.turn_left(5)
        pass

        print(self.ValueCompass())

    def ValueCompass(self):
        north = self.compass.getValues()
        rad = math.atan2(north[1], north[0])
        bearing = (rad - 1.5708) / math.pi * 180.0
        if (bearing < 0.0):
            bearing = bearing + 360.0
        return bearing



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

