"""my_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot,Motor,Camera,Compass

class Propellers ():
    def __init__(self,robot:Robot):
        self.propeller_fr_right = robot.getDevice('front right propeller')
        self.propeller_fr_left = robot.getDevice('front left propeller')
        self.propeller_rr_right = robot.getDevice('rear right propeller')
        self.propeller_rr_left = robot.getDevice('rear left propeller')
        self.propeller_fr_right.setPosition(float('inf'))
        self.propeller_fr_left.setPosition(float('inf'))
        self.propeller_rr_right.setPosition(float('inf'))
        self.propeller_rr_left.setPosition(float('inf'))

        pass

    def turn_right(self,Value):
        self.propeller_fr_right.setVelocity(-Value)
        self.propeller_fr_left.setVelocity(Value)
        self.propeller_rr_right.setVelocity(-Value)
        self.propeller_rr_left.setVelocity(Value)
        pass

    def turn_left(self,Value):
        self.propeller_fr_right.setVelocity(Value)
        self.propeller_fr_left.setVelocity(-Value)
        self.propeller_rr_right.setVelocity(Value)
        self.propeller_rr_left.setVelocity(-Value)
        pass

    def move_forward(self,Value):
        self.propeller_fr_right.setVelocity(-Value)
        self.propeller_fr_left.setVelocity(-Value)
        self.propeller_rr_right.setVelocity(Value)
        self.propeller_rr_left.setVelocity(Value)
        pass

    def move_backward(self,Value):
        self.propeller_fr_right.setVelocity(Value)
        self.propeller_fr_left.setVelocity(Value)
        self.propeller_rr_right.setVelocity(-Value)
        self.propeller_rr_left.setVelocity(-Value)
        pass

    def going_up(self,Value):
        self.propeller_fr_right.setVelocity(Value)
        self.propeller_fr_left.setVelocity(Value)
        self.propeller_rr_right.setVelocity(Value)
        self.propeller_rr_left.setVelocity(Value)
        pass

    def going_down(self,Value):
        self.propeller_fr_right.setVelocity(-Value)
        self.propeller_fr_left.setVelocity(-Value)
        self.propeller_rr_right.setVelocity(-Value)
        self.propeller_rr_left.setVelocity(-Value)
        pass









class Maveric(Robot):
    def __init__(self):
        super().__init__()

        self.timestep = int(self.getBasicTimeStep())

        self.propellers=Propellers(self)
        self.compass:Compass=self.getDevice('compass')
        self.compass.enable(self.timestep)
        self.camera = self.getDevice("camera")
        self.camera.enable(self.timestep)
        self.imu = self.getDevice("inertial unit")
        self.imu.enable(self.timestep)
        self.gps = self.getDevice("gps")
        self.gps.enable(self.timestep)
        self.gyro = self.getDevice("gyro")
        self.gyro.enable(self.timestep)

        self.target_altitude = 0
        self.target_position = [0, 0, 0]

    def ValueCompass(self):
        north = self.compass.getValues()
        rad = math.atan2(north[1], north[0])
        bearing = (rad - 1.5708) / math.pi * 180.0
        if (bearing < 0.0):
            bearing = bearing + 360.0
        return bearing
    
    def set_position(self, pos):
        """
            pos (list): [X,Y,Z,yaw,pitch,roll] current absolute position and angles
        """
        self.current_pose = pos
        
    def run(self):
        robot.propellers.going_up(210)
        """robot.propellers.move_forward(100)"""
        
        waypoints = [[-54, 76, 4], [-60, 69, 7], [-74, 75, 2], [-68, 59, 10]]

        roll, pitch, yaw = self.imu.getRollPitchYaw()
        x_pos, y_pos, altitude = self.gps.getValues()
        roll_acceleration, pitch_acceleration, _ = self.gyro.getValues()
        self.set_position([x_pos, y_pos, altitude, roll, pitch, yaw])









# create the Robot instance.
robot = Maveric()


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
    

    robot.run()

    pass

# Enter here exit cleanup code.
