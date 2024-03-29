from controller import Robot, Camera, Motor, DistanceSensor

# Créer un contrôleur pour le robot Summit XL Steel
class MarieBastienRobot(Robot):
    def __init__(self):
        super().__init__()
        
        self.max_speed = 37
        self.wall_distance_threshold = 1000.0

        self.timestep = int(self.getBasicTimeStep())

        self.camera:Camera=self.getDevice("rgb_camera")
        self.camera.enable(self.timestep)

        self.ds_left:DistanceSensor = self.getDevice('distance sensor left')
        self.ds_right:DistanceSensor = self.getDevice('distance sensor right')
        self.ds_left.enable(self.timestep)
        self.ds_right.enable(self.timestep)

        self.front_left_wheel:Motor = self.getDevice('front_left_wheel_joint')
        self.front_right_wheel:Motor = self.getDevice('front_right_wheel_joint')
        self.back_left_wheel:Motor = self.getDevice('back_left_wheel_joint')
        self.back_right_wheel:Motor = self.getDevice('back_right_wheel_joint')

        self.front_left_wheel.setPosition(float('inf'))
        self.front_right_wheel.setPosition(float('inf'))
        self.back_left_wheel.setPosition(float('inf'))
        self.back_right_wheel.setPosition(float('inf'))

        self.front_left_wheel.setVelocity(0)
        self.front_right_wheel.setVelocity(0)
        self.back_left_wheel.setVelocity(0)
        self.back_right_wheel.setVelocity(0)
        

    def run(self):
        while self.step(self.timestep) != 1:
            # Lire les données des capteurs de distance

            print('left',self.ds_left.getValue())
            print('right',self.ds_right.getValue())

            # Vérifier la présence de murs sur les côtés
            if self.ds_left.getValue() < self.wall_distance_threshold:
                # Éviter le mur à gauche
                self.front_left_wheel.setVelocity(self.max_speed)
                self.back_left_wheel.setVelocity(self.max_speed)
                self.front_right_wheel.setVelocity(-self.max_speed)
                self.back_right_wheel.setVelocity(-self.max_speed)
                print("je vais à droite")

            elif self.ds_right.getValue() < self.wall_distance_threshold:
                # Éviter le mur à droite
                self.front_left_wheel.setVelocity(-self.max_speed)
                self.back_left_wheel.setVelocity(-self.max_speed)
                self.front_right_wheel.setVelocity(self.max_speed)
                self.back_right_wheel.setVelocity(self.max_speed)
                print("je vais à gauche")
                
            else:
                self.front_left_wheel.setVelocity(self.max_speed)
                self.back_left_wheel.setVelocity(self.max_speed)
                self.front_right_wheel.setVelocity(self.max_speed)
                self.back_right_wheel.setVelocity(self.max_speed)
                print("je vais tout droit")
            
            

robot = MarieBastienRobot()
robot.run()

