from controller import Robot, Camera, Motor, DistanceSensor

# Créer un contrôleur pour le robot Summit XL Steel
class MarieBastienRobot(Robot):
    def __init__(self):
        super().__init__()
        self.max_speed = 5.0
        self.wall_distance_threshold = 0.5  

        self.timestep = int(self.getBasicTimeStep())

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
                self.front_right_wheel.setVelocity(0)
                self.back_right_wheel.setVelocity(0)
            elif self.ds_left.getValue() < self.wall_distance_threshold:
                # Éviter le mur à droite
                self.front_left_wheel.setVelocity(0)
                self.back_left_wheel.setVelocity(0)
                self.front_right_wheel.setVelocity(self.max_speed)
                self.back_right_wheel.setVelocity(self.max_speed)
            else:
            #     # Suivre la piste en utilisant le capteur avant
            #     if front_distance > self.wall_distance_threshold:
            #         # Aucun mur devant, avancer
            #         self.robot.set_left_velocity(self.max_speed)
            #         self.robot.set_right_velocity(self.max_speed)
            #     else:
            #       # Mur devant, ajuster la direction
                    self.front_left_wheel.setVelocity(self.max_speed)
                    self.back_left_wheel.setVelocity(self.max_speed)
                    self.front_right_wheel.setVelocity(self.max_speed)
                    self.back_right_wheel.setVelocity(self.max_speed)

            # # Pause pour permettre la lecture des capteurs
            # self.robot.wait(0.1)



robot = MarieBastienRobot()
robot.run()
