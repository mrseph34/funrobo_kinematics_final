from math import *
from funrobo_kinematics.core import Visualizer, RobotSim
from funrobo_kinematics.core.arm_models import TwoDOFRobotTemplate


class TwoDOFRobot(TwoDOFRobotTemplate):
    def __init__(self):
        super().__init__()




if __name__ == "__main__":
    
    model = TwoDOFRobot()
    
    robot = RobotSim(robot_model=model)
    viz = Visualizer(robot=robot)
    viz.run()