from funrobo_kinematics.core.visualizer import Visualizer, RobotSim
from funrobo_kinematics.core.arm_models import TwoDOFRobotTemplate

if __name__ == "__main__":
    model = TwoDOFRobotTemplate()
    robot = RobotSim(robot_model=model)
    viz = Visualizer(robot=robot)
    viz.run()