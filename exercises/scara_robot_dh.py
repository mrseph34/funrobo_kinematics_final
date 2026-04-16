from math import *
import numpy as np
import funrobo_kinematics.core.utils as ut
from funrobo_kinematics.core import Visualizer, RobotSim
from funrobo_kinematics.core.arm_models import (
    TwoDOFRobotTemplate, ScaraRobotTemplate, FiveDOFRobotTemplate
)


class ScaraRobot(ScaraRobotTemplate):
    def __init__(self):
        super().__init__()


    def calc_forward_kinematics(self, joint_values: list, radians=True):
        curr_joint_values = joint_values.copy()
        
        th1, th2, d3 = curr_joint_values[0], curr_joint_values[1], curr_joint_values[2]
        l1, l2, l3, l4 = self.l1, self.l2, self.l3, self.l4

        # Joint 1: Rotation th1, Height l1, Length l2
        H0_1 = np.array([[cos(th1), -sin(th1), 0, l2*cos(th1)],
                         [sin(th1),  cos(th1), 0, l2*sin(th1)],
                         [0,         0,        1, l1],
                         [0,         0,        0, 1]])

        # Joint 2: Rotation th2, Height l3, Length l4
        H1_2 = np.array([[cos(th2), -sin(th2), 0, l4*cos(th2)],
                         [sin(th2),  cos(th2), 0, l4*sin(th2)],
                         [0,         0,        1, l3],
                         [0,         0,        0, 1]])
        
        # Joint 3: Prismatic d3 (Vertical movement only)
        H2_3 = np.array([[1, 0, 0, 0],
                         [0, 1, 0, 0],
                         [0, 0, 1, -d3],
                         [0, 0, 0, 1]])
        
        Hlist = [H0_1, H1_2, H2_3]

        # Calculate EE position and rotation
        H_ee = H0_1 @ H1_2 @ H2_3  # Final transformation matrix for EE

        # Set the end effector (EE) position
        ee = ut.EndEffector()
        pos = H_ee @ np.array([0, 0, 0, 1])
        ee.x, ee.y, ee.z = pos[0], pos[1], pos[2]
        
        # Extract and assign the RPY (roll, pitch, yaw) from the rotation matrix
        rpy = ut.rotm_to_euler(H_ee[:3, :3])
        ee.rotx, ee.roty, ee.rotz = rpy[0], rpy[1], rpy[2]

        return ee, Hlist


if __name__ == "__main__":
    model = ScaraRobot()
    robot = RobotSim(robot_model=model)
    viz = Visualizer(robot=robot)
    viz.run()