from math import *
import numpy as np
import funrobo_kinematics.core.utils as ut
from funrobo_kinematics.core.visualizer import Visualizer, RobotSim
from funrobo_kinematics.core.arm_models import (
    TwoDOFRobotTemplate, ScaraRobotTemplate, FiveDOFRobotTemplate
)



class TwoDOFRobot(TwoDOFRobotTemplate):
    def __init__(self):
        super().__init__()


    def calc_forward_kinematics(self, joint_values: list, radians=True):
        curr_joint_values = joint_values.copy()
        
        th1, th2 = curr_joint_values[0], curr_joint_values[1]
        l1, l2 = self.l1, self.l2


        H0_1 = np.array([[cos(th1), -sin(th1), 0, l1*cos(th1)],
                         [sin(th1), cos(th1), 0, l1*sin(th1)],
                         [0, 0, 1, 0],
                         [0, 0, 0, 1]]
                        )


        H1_2 = np.array([[cos(th2), -sin(th2), 0, l2*cos(th2)],
                         [sin(th2), cos(th2), 0, l2*sin(th2)],
                         [0, 0, 1, 0],
                         [0, 0, 0, 1]]
                        )
        
        Hlist = [H0_1, H1_2]


        # Calculate EE position and rotation
        H_ee = H0_1@H1_2  # Final transformation matrix for EE


        # Set the end effector (EE) position
        ee = ut.EndEffector()
        ee.x, ee.y, ee.z = (H_ee @ np.array([0, 0, 0, 1]))[:3]
        
        # Extract and assign the RPY (roll, pitch, yaw) from the rotation matrix
        rpy = ut.rotm_to_euler(H_ee[:3, :3])
        ee.rotx, ee.roty, ee.rotz = rpy[0], rpy[1], rpy[2]


        return ee, Hlist


    def calc_inverse_kinematics(self, ee: ut.EndEffector, init_joint_values: list, soln=0):
        x, y = ee.x, ee.y
        l1, l2 = self.l1, self.l2

        cos_th2 = (x**2 + y**2 - l1**2 - l2**2) / (2 * l1 * l2)
        cos_th2 = np.clip(cos_th2, -1.0, 1.0)

        sin_th2 = sqrt(1 - cos_th2**2) if soln == 0 else -sqrt(1 - cos_th2**2)
        th2 = atan2(sin_th2, cos_th2)

        k1 = l1 + l2 * cos_th2
        k2 = l2 * sin_th2
        th1 = atan2(y, x) - atan2(k2, k1)

        joint_values = [th1, th2]

        if not ut.check_joint_limits(joint_values, self.joint_limits):
            sin_th2 = -sin_th2
            th2 = atan2(sin_th2, cos_th2)
            k2 = l2 * sin_th2
            th1 = atan2(y, x) - atan2(k2, k1)
            joint_values = [th1, th2]

        return joint_values


    def calc_numerical_ik(self, ee: ut.EndEffector, init_joint_values: list, max_iter=1000, tol=0.002):
        theta = np.array(init_joint_values, dtype=float)

        # move robot slightly out of zeros singularity
        if all(t == .0 for t in theta):
            theta += np.random.rand(len(theta)) * .02

        x_d = np.array([ee.x, ee.y])

        for _ in range(max_iter):
            curr_ee, _ = self.calc_forward_kinematics(theta.tolist(), radians=True)
            x_curr = np.array([curr_ee.x, curr_ee.y])

            e_x = x_d - x_curr

            if np.linalg.norm(e_x) < tol:
                break

            theta = theta + self.inverse_jacobian(theta.tolist()) @ e_x

            # Ensure joint angles stay within limits
            theta = np.clip(theta,
                            [lim[0] for lim in self.joint_limits],
                            [lim[1] for lim in self.joint_limits])

        return theta.tolist()
    

    def calc_velocity_kinematics(self, joint_values: list, vel: list, dt=0.02):
        """
        Calculates the velocity kinematics for the robot based on the given velocity input.

        Args:
            vel (list): The velocity vector for the end effector [vx, vy].
        """
        new_joint_values = joint_values.copy()


        # move robot slightly out of zeros singularity
        if all(theta == 0.0 for theta in new_joint_values):
            new_joint_values = [theta + np.random.rand()*0.02 for theta in new_joint_values]
        
        # Calculate joint velocities using the inverse Jacobian
        vel = vel[:2]  # Consider only the first two components of the velocity
        joint_vel = self.inverse_jacobian(new_joint_values) @ vel
        
        joint_vel = np.clip(joint_vel, 
                            [limit[0] for limit in self.joint_vel_limits], 
                            [limit[1] for limit in self.joint_vel_limits]
                        )


        # Update the joint angles based on the velocity
        for i in range(self.num_dof):
            new_joint_values[i] += dt * joint_vel[i]


        # Ensure joint angles stay within limits
        new_joint_values = np.clip(new_joint_values, 
                               [limit[0] for limit in self.joint_limits], 
                               [limit[1] for limit in self.joint_limits]
                            )
        
        return new_joint_values


    def jacobian(self, joint_values: list):
        """
        Returns the Jacobian matrix for the robot. 

        Args:
            joint_values (list): The joint angles for the robot.

        Returns:
            np.ndarray: The Jacobian matrix (2x2).
        """
        
        return np.array([
            [-self.l1 * sin(joint_values[0]) - self.l2 * sin(joint_values[0] + joint_values[1]), 
             -self.l2 * sin(joint_values[0] + joint_values[1])],
            [self.l1 * cos(joint_values[0]) + self.l2 * cos(joint_values[0] + joint_values[1]), 
             self.l2 * cos(joint_values[0] + joint_values[1])]
        ])
    

    def inverse_jacobian(self, joint_values: list):
        """
        Returns the inverse of the Jacobian matrix.

        Returns:
            np.ndarray: The inverse Jacobian matrix.
        """
        return np.linalg.pinv(self.jacobian(joint_values))




if __name__ == "__main__":
    model = TwoDOFRobot()
    robot = RobotSim(robot_model=model)
    viz = Visualizer(robot=robot)
    viz.run()
