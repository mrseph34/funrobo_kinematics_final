# main.py
"""
Main Application Script
----------------------------
Coordinates gamepad input and robot control.
"""

import sys, os
import time
import traceback

# Extend system path to include script directory
sys.path.append(os.path.join(os.getcwd(), 'scripts'))

from funrobo_kinematics.core.arm_models import FiveDOFRobotTemplate
import funrobo_kinematics.core.utils as ut
from funrobo_hiwonder.core.hiwonder import HiwonderRobot


JOINT_WAYPOINTS = [[25.6, -25.6, 36.8, -70.5, 0.0, -100],
                   [25.6, -61.6, 37.8, -70.5, 0.0, -100],
                   [25.6, -61.6, 37.8, -70.5, 0.0, -10],
                   [25.6, -61.6, 36.8, -70.5, 0.0, -10],
                   [-40.6, -25.6, 36.8, -70.5, 0.0, -10],
                   [-40.6, -54.6, 36.8, -70.5, 0.0, -10],
                   [-40.6, -54.6, 36.8, -70.5, 0.0, -100],
                   [-40.6, -25.6, 36.8, -70.5, 0.0, -100]]

HOME_POSITION = [0, 0, 90, -30, 0, 0]

# Initialize components
robot = HiwonderRobot()
model = FiveDOFRobotTemplate()



class BlockPickerFSM():
    def __init__(self, dt=0.05):
        self.stage = 0
        self.t = 0.0
        self.dt = dt
        self.T = 3 # seconds

    def step(self):

        if self.stage == 0:
            start_pos = HOME_POSITION
            theta = []
            for i in range(len(start_pos)):
                theta.append(round((1 - self.t/self.T)*start_pos[i] + (self.t/self.T)*JOINT_WAYPOINTS[0][i],2))
            self.t += self.dt

            if (self.t-self.T)>self.dt:
                self.stage = 1
                self.t = 0.0

        elif self.stage == 8:
            start_pos = JOINT_WAYPOINTS[self.stage-1]
            theta = []
            for i in range(len(start_pos)):
                theta.append(round((1 - self.t/self.T)*start_pos[i] + (self.t/self.T)*HOME_POSITION[i],2))
            self.t += self.dt

            if (self.t-self.T)>self.dt:
                self.stage = 0
                self.t = 0.0

        else:
            start_pos = JOINT_WAYPOINTS[self.stage-1]
            theta = []
            for i in range(len(start_pos)):
                theta.append(round((1 - self.t/self.T)*start_pos[i] + (self.t/self.T)*JOINT_WAYPOINTS[self.stage][i],2))
            self.t += self.dt

            if (self.t-self.T)>self.dt:
                self.stage += 1
                self.t = 0.0

        print(f'Stage: {self.stage} theta: {theta} t: {round(self.t,3)})')
        robot.set_joint_values(theta, duration=self.dt, radians=False)





def main():
    """ Main loop that reads gamepad commands and updates the robot accordingly. """
    try:
       
        control_hz = 20 
        dt = 1 / control_hz
        t0 = time.time()

        fsm = BlockPickerFSM(dt=dt)

        while True:
            t_start = time.time()

            if robot.read_error is not None:
                print("[FATAL] Reader failed:", robot.read_error)
                break

            if robot.gamepad.cmdlist:
                fsm.step()


            elapsed = time.time() - t_start
            remaining_time = dt - elapsed
            if remaining_time > 0:
                time.sleep(remaining_time)

            
    except KeyboardInterrupt:
        print("\n[INFO] Keyboard Interrupt detected. Initiating shutdown...")
    except Exception as e:
        print(f"[ERROR] Unexpected error: {e}")
        traceback.print_exc()
    finally:
        robot.shutdown_robot()




if __name__ == "__main__":
    main()


