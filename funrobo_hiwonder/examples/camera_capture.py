# main.py
"""
Main Application Script
----------------------------
Coordinates gamepad input and robot control.
"""


import time
import traceback
import cv2

from funrobo_hiwonder.core.hiwonder import HiwonderRobot



class CameraExampleFSM:
    """
    A minimal "FSM-style" controller that combines:
    - Joystick-driven arm motion (incremental joint updates)
    - Periodic camera capture

    Args:
        dt (float): Control time step in seconds (e.g., 0.05 for 20 Hz).
        robot: An instance of HiwonderRobot (RobotV5 or RobotV36).
    """
    def __init__(self, dt: float = 0.05, robot=None) -> None:
        if robot is None:
            raise ValueError("CameraExampleFSM requires a robot instance.")

        self.t = 0.0
        self.dt = dt
        self.robot = robot

        self.new_joint_values = robot.get_joint_values()

        # Open default camera (index 0). On some systems you may need 1, 2, etc.
        self.camera = cv2.VideoCapture(0)

        if not self.camera.isOpened():
            print("[WARN] Could not open camera 0. Image capture will fail.")


    def close(self) -> None:
        """
        Release resources used by this controller.
        """
        try:
            if self.camera is not None:
                self.camera.release()
        except Exception:
            pass


    def js_control(self):
        cmd = self.robot.gamepad.cmdlist[-1]

        max_rate = 400  # 400 x 0.1 = 40 deg/s
        self.new_joint_values[0] += self.dt * max_rate * cmd.arm_j1
        self.new_joint_values[1] += self.dt * max_rate * cmd.arm_j2
        self.new_joint_values[2] += self.dt * max_rate * cmd.arm_j3
        self.new_joint_values[3] += self.dt * max_rate * cmd.arm_j4
        self.new_joint_values[4] += self.dt * max_rate * cmd.arm_j5
        self.new_joint_values[5] += self.dt * max_rate * cmd.arm_ee

        self.new_joint_values = self.robot.enforce_joint_limits(self.new_joint_values)
        self.new_joint_values = [round(theta, 3) for theta in self.new_joint_values]

        self.robot.set_joint_values(self.new_joint_values, duration=self.dt, radians=False)


    def process_image(self):
        ret, frame = self.camera.read()
        self.t = 0.0 # reset timer

        if ret:
            print("Saving to frame.png")
            cv2.imwrite("frame.png", frame)
        else:
            print("Failed to capture frame")


    def step(self) -> None:
        """
        Perform one control cycle:
        - Periodically capture and save an image (every 0.5 s)
        - Update robot arm joint targets from joystick input
        - Advance internal timer
        """
        # Periodic behavior: capture frame every 0.5 seconds
        if self.t >= 0.5:
            self.process_image()

        # Continuous behavior: joystick control every step
        self.js_control()

        # Update timer
        self.t += self.dt



def main() -> None:
    """
    Runs a fixed-rate (20 Hz) loop:
    - checks the robot's reader thread for errors
    - runs the FSM step if there is at least one gamepad command
    - sleeps to maintain the target loop period
    - performs safe shutdown on exit
    """
    robot = None
    fsm = None

    try:
        robot = HiwonderRobot()

        control_hz = 20
        dt = 1 / control_hz

        fsm = CameraExampleFSM(dt=dt, robot=robot)

        while True:
            t_start = time.time()

            # If the background read thread died, exit safely.
            if robot.read_error is not None:
                print("[FATAL] Reader failed:", robot.read_error)
                break

            # Only run control if we have at least one command.
            if robot.gamepad.cmdlist:
                fsm.step()

            # Maintain fixed loop rate
            elapsed = time.time() - t_start
            remaining_time = dt - elapsed
            if remaining_time > 0:
                time.sleep(remaining_time)

            
    except KeyboardInterrupt:
        print("\n[INFO] KeyboardInterrupt detected. Initiating shutdown...")
    except Exception as e:
        print(f"[ERROR] Unexpected error: {e}")
        traceback.print_exc()
    finally:
        # Release camera resources first (if created)
        if fsm is not None:
            fsm.close()
        # Then shutdown robot hardware (if created)
        if robot is not None:
            robot.shutdown_robot()




if __name__ == "__main__":
    main()


