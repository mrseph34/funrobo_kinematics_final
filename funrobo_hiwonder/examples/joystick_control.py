"""
joystick_control.py

Gamepad teleoperation for Hiwonder mobile manipulator.

This script shows a simple fixed-rate control loop (20 Hz) that:
1) Reads the latest gamepad command from the robot's GamepadControl thread
2) Integrates joystick inputs into incremental arm joint commands
3) Converts desired chassis velocities (vx, vy, w) into individual wheel speeds
4) Sends arm + base commands to the robot

"""

import time
import traceback

from funrobo_hiwonder.core.hiwonder import HiwonderRobot



def joystick_control(robot, dt, joint_values):
    """
    Apply joystick commands to control both the arm and the mobile base.

    This function reads the most recent gamepad command and performs two tasks: arm control
    and base control

    Args:
        robot: Instance of HiwonderRobot (RobotV5 or RobotV36).
        dt (float): Control timestep in seconds.
        joint_values (list[float]): Length-6 list of joint angle targets (degrees),
            maintained across loop iterations.

    Returns:
        list[float]: Updated joint_targets (degrees), to be fed back into the next call.

    Relevant notes:
        - `cmd.arm_j1 ... cmd.arm_ee` are assumed to be in [-1, 1].
    """

    cmd = robot.gamepad.cmdlist[-1]

    # ----------------------------------------------------------------------
    # Arm joint control
    # ----------------------------------------------------------------------

    max_rate = 400  # 400 x 0.1 = 40 deg/s
    joint_values[0] += dt * max_rate * cmd.arm_j1
    joint_values[1] += dt * max_rate * cmd.arm_j2
    joint_values[2] += dt * max_rate * cmd.arm_j3
    joint_values[3] += dt * max_rate * cmd.arm_j4
    joint_values[4] += dt * max_rate * cmd.arm_j5
    joint_values[5] += dt * max_rate * cmd.arm_ee

    new_joint_values = robot.enforce_joint_limits(joint_values)
    new_joint_values = [round(theta,3) for theta in new_joint_values]

    print(f'[DEBUG] Commanded joint angles: [j1, j2, j3, j4, j5, ee]: {new_joint_values}')
    print(f'-------------------------------------------------------------------------------------\n')    
    
    # set new joint angles
    robot.set_joint_values(new_joint_values, duration=dt, radians=False)

    # ----------------------------------------------------------------------
    # base veocity control
    # ----------------------------------------------------------------------

    """
    Omni/mecanum-style wheel mixing (typical form):

        w0 = (vx - vy - w*L) / R
        w1 = (vx + vy + w*L) / R
        w2 = (vx + vy - w*L) / R
        w3 = (vx - vy + w*L) / R

    Where:
        - vx: desired forward velocity (m/s)
        - vy: desired lateral velocity (m/s)
        - w: desired yaw rate (rad/s)
        - R: wheel radius (m)
        - L: effective half-length + half-width (m), here approximated as
             (base_length_x + base_length_y)
    """
    vx, vy, w = cmd.base_vx, cmd.base_vy, cmd.base_w

    # Compute wheel speeds
    w0 = (vx - vy - w * (robot.base_length_x + robot.base_length_y)) / robot.wheel_radius
    w1 = (vx + vy + w * (robot.base_length_x + robot.base_length_y)) / robot.wheel_radius
    w2 = (vx + vy - w * (robot.base_length_x + robot.base_length_y)) / robot.wheel_radius
    w3 = (vx - vy + w * (robot.base_length_x + robot.base_length_y)) / robot.wheel_radius

    robot.set_wheel_speeds([w0, w1, w2, w3])



def main():
    """
    Main loop: run joystick teleoperation at a fixed rate.

    The loop:
    - creates the robot object (which starts background threads)
    - checks whether the joint reader thread has crashed
    - runs joystick_control() if gamepad commands are available
    - sleeps to maintain a constant control rate (20 Hz)
    - shuts down safely on exit
    """
    robot = None

    try:
        robot = HiwonderRobot()
        
        control_hz = 20 
        dt = 1 / control_hz

        joint_values = robot.get_joint_values()

        while True:
            t_start = time.time()

            if robot.read_error is not None:
                print("[FATAL] Reader failed:", robot.read_error)
                break

            if robot.gamepad.cmdlist:
                joystick_control(robot, dt, joint_values)


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
        if robot is not None:
            robot.shutdown_robot()





if __name__ == "__main__":
    main()
