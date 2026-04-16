"""
hiwonder.py

Hiwonder Robot Controller
-------------------------
This module provides a unified Python interface for controlling a Hiwonder
mobile base + 6-servo arm (5-DOF + gripper) across two hardware/firmware
variants:

- v5: uses `ros_robot_controller_sdk.Board` + bus servo API
- v36: uses `BoardController` + `setServoPulse/getServoPulse`

Key ideas for students:
- The robot state (joint angles) is read in a background thread.
- Gamepad monitoring runs in another background thread.
- Commands to hardware are protected by locks to avoid concurrent access.
- Angles are expressed in *degrees* by default, with an optional radians mode.
"""

import sys, os
import time
import numpy as np
import threading

from funrobo_hiwonder.core.gamepad_control import GamepadControl

# v5 drivers
from funrobo_hiwonder.drivers.v5.ros_robot_controller_sdk import Board
from funrobo_hiwonder.drivers.v5.bus_servo_control import *

# v36 drivers
from funrobo_hiwonder.drivers.v36.board_controller import BoardController
from funrobo_hiwonder.drivers.v36.servo_cmd import *



def detect_version() -> str:
    """
    Detect which Hiwonder controller/firmware stack is present.

    Returns:
        str: Either "v5" or "v36".

    How it works:
        Some Hiwonder images expose the device node `/dev/rrc` for the v5 stack.
        If that path exists, we assume v5, otherwise we assume v36.
    """
    return "v5" if os.path.exists("/dev/rrc") else "v36"


class BaseRobot():
    """
    Hardware-agnostic robot interface.

    This base class defines shared behavior for:
    - storing joint values and limits
    - conversion utilities (angle <-> pulse)
    - thread management for reading joint state and monitoring the gamepad
    - a consistent API that both RobotV5 and RobotV36 implement

    """

    def __init__(self) -> None:
        # --- Arm state ---
        self.joint_values = [0.0] * 6  # degrees in SOFTWARE joint order
        self.joint_limits = [
            [-120, 120], [-90, 90], [-120, 120],
            [-100, 100], [-90, 90], [-120, 30]
        ]

        # --- Mobile base geometry (meters) ---
        self.wheel_radius = 0.047
        self.base_length_x = 0.096
        self.base_length_y = 0.105

        # --- Convenience poses / gripper behavior ---
        self.home_position = [0, 0, 90, -30, 0, 0]  # degrees (software order)
        self.open_gripper_angle = -110              # degrees
        self.close_gripper_angle = 0                # degrees
        self.gripper_duration = 250                 # ms

        # --- Background thread control ---
        self.read_hz = 5
        self.shutdown_event = threading.Event()
        self.read_error = None

        # joint_values is shared between threads, so protect it
        self.joint_lock = threading.Lock()

        # Start joint-reader thread
        self.read_thread = threading.Thread(target=self.read_joint_values, daemon=True)
        self.read_thread.start()

        # Start gamepad thread (reads joystick events and updates GamepadControl state)
        self.gamepad = GamepadControl()
        self.gamepad_thread = threading.Thread(target=self.gamepad.monitor_gamepad, daemon=True)
        self.gamepad_thread.start()

        # Run startup motion / any subclass init behavior
        self.initialize_robot()

    
    def set_wheel_speeds(self, speedlist: list) -> None:
        """
        Command the 4 wheel speeds on the mobile base.

        Args:
            speedlist (list): Typically a length-4 list with wheel speeds.
                Units are hardware-specific (often a scaled rad/s or percent).
        """
        return
    

    def set_joint_values(self, joint_values: list, duration=1, radians=False) -> None:
        """
        Command all 6 joints to target angles.

        Args:
            joint_values (list): Length-6 target joint angles (software order).
            duration (float): Movement time (seconds for v5, converted to ms for v36).
            radians (bool): If True, interpret joint_values as radians.
        """
        return
    

    def set_joint_value(self, joint_value: float, duration=1, radians=False) -> None:
        pass


    def read_joint_values(self) -> None:
        """
        Background thread: continuously read joint state from hardware.

        Expected behavior:
        - Poll each joint.
        - Convert pulses to angles.
        - Update self.joint_values in a thread-safe way.
        - On repeated failure, set self.read_error and trigger shutdown_event.
        """
        pass


    def read_joint_value(self, joint_id: int):
        """
        Read a single joint "position" from hardware.

        Args:
            joint_id (int): 1-based servo ID in *hardware* addressing.

        Returns:
            A hardware-dependent value (often a pulse count),
            or None if the read failed.
        """
        pass


    def get_joint_values(self) -> list:
        """
        Thread-safe getter for joint angles.

        Returns:
            list: Joint angles (degrees) in software joint order.
        """
        with self.joint_lock:
            return self.joint_values.copy()


    def enforce_joint_limits(self, joint_values: list) -> list:
        """
        Clamp joint angles within hardware limits.

        Args:
            joint_values (list): Desired joint angles (degrees).

        Returns:
            list: Clamped joint angles (degrees).
        """
        return [np.clip(theta, *limit) for theta, limit in zip(joint_values, self.joint_limits)]
  

    def initialize_robot(self) -> None:
        """
        Run startup behavior after threads/hardware are initialized.

        Current behavior:
        - brief delay for threads/hardware to settle
        - move arm to home position
        - print readiness message
        """
        time.sleep(1)
        self.move_to_home_position()
        time.sleep(1)
        print('------------------- System is now ready!-------------------\n')


    def move_to_home_position(self) -> None:
        """
        Move arm to a known "home" posture (self.home_position).

        Teaching note:
            Home posture is useful for:
            - calibration checks
            - safe startup
            - repeatable demos
        """
        print('Moving to home position...')
        self.set_joint_values(self.home_position, duration=1)
        time.sleep(2.0)

        print(f'Arrived at home position: {self.home_position}\n')

        with self.joint_lock:
            self.joint_values = self.home_position.copy()

        time.sleep(1.0)


    def angle_to_pulse(self, x: float) -> int:
        """
        Convert an angle (degrees) to a servo pulse value.

        Notes:
            This assumes a linear mapping between joint angle range
            and hardware pulse range [0, 1000].

        Args:
            x (float): Joint angle in degrees.

        Returns:
            int: Pulse value (0..1000).
        """
        hw_min, hw_max = 0, 1000
        joint_min, joint_max = -120, 120
        return int((x - joint_min) * (hw_max - hw_min) / (joint_max - joint_min) + hw_min)


    def pulse_to_angle(self, x: float) -> float:
        """
        Convert a servo pulse value to an angle (degrees).

        Args:
            x (float): Pulse value (0..1000).

        Returns:
            float: Angle in degrees.
        """
        hw_min, hw_max = 0, 1000
        joint_min, joint_max = -120, 120
        return round((x - hw_min) * (joint_max - joint_min) / (hw_max - hw_min) + joint_min, 2)


    def remap_joints(self, joint_values: list) -> list:
        """
        Remap a list of 6 joint values between software order and hardware order.

        In this codebase, the "software joint order" is reversed relative to the
        "hardware servo ID order", so we map by reversing the list.

        Args:
            joint_values (list): Length-6 list in one ordering.

        Returns:
            list: Length-6 list in the opposite ordering.

        Note:
            HARDWARE servo IDs (1..6) correspond to a different physical ordering
            than the software convention. Here we use a simple reverse mapping:
            `joint_values[::-1]`.
        """
        return joint_values[::-1]



class RobotV5(BaseRobot):
    """
    Implementation for the v5 controller stack.

    Uses:
        - Board() from ros_robot_controller_sdk
        - bus servo read/write functions over the board interface

    Threading:
        - board_lock protects hardware access
        - joint_lock protects shared state (joint_values)
    """

    def __init__(self) -> None:
        self.board = Board()
        self.board.enable_reception()
        self.board_lock = threading.Lock()
        super().__init__()
 
 
    def set_wheel_speeds(self, speedlist: list) -> None:
        """
        Send wheel speed commands to the v5 motor controller.

        motor3 w0|  ↑  |w1 motor1
                 |     |
        motor4 w2|     |w3 motor2

        Args:
            speedlist (list): Length-4 list of wheel speeds (typically normalized).
        """
        wheel_speeds = []
        for wheel_id, speed in enumerate(speedlist, start=1):
            wheel_speeds.append([wheel_id, speed * 8.4])

        with self.board_lock:
            self.board.set_motor_speed(wheel_speeds)
        

    def set_joint_values(self, joint_values: list, duration=1, radians=False) -> None:
        """
        Move all 6 joints to target angles on v5 hardware.

        Args:
            joint_values (list): Length-6 target angles (degrees unless radians=True).
            duration (float): Movement duration in seconds.
            radians (bool): If True, joint_values is interpreted as radians.
        """
        if len(joint_values) != 6:
            raise ValueError("Provide 6 joint angles.")

        if radians:
            joint_values = [np.rad2deg(theta) for theta in joint_values]

        joint_values = self.enforce_joint_limits(joint_values)
        # self.joint_values = joint_values.copy() # updates joint_values with commanded thetalist
        joint_values_hw = self.remap_joints(joint_values)

        positions = []
        for joint_id, theta in enumerate(joint_values_hw, start=1):
            positions.append([joint_id, self.angle_to_pulse(theta)])

        with self.board_lock:
            self.board.bus_servo_set_position(duration, positions)


    def read_joint_values(self) -> None:
        """
        Background loop: read all joint pulse values from v5 hardware,
        convert to degrees, and update `self.joint_values`.

        Failure behavior:
            - If an exception occurs, store it in self.read_error and signal shutdown_event.
        """
        try:
            while not self.shutdown_event.is_set():
                t0 = time.time()

                raw = [self.read_joint_value(i + 1) for i in range(len(self.joint_values))]
                raw = self.remap_joints(raw)

                with self.joint_lock:
                    prev = self.joint_values.copy()

                # Update angles (keep previous on read failure)
                new_angles = prev
                for i in range(len(raw)):
                    if raw[i] is None:
                        new_angles[i] = prev[i]
                    else:
                        new_angles[i] = self.pulse_to_angle(raw[i][0])

                with self.joint_lock:
                    self.joint_values = new_angles

                time.sleep(1 / self.read_hz)

                dt = time.time() - t0
                if dt > (10 / self.read_hz):
                    print("[WARN] reader slow:", dt, "raw:", raw)
                    raise RuntimeError("Read thread is damaged... restart!")

        except Exception as e:
            self.read_error = e
            self.shutdown_event.set()


    def read_joint_value(self, joint_id: int):
        """
        Read a single servo position from v5 hardware.

        Args:
            joint_id (int): 1-based hardware servo ID.

        Returns:
            Usually a tuple/list from the SDK containing the pulse value,
            or None if the read repeatedly fails.
        """
        max_count = 20
        for _ in range(max_count):
            with self.board_lock:
                res = self.board.bus_servo_read_position(joint_id)
            if res is not None:
                return res
            time.sleep(0.01)
        return None


    def open_gripper(self) -> None:
        """
        Open the gripper on v5 hardware.
        We command all joints to hold their current values, but replace
        the gripper joint with `self.open_gripper_angle`.

        """
        joint_values = self.get_joint_values()
        joint_values_hw = self.remap_joints(joint_values)

        positions = []
        for joint_id, theta in enumerate(joint_values_hw, start=1):
            if joint_id == 1:  
                pulse = self.angle_to_pulse(self.open_gripper_angle)
            else:
                pulse = self.angle_to_pulse(theta)
            positions.append([joint_id, pulse])

        with self.board_lock:
            self.board.bus_servo_set_position(self.gripper_duration, positions)


    def close_gripper(self) -> None:
        """
        Close the gripper on v5 hardware.
        """
        joint_values = self.get_joint_values()
        joint_values_hw = self.remap_joints(joint_values)

        positions = []
        for joint_id, theta in enumerate(joint_values_hw, start=1):
            if joint_id == 1:
                pulse = self.angle_to_pulse(self.close_gripper_angle)
            else:
                pulse = self.angle_to_pulse(theta)
            positions.append([joint_id, pulse])

        with self.board_lock:
            self.board.bus_servo_set_position(self.gripper_duration, positions)


    def disable_servos(self) -> None:
        """
        Disable torque on all servos (v5).
        """
        with self.board_lock:
            for joint_id in range(1, len(self.joint_values) + 1):
                self.board.bus_servo_enable_torque(joint_id, 0)


    def shutdown_robot(self) -> None:
        """
        Safely shut down v5 hardware:
        - move to home position
        - disable torque
        - stop background threads
        - close the hardware port
        """
        print("\n[INFO] Shutting down the robot safely...")

        self.set_joint_values(self.home_position, duration=2)
        time.sleep(1.5)

        self.disable_servos()
        self.shutdown_event.set()

        print("[INFO] Closing hardware interfaces...")
        with self.board_lock:
            try:
                self.board.enable_reception(False)
            except Exception:
                pass
            try:
                self.board.port.close()
            except Exception:
                pass

        print("[INFO] Shutdown complete. Safe to power off.")



class RobotV36(BaseRobot):
    """
    Implementation for the v36 controller stack.
    """

    def __init__(self) -> None:
        self.motor_board = BoardController()
        super().__init__()


    def set_wheel_speeds(self, speedlist: list) -> None:
        """
        Send wheel speed commands to the v36 motor controller.

        Args:
            speedlist (list): Length-4 list of wheel speeds (hardware units).
        """
        self.motor_board.set_motor_speed(speedlist)


    def set_joint_values(self, joint_values: list, duration=1, radians=False) -> None:
        """
        Move all 6 joints to target angles on v36 hardware.

        Args:
            joint_values (list): Length-6 target angles (degrees unless radians=True).
            duration (float): Movement duration in seconds (converted to ms).
            radians (bool): If True, interpret joint_values as radians.
        """
        if len(joint_values) != 6:
            raise ValueError("Provide 6 joint angles.")

        if radians:
            joint_values = [np.rad2deg(theta) for theta in joint_values]

        joint_values = self.enforce_joint_limits(joint_values)
        self.joint_values = joint_values.copy() # updates joint_values with commanded thetalist
        joint_values_hw = self.remap_joints(joint_values)

        for joint_id, theta in enumerate(joint_values_hw, start=1):
            pulse = self.angle_to_pulse(theta)
            setServoPulse(joint_id, pulse, int(duration * 1000))
    
    
    def read_joint_value(self, joint_id: int):
        """
        Read a single servo pulse value from v36 hardware.

        Args:
            joint_id (int): 1-based hardware servo ID.

        Returns:
            int|None: pulse value if successful, otherwise None.
        """
        #TODO Fix read error issue
        # max_count = 20
        # for _ in range(max_count):
        #     res = getServoPulse(joint_id)
        #     if res is not None:
        #         return res
        #     time.sleep(0.01)
        return None
            
    
    def read_joint_values(self) -> None:
        """
        Background loop: read servo pulses from v36 hardware, convert to degrees,
        and update `self.joint_values`.
        """
        try:
            while not self.shutdown_event.is_set():
                t0 = time.time()

                raw = [self.read_joint_value(i + 1) for i in range(len(self.joint_values))]
                raw = self.remap_joints(raw)

                with self.joint_lock:
                    prev = self.joint_values.copy()

                new_angles = prev
                for i in range(len(raw)):
                    if raw[i] is None:
                        new_angles[i] = prev[i]
                    else:
                        new_angles[i] = self.pulse_to_angle(raw[i])

                with self.joint_lock:
                    self.joint_values = new_angles

                time.sleep(1 / self.read_hz)

                dt = time.time() - t0
                if dt > (10 / self.read_hz):
                    print("[WARN] reader slow:", dt, "raw:", raw)
                    raise RuntimeError("Read thread is damaged... restart!")

        except Exception as e:
            self.read_error = e
            self.shutdown_event.set()

    
    def get_joint_values(self) -> list:
        """
        Thread-safe getter for joint angles (v36).

        Returns:
            list: Joint angles (degrees) in software joint order.
        """
        with self.joint_lock:
            return self.joint_values.copy()


    def open_gripper(self) -> None:
        """
        Open the gripper on v36 hardware.
        """
        open_pulse = self.angle_to_pulse(self.open_gripper_angle)
        setServoPulse(1, open_pulse, int(self.gripper_duration))


    def close_gripper(self) -> None:
        """
        Close the gripper on v36 hardware.
        """
        close_pulse = self.angle_to_pulse(self.close_gripper_angle)
        setServoPulse(1, close_pulse, int(self.gripper_duration))

    
    def shutdown_robot(self) -> None:
        """
        Safely shut down v36 hardware.

        Current behavior:
            - Move to home
            - Print status messages

        Note:
            You may want to stop motors / disable torque depending on your lab safety needs.
        """
        print("\n[INFO] Shutting down the robot safely...")
        self.set_joint_values(self.home_position, duration=2)
        time.sleep(1.5)
        print("[INFO] Shutdown complete. Safe to power off.")


    def stop_motors(self):
        """ Stops all motors safely """
        self.motor_board.set_motor_speed([0]*4)
        print("[INFO] Motors stopped.")



# Alias that selects the correct class for the current hardware.
HiwonderRobot = RobotV5 if detect_version() == "v5" else RobotV36
