"""
GUI-based robot manipulator visualizer for the FunRobo kinematics library.

This module provides two main classes:

- Visualizer:
    A Tkinter GUI for interacting with a robot simulation:
    - Forward kinematics (set joint values)
    - Inverse kinematics (set end-effector pose)
    - Velocity kinematics (keyboard-controlled end-effector velocity)
    - Simple waypoint loading and trajectory demo hooks

- RobotSim:
    A lightweight simulation/visualization wrapper around a robot "model" object.
    It uses Matplotlib (embedded in Tkinter) to render joint positions, links, and
    coordinate frames.

Expected robot model interface
------------------------------
RobotSim expects `robot_model` to provide at least:
    - num_dof (int)
    - joint_values (list[float])
    - ee (EndEffector)
    - points (list[np.ndarray])
    - EE_axes (np.ndarray shape (3, 3) or similar)
    - calc_forward_kinematics(joint_values, radians=True/False) -> (EndEffector, list[np.ndarray])
    - calc_inverse_kinematics(ee, joint_values, soln=0) -> list[float]
    - calc_numerical_ik(ee, joint_values, tol=..., ilimit=...) -> list[float]
    - calc_velocity_kinematics(joint_values, vel, dt=...) -> list[float]
    - calc_robot_points(joint_values, Hlist, radians=True/False) -> None

Conventions:
- Lengths are meters.
- Angles are radians internally; the GUI often accepts degrees for convenience.
- FK returns a list of **individual link transforms** (one per joint), not cumulative transforms.

Notes for teaching:
- This module is intended for interactive exploration, not real-time control.
- Some features (trajectory generation) assume external classes/functions exist.

"""

import math
import time
import os, sys
from typing import List, Tuple

import numpy as np
import tkinter as tk
from tkinter import ttk

from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure

import yaml
from pynput import keyboard

from funrobo_kinematics.core.utils import EndEffector, wraptopi
from funrobo_kinematics.core.path_planner import RobotPathPlanner
from funrobo_kinematics.core.trajectory_generator import MultiAxisTrajectoryGenerator, MultiSegmentTrajectoryGenerator

class Visualizer:
    """
    Tkinter GUI for visualizing and controlling a robot manipulator.

    The GUI embeds a Matplotlib figure and provides panels for:
    - Forward kinematics: set joint values via entries or sliders (degrees shown)
    - Inverse kinematics: set a desired end-effector pose and solve IK
    - Velocity kinematics: toggle keyboard-controlled velocity motion
    - Waypoints / trajectory demos (optional hooks)

    Args:
        robot: A RobotSim-like object that owns a Matplotlib Figure and exposes
            methods such as `update_plot(...)` and `move_velocity(...)`.

    Attributes:
        robot: Robot simulation/visualization wrapper.
        root: Tkinter root window owned by this Visualizer.
        canvas: Tkinter-embedded Matplotlib canvas.
        vk_status: Whether velocity-control mode is active.
        v: Current commanded Cartesian velocity [vx, vy, vz] used in VK mode.
        listener: Background keyboard listener used for VK mode.
    """

    def __init__(self, robot) -> None:
        """
         Initialize the Visualizer and build the GUI.

        This creates the Tkinter root window, starts the keyboard listener
        for velocity control, and builds the GUI layout.

        Args:
            robot: A robot simulation/visualization object (e.g., RobotSim).
        """
        self.robot = robot

        # Create tk root
        self.root = tk.Tk()
        title = f"Robot Manipulator Visualization ({self.robot.num_joints}-DOF)"
        self.root.title(title)

        print(
            f"\nInitialized the Robot Manipulator Visualization for the "
            f"{self.robot.num_joints}-DOF robot for Kinematics Analysis\n"
        )
        
        # Variables for velocity kinematics
        self.vk_status = False
        self.v = [0.0, 0.0, 0.0]  # [vx, vy, vz]
        self.vk_status_font = ('black')

        # Keyboard listener for velocity kinematics
        self.listener = keyboard.Listener(
            on_press=self.on_press,
            on_release=self.on_release
        )
        self.listener.start()

        # Build the GUI
        self._build_layout()

    
    def _build_layout(self) -> None:
        """
        Build the main GUI layout.

        Creates:
        - Left: control panel frame (Tk widgets)
        - Right: plot frame embedding the robot's Matplotlib figure
        """
        # Create the control frame for the GUI
        self.control_frame = ttk.Frame(self.root)
        self.control_frame.grid(row=0, column=0, padx=15, pady=15)
        
        # Create the plot frame for the GUI
        self.plot_frame = ttk.Frame(self.root)
        self.plot_frame.grid(row=0, column=1, padx=10, pady=10)

        # Embed the robot's figure into the Tkinter canvas
        self.canvas = FigureCanvasTkAgg(self.robot.fig, master=self.plot_frame)
        self.canvas.draw()
        self.canvas.get_tk_widget().grid(row=0, column=0)

        # Set up the kinematics panel
        self.set_kinematics_panel()

        
    def set_kinematics_panel(self) -> None:
        """
        Create the kinematics control panel widgets.

        Panels include:
        - Forward Kinematics: joint entries + sliders + reset
        - Inverse Kinematics: end-effector pose entries + solve buttons
        - Velocity Kinematics: activate/deactivate buttons
        - Trajectory Generation: waypoint upload + demo generation buttons

        Returns:
            None. Widgets are created and stored as instance attributes.
        """
        # ------------------------------------------------------------------------------------------------
        # Forward position kinematics
        # ------------------------------------------------------------------------------------------------
        self.joint_values = []
        row_number = 0

        # Add title for the forward kinematics entry field
        self.fk_entry_title = ttk.Label(self.control_frame, text="Forward Kinematics:", font=("Arial", 13, "bold"))
        self.fk_entry_title.grid(column=0, row=row_number, columnspan=2, pady=(0, 10))
        row_number += 1

        # Create joint entry fields and labels
        self.joint_button = []
        for i in range(self.robot.num_joints):
            joint_label = ttk.Label(self.control_frame, text=f"theta {i+1} (deg or m):")
            joint_label.grid(column=0, row=row_number, sticky=tk.W)
            joint_value = ttk.Entry(self.control_frame)
            joint_value.insert(0, "0")
            joint_value.grid(column=1, row=row_number)
            self.joint_button.append(joint_value)
            row_number += 1

        # Create the Move button
        self.fk_move_button = ttk.Button(self.control_frame, text="Move", command=self.joints_from_button)
        self.fk_move_button.grid(column=0, row=row_number, columnspan=2, pady=5)
        row_number += 1

        # Create the joint slider field and labels
        self.joint_scales = []
        for i in range(self.robot.num_joints):
            joint_label = ttk.Label(self.control_frame, text=f"theta {i+1} (deg or m):")
            joint_label.grid(column=0, row=row_number, sticky=tk.W)

            joint_value = tk.DoubleVar()
            slider = ttk.Scale(
                self.control_frame, 
                from_=-180, 
                to=180, 
                variable=joint_value, 
                command=self.joints_from_sliders
            )
            slider.grid(column=1, row=row_number)
            row_number += 1
            self.joint_scales.append(joint_value)

        # Create the Reset button
        self.fk_reset_button = ttk.Button(
            self.control_frame, 
            text="Reset", 
            command=self.reset_joints
        )
        self.fk_reset_button.grid(column=0, row=row_number, columnspan=2, pady=5)
        row_number += 3

        # ------------------------------------------------------------------------------------------------
        # Inverse position kinematics
        # ------------------------------------------------------------------------------------------------
        self.ik_entry_title = ttk.Label(
            self.control_frame, 
            text="Inverse Kinematics:", 
            font=("Arial", 13, "bold")
        )
        self.ik_entry_title.grid(column=0, row=row_number, columnspan=2, pady=(0, 10))
        row_number += 1

        # Create end-effector pose field and labels
        self.pose_button = []
        pose_labels = ['X(m)', 'Y(m)', 'Z(m)', 'RotX(rad)', 'RotY(rad)', 'RotZ(rad)']
        for i in range(len(pose_labels)):
            position_label = ttk.Label(self.control_frame, text=pose_labels[i] + ":")
            position_label.grid(column=0, row=row_number, sticky=tk.W)
            position_value = ttk.Entry(self.control_frame)
            position_value.insert(0, "0")
            position_value.grid(column=1, row=row_number)
            row_number += 1
            self.pose_button.append(position_value)

        # Create buttons for inverse kinematics solutions
        self.ik1_move_button = ttk.Button(
            self.control_frame, 
            text="Solve 1", 
            command=self.solve_IK1
        )
        self.ik1_move_button.grid(column=0, row=row_number, columnspan=1, pady=2)

        self.ik2_move_button = ttk.Button(
            self.control_frame, 
            text="Solve 2", 
            command=self.solve_IK2
        )
        self.ik2_move_button.grid(column=1, row=row_number, columnspan=1, pady=2)

        self.ik3_move_button = ttk.Button(
            self.control_frame, 
            text="Num Solve", 
            command=self.numerical_solve
        )
        self.ik3_move_button.grid(column=2, row=row_number, columnspan=1, pady=2)

        self.ik_set_pose_button = ttk.Button(
            self.control_frame,
            text="Set Pose",
            command=self.load_current_pose
        )
        self.ik_set_pose_button.grid(column=1, row=row_number+1, columnspan=1, pady=1)
        row_number += 5
        

        # ------------------------------------------------------------------------------------------------
        # Path Planning
        # ------------------------------------------------------------------------------------------------
        self.mp_entry_title = ttk.Label(self.control_frame, text="Path Planning:", font=("Arial", 13, "bold"))
        self.mp_entry_title.grid(column=0, row=row_number, columnspan=2, pady=(0, 10))
        row_number += 1

        # Create start and goal configuration entry fields and labels
        init_config = [-70, 60, -30, 80, 0]#[0] * self.robot.num_joints
        start_label = ttk.Label(self.control_frame, text=f"Start configuration (deg):")
        start_label.grid(column=0, row=row_number, sticky=tk.W)
        start_config_value = ttk.Entry(self.control_frame)
        start_config_value.insert(0, str(init_config))
        start_config_value.grid(column=1, row=row_number)
        self.start_config = start_config_value
        row_number += 1

        goal_label = ttk.Label(self.control_frame, text=f"Goal configuration (deg):")
        goal_label.grid(column=0, row=row_number, sticky=tk.W)
        goal_config_value = ttk.Entry(self.control_frame)
        goal_config_value.insert(0, str([60, 60, -30, 90, 0]))#str(init_config))
        goal_config_value.grid(column=1, row=row_number)
        self.goal_config = goal_config_value
        row_number += 1

        self.path_generate_button = ttk.Button(self.control_frame, text="Generate Path", command=self.generate_path)
        self.path_generate_button.grid(column=0, row=row_number, columnspan=1, pady=2)

        self.path_clear_button = ttk.Button(self.control_frame, text="Clear Path", command=self.clear_path)
        self.path_clear_button.grid(column=1, row=row_number, columnspan=1, pady=2)

        self.path_obstacle_button = ttk.Button(self.control_frame, text="Toggle Obstacles", command=self.toggle_obstacles)
        self.path_obstacle_button.grid(column=2, row=row_number, columnspan=1, pady=2)
        row_number += 1

        self.path_follow_button = ttk.Button(self.control_frame, text="Follow Path", command=self.follow_path)
        self.path_follow_button.grid(column=1, row=row_number, columnspan=1, pady=2)
        row_number += 1

        # # ------------------------------------------------------------------------------------------------
        # # Trajectory generation
        # # ------------------------------------------------------------------------------------------------
        self.mp_entry_title = ttk.Label(self.control_frame, text="Trajectory Generation:", font=("Arial", 13, "bold"))
        self.mp_entry_title.grid(column=0, row=row_number, columnspan=2, pady=(0, 10))
        row_number += 1

        # traj_method_label = ttk.Label(self.control_frame, text=f"Select method (linear, cubic, etc.):")
        # traj_method_label.grid(column=0, row=row_number, sticky=tk.W)
        # traj_method_value = ttk.Entry(self.control_frame)
        # traj_method_value.insert(0, "")
        # traj_method_value.grid(column=1, row=row_number)
        # row_number += 1

        self.mp_follow_task_button = ttk.Button(self.control_frame, text="Generate Traj (Task-space)", command=self.generate_traj_task_space)
        self.mp_follow_task_button.grid(column=0, row=row_number, columnspan=1, pady=2)

        self.mp_follow_joint_button = ttk.Button(self.control_frame, text="Generate Traj (Joint-space)", command=self.generate_traj_joint_space)
        self.mp_follow_joint_button.grid(column=1, row=row_number, columnspan=1, pady=2)
        row_number += 1


    def joints_from_sliders(self, val) -> None:
        """
        Updates the forward kinematics based on the joint angles set by the sliders.
        """
        joint_values = [float(th.get()) for th in self.joint_scales]
        self.update_FK(joint_values)


    def joints_from_button(self) -> None:
        """
        Updates the forward kinematics based on the joint angles entered in the input fields.
        """
        joint_values = [float(th.get()) for th in self.joint_button]
        self.update_FK(joint_values)


    def reset_joints(self) -> None:
        """
        Resets all joint angles to 0 and updates the forward kinematics.
        """
        joint_values = [0.0] * self.robot.num_joints
        self.robot.reset_ee_trajectory()

        # --- Reset sliders ---
        for var in self.joint_scales:
            var.set(0.0)

        # --- Reset entry boxes ---
        for entry in self.joint_button:
            entry.delete(0, tk.END)
            entry.insert(0, "0")

        self.update_FK(joint_values)


    def get_ee_from_input(self) -> EndEffector:
        """
        Read the end-effector pose from the IK input fields.

        Returns:
            EndEffector: End-effector pose populated from the GUI entries.
        """
        EE = EndEffector()
        EE.x = float(self.pose_button[0].get())
        EE.y = float(self.pose_button[1].get())
        EE.z = float(self.pose_button[2].get())
        EE.rotx = float(self.pose_button[3].get())
        EE.roty = float(self.pose_button[4].get())
        EE.rotz = float(self.pose_button[5].get())
        return EE
    

    def solve_IK1(self) -> None:
        """
        Solves the inverse kinematics for a given end-effector pose using the first solution.
        """
        self.update_IK(pose=self.get_ee_from_input(), soln=0)


    def solve_IK2(self) -> None:
        """
        Solves the inverse kinematics for a given end-effector pose using the second solution.
        """
        self.update_IK(pose=self.get_ee_from_input(), soln=1)


    def numerical_solve(self) -> None:
        """
        Solves the inverse kinematics for a given end-effector pose using a numerical method.
        """
        self.update_IK(pose=self.get_ee_from_input(), soln=1, numerical=True)


    def update_FK(self, joint_values: List[float], display_traj: bool = False) -> None:
        """
        Update the visualization using forward kinematics.

        The GUI accepts joint values in degrees for convenience; they are converted
        to radians before calling the robot simulation.

        Args:
            joint_values: Joint values in degrees (revolute joints) or meters (prismatic joints).
            display_traj: If True, appends the current EE pose to the trajectory trace.

        Returns:
            None. Updates the plot in-place.

        Raises:
            ValueError: If any joint input cannot be parsed as a float.
        """
        if display_traj:
            self.robot.update_ee_trajectory()
        
        try:
            self.robot.update_plot(joint_values=np.deg2rad(joint_values))
            self.canvas.draw()
            self.canvas.flush_events()
        except ValueError:
            tk.messagebox.showerror("Input Error", "Please enter valid numbers")


    def update_IK(
        self, pose: EndEffector, soln: int = 0, numerical: bool = False, display_traj: bool = False
    ) -> None:
        """
        Updates the inverse kinematics plot based on the given end-effector pose.

        Args:
            pose: Desired end-effector pose.
            soln: Solution branch index used by analytical IK. Defaults to 0.
            numerical: If True, use numerical IK. Defaults to False.
            display_traj: If True, appends the current EE pose to the trajectory trace.
        """
        if display_traj:
            self.robot.update_ee_trajectory()
        
        if numerical:
            self.robot.update_plot(pose=pose, soln=soln, numerical=True)
        else:
            self.robot.update_plot(pose=pose, soln=soln)
        self.canvas.draw()
        self.canvas.flush_events()


    def activate_VK(self) -> None:
        """
        Activate velocity kinematics mode.

        While VK is active, the robot is continuously updated using the current
        commanded velocity vector `self.v`, which is controlled by the keyboard
        callbacks (`on_press`, `on_release`).

        Returns:
            None. Enters a loop until VK is deactivated.

        Warning:
            This method runs a blocking loop in the UI thread. For more responsive GUIs,
            consider using `root.after(...)` or threading.
        """
        self.vk_status = True
        while self.vk_status:
            self.robot.move_velocity(self.v)
            self.canvas.draw()
            self.canvas.flush_events()
            time.sleep(0.05)


    def deactivate_VK(self) -> None:
        """
        Deactivates velocity kinematics, stopping the robot's movement.
        """
        self.vk_status = False


    def clear_path(self) -> None:
        joint_values = self.robot.get_joint_values()
        self.robot.reset_ee_trajectory()
        self.update_FK(joint_values)

    
    def generate_path(self) -> None:
        """
        Generates and visualizes a path between the start and goal configurations using RRT.

        This method reads the start and goal joint configurations from the GUI entries,
        plans a path using an RRT planner, and visualizes the resulting path on the plot.

        Returns:
            None. Updates the plot with the planned path.
        """

        if self.robot.model.name == "2-dof":
            self.path_planner = RobotPathPlanner(robot_name="2-dof", obstacle_list=self.robot.obstacle_list)
        elif self.robot.model.name == "hiwonder":
            self.path_planner = RobotPathPlanner(robot_name="hiwonder", obstacle_list=self.robot.obstacle_list)


        q_start = np.deg2rad(eval(self.start_config.get()))
        q_end = np.deg2rad(eval(self.goal_config.get()))
        
        self.path = self.path_planner.plan_path(q_start, q_end, visualize=True)

        if not self.path:
            tk.messagebox.showerror("Planning Error", "No path found within the time limit.")
            return

        waypoint_list = []
        for joint_values in self.path:
            ee, _ = self.robot.model.calc_forward_kinematics(joint_values, radians=True)
            waypoint_list.append([ee.x, ee.y, ee.z])

        self.robot.update_waypoints(waypoint_list)

        self.robot.plot_3D()
        self.canvas.draw()
      
    
    def generate_trajectory(self) -> None:
        """
        Discretizes a joint space path into a trajectory with position, velocity, and acceleration.

        This method takes a list of joint configurations (the path) and generates
        a smooth trajectory by fitting a B-spline. It then computes the position,
        velocity, and acceleration profiles for each joint.

        Args:
            path: A list of joint configurations representing the path.

        Returns:
            None. Updates the trajectory attributes.
        """

        if not hasattr(self, 'path_planner'):
            tk.messagebox.showerror("Error", "Please generate a path first!")
            return
    
        self.trajectory = self.path_planner.generate_trajectory(self.path)

        # for joint_values in path:
        for joint_values in self.trajectory:
            joint_values_deg = np.rad2deg(joint_values)
            self.update_FK(joint_values=joint_values_deg, display_traj=True) 
            time.sleep(0.1)
        
        self.path_planner.plot()


    def update_waypoints(self) -> None:
        """
        Load waypoints from a YAML file and update the robot visualization.

        This reads `waypoints.yml` from the current working directory and expects
        a structure like:

            points:
              - [x, y, z]
              - [x, y, z]
        """
        print('\nUpdating waypoints...')

        with open('data/waypoints.yml', 'r') as file:
            waypoints = yaml.safe_load(file)

        self.waypoint_idx = 0
        self.robot.update_waypoints(waypoints['points'])
        self.robot.plot_3D()
        self.canvas.draw()


    def follow_path(self) -> None:
        """
        Generates and visualizes a joint-space trajectory by solving inverse kinematics at waypoints
        and interpolating between resulting joint configurations.
        """

        print('\nFollowing trajectory in joint space...')
        
        if not hasattr(self, 'path_planner'):
            tk.messagebox.showerror("Error", "Please generate a path first!")
            return

        # for joint_values in path:
        for joint_values in self.path:
            joint_values_deg = np.rad2deg(joint_values)
            self.update_FK(joint_values=joint_values_deg, display_traj=True) 
            time.sleep(0.5)


    def generate_traj_task_space(self):
        """
        Generates and visualizes a task-space trajectory using a polynomial interpolator between waypoints.
        """
    
        print('\nFollowing trajectory in task space...')
    
        if not hasattr(self, 'path_planner'):
            tk.messagebox.showerror("Error", "Please generate a path first!")
            return
        
        points = []
        for joint_values in self.path:
            ee, _ = self.robot.model.calc_forward_kinematics(joint_values, radians=True)
            points.append([ee.x, ee.y, ee.z])

        mstraj = MultiSegmentTrajectoryGenerator(  
            method=self.robot.traj_model, mode="task", ndof=3
        )
        mstraj.solve(points, T=1)
        mstraj.generate(nsteps_per_segment=3)

        traj = mstraj.X[:,0,:].T # taking the position range of X and transposing to get (N, ndof)

        for pos in traj:
            ee = EndEffector(*pos, 0, -math.pi/2, wraptopi(math.atan2(pos[1], pos[0]) + math.pi))
            self.update_IK(ee, soln=0, numerical=True, display_traj=True)
            time.sleep(0.1)
        
        mstraj.plot()


    def generate_traj_joint_space(self):
        """
        Generates and visualizes a joint-space trajectory by solving inverse kinematics at waypoints
        and interpolating between resulting joint configurations.
        """

        print('\nFollowing trajectory in joint space...')

        if not hasattr(self, 'path_planner'):
            tk.messagebox.showerror("Error", "Please generate a path first!")
            return

        mstraj = MultiSegmentTrajectoryGenerator(  
            method=self.robot.traj_model, mode="task", ndof=self.robot.num_joints
        )
        mstraj.solve(self.path, T=1)
        mstraj.generate(nsteps_per_segment=10)

        traj = mstraj.X[:,0,:].T # taking the position range of X and transposing to get (N, ndof)

        for joint_values in traj:
            joint_values_deg = np.rad2deg(joint_values)
            self.update_FK(joint_values=joint_values_deg, display_traj=True) 
            time.sleep(0.1)
        
        mstraj.plot()


    def toggle_obstacles(self) -> None:
        self.robot.toggle_obstacles()
        self.robot.plot_3D()
        self.canvas.draw()

        
    def check_vk_status(self) -> str:
        """
        Checks and returns the status of the velocity kinematics.

        Returns:
            str: The status of velocity kinematics ("Activated!" or "Deactivated!").
        """
        return 'Deactivated!' if not self.vk_status else 'Activated!'


    def on_press(self, key: keyboard.Key) -> None:
        """
        Handles key press events to control the velocity of the robot.

        Args:
            key (pynput.keyboard.Key): The key that was pressed.
        """
        if self.vk_status:
            if key == keyboard.Key.up:
                self.v[1] = 1
            elif key == keyboard.Key.down:
                self.v[1] = -1
            elif key == keyboard.Key.left:
                self.v[0] = -1
            elif key == keyboard.Key.right:
                self.v[0] = 1
            elif hasattr(key, 'char'):
                if key.char == 'w':
                    self.v[2] = 1
                elif key.char == 's':
                    self.v[2] = -1


    def on_release(self, key: keyboard.Key) -> None:
        """
        Handles key release events to stop the robot's movement.

        Args:
            key (pynput.keyboard.Key): The key that was released.
        """
        if key == keyboard.Key.up:
            self.v[1] = 0
        elif key == keyboard.Key.down:
            self.v[1] = 0
        elif key == keyboard.Key.left:
            self.v[0] = 0
        elif key == keyboard.Key.right:
            self.v[0] = 0
        elif hasattr(key, 'char'):
            if key.char == 'w':
                self.v[2] = 0
            elif key.char == 's':
                self.v[2] = 0

    
    def set_pose_values(self, values):
        """
        Populate the IK pose entry fields with specified values.

        Args:
            values: List of 6 values [x, y, z, rotx, roty, rotz]
        """
        if len(values) != 6:
            raise ValueError("Pose must contain exactly 6 values.")

        for entry, val in zip(self.pose_button, values):
            entry.delete(0, tk.END)
            entry.insert(0, str(val))   

    
    def load_current_pose(self):
        """
        Load the current robot pose into the IK entry fields.
        """
        ee = self.robot.model.ee
        pose = [round(val, 4) for val in [ee.x, ee.y, ee.z, ee.rotx, ee.roty, ee.rotz]]

        self.set_pose_values(pose)


    def run(self) -> None:
        """
        Start the Tkinter main loop (if this Visualizer owns the root window).
        """
        self.root.mainloop()



class RobotSim:
    """
    Robot simulation + 3D visualization wrapper for a kinematics model.

    This class wraps a robot model and:
    - Manages a Matplotlib 3D figure
    - Calls the model's kinematics methods to update joint/EE state
    - Draws links, joints, reference frames, and waypoints

    Args:
        robot_model: A robot model providing kinematics methods and state (see module docstring).
        show_animation: Whether to create and update the Matplotlib figure.

    Attributes:
        model: The underlying robot model.
        num_joints: Number of joints/DOF derived from the model.
        fig: Matplotlib Figure (if show_animation=True).
        sub1: 3D axes (if show_animation=True).
        plot_limits: [x_limit, y_limit, z_limit] for display.
        waypoint_x/y/z: Stored waypoint coordinates for visualization.
    """

    def __init__(self, robot_model=None, traj_model=None, show_animation: bool=True) -> None:
        """
        Initializes a robot with a specific configuration based on the type.

        Args:
            robot_model: Robot model instance to be visualized and controlled.
            traj_model: Trajectory method instance to be used in generating feasible trajectories
            show_animation: If True, creates a Matplotlib figure and renders motion.
        """

        self.model = robot_model
        self.traj_model = traj_model
        self.num_joints = robot_model.num_dof        
        
        self.origin = [0., 0., 0.]
        self.axes_length = 0.04
        self.point_x, self.point_y, self.point_z = [], [], []
        self.waypoint_x, self.waypoint_y, self.waypoint_z = [], [], []
        self.waypoint_rotx, self.waypoint_roty, self.waypoint_rotz = [], [], []
        self.joint_trajectory = []
        self.show_animation = show_animation
        self.plot_limits = [0.65, 0.65, 0.8]

        # ----------------------------
        # Obstacles (visual only)
        # ----------------------------
        self.obstacle_list = []  # list of dicts describing obstacles


        if self.show_animation:
            self.fig = Figure(figsize=(12, 10), dpi=100)
            self.sub1 = self.fig.add_subplot(1,1,1, projection='3d') 
            self.fig.suptitle("Manipulator Kinematics Visualization", fontsize=16)
        
        self.init_plot()

    
    def init_plot(self) -> None:
        """
        Initializes the plot by calculating the robot's points and calling the plot function.
        """
        curr_joint_values = self.get_joint_values()
        _, Hlist = self.model.calc_forward_kinematics(curr_joint_values, radians=True)
        self.model.calc_robot_points(self.model.joint_values, Hlist)
        self.plot_3D()

    
    def update_plot(
        self,
        pose: EndEffector = None,
        joint_values: List[float] = None,
        soln: int = 0,
        numerical: bool = False,
    ) -> None:
        """
        Updates the robot's state based on new pose or joint angles and updates the visualization.

        Args:
            pose: Desired end-effector pose. If provided, IK is used.
            joint_values: Joint values. If provided (and pose is None), FK is used.
            soln: IK solution branch index (for analytical IK). Defaults to 0.
            numerical: If True, use numerical IK when pose is provided.
        """
        if pose is not None: # Inverse kinematics case
            curr_joint_values = self.get_joint_values()
            if not numerical:
                new_joint_values = self.model.calc_inverse_kinematics(pose, curr_joint_values, soln=soln)
                _, Hlist = self.model.calc_forward_kinematics(new_joint_values, radians=True)
            else:
                new_joint_values = self.model.calc_numerical_ik(pose, curr_joint_values)
                _, Hlist = self.model.calc_forward_kinematics(new_joint_values, radians=True)
            
            self.model.calc_robot_points(new_joint_values, Hlist)
        
        elif joint_values is not None: # Forward kinematics case
            _, Hlist = self.model.calc_forward_kinematics(joint_values)
            self.model.calc_robot_points(joint_values, Hlist)

        else:
            return
        
        self.plot_3D()


    def move_velocity(self, vel: List[float]) -> None:
        """
        Moves the robot based on a given velocity input.

        Args:
            vel (list): Velocity input for the robot.
        """
        curr_joint_values = self.get_joint_values()
        new_joint_values = self.model.calc_velocity_kinematics(curr_joint_values, vel)
        _, Hlist = self.model.calc_forward_kinematics(new_joint_values)
        self.model.calc_robot_points(new_joint_values, Hlist)
        self.plot_3D()

    
    def plot_ee_trajectory(self):
        xlist, ylist, zlist = [], [], []

        for joint_values in self.joint_trajectory[1:]:  # skip the initial configuration
            ee, _ = self.model.calc_forward_kinematics(joint_values, radians=True)
            xlist.append(ee.x)
            ylist.append(ee.y)
            zlist.append(ee.z)

        # draw the points
        self.sub1.plot(xlist, ylist, zlist, 'b--')
        self.sub1.plot(xlist, ylist, zlist, 'ro', markersize=4)


    def update_ee_trajectory(self):
        self.joint_trajectory.append(self.get_joint_values()) # add the latest thetalist


    def reset_ee_trajectory(self):
        self.joint_trajectory.clear()
        self.waypoint_x.clear()
        self.waypoint_y.clear()
        self.waypoint_z.clear()
        

    def draw_line_3D(self, p1: List[float], p2: List[float], format_type: str = "k-") -> None:
        """
        Draws a 3D line between two points.

        Args:
            p1 (list): Coordinates of the first point.
            p2 (list): Coordinates of the second point.
            format_type (str, optional): The format of the line. Defaults to "k-".
        """
        self.sub1.plot([p1[0], p2[0]], [p1[1], p2[1]], [p1[2], p2[2]], format_type)


    def draw_ref_line(self, point: List[float], axes = None, ref: str = "xyz") -> None:
        """
        Draws reference lines from a given point along specified axes.

        Args:
            point (list): The coordinates of the point to draw from.
            axes (matplotlib.axes, optional): The axes on which to draw the reference lines.
            ref (str, optional): Which reference axes to draw ('xyz', 'xy', or 'xz'). Defaults to 'xyz'.
        """
        line_width = 0.7
        if ref == 'xyz':
            axes.plot([point[0], self.plot_limits[0]],
                      [point[1], point[1]],
                      [point[2], point[2]], 'b--', linewidth=line_width)    # X line
            axes.plot([point[0], point[0]],
                      [point[1], self.plot_limits[1]],
                      [point[2], point[2]], 'b--', linewidth=line_width)    # Y line
            axes.plot([point[0], point[0]],
                      [point[1], point[1]],
                      [point[2], 0.0], 'b--', linewidth=line_width)         # Z line
        elif ref == 'xy':
            axes.plot([point[0], self.plot_limits[0]],
                      [point[1], point[1]], 'b--', linewidth=line_width)    # X line
            axes.plot([point[0], point[0]],
                      [point[1], self.plot_limits[1]], 'b--', linewidth=line_width)    # Y line
        elif ref == 'xz':
            axes.plot([point[0], self.plot_limits[0]],
                      [point[2], point[2]], 'b--', linewidth=line_width)    # X line
            axes.plot([point[0], point[0]],
                      [point[2], 0.0], 'b--', linewidth=line_width)         # Z line


    def plot_waypoints(self) -> None:
        """
        Plots the waypoints in the 3D visualization
        """
        # draw the points
        self.sub1.plot(self.waypoint_x, self.waypoint_y, self.waypoint_z, 'ob', markersize=6)
        self.sub1.plot(self.waypoint_x, self.waypoint_y, self.waypoint_z, '--', color='grey', linewidth=1)


    def update_waypoints(self, waypoints: List[List[float]]) -> None:
        """
        Store waypoint positions for visualization.

        Args:
            waypoints: List of waypoint positions. Each waypoint is expected to contain
                at least [x, y, z].
        """
        for i in range(len(waypoints)):
            self.waypoint_x.append(waypoints[i][0])
            self.waypoint_y.append(waypoints[i][1])
            self.waypoint_z.append(waypoints[i][2])
            # self.waypoint_rotx.append(waypoints[i][3])
            # self.waypoint_roty.append(waypoints[i][4])
            # self.waypoint_rotz.append(waypoints[i][5])


    def get_waypoints(self):
        return [
            [self.waypoint_x[0], self.waypoint_y[0], self.waypoint_z[0]],
            [self.waypoint_x[1], self.waypoint_y[1], self.waypoint_z[1]]
        ]
   
   
    def plot_3D(self) -> None:
        """
        Redraw the full 3D visualization frame.

        Draws:
            - Robot links and joint points
            - End-effector point and coordinate frame
            - Base coordinate frame
            - Waypoints
            - Reference/trace lines
            - Text overlay with EE pose and joint values
        """        
        self.sub1.cla()
        self.point_x.clear()
        self.point_y.clear()
        self.point_z.clear()

        self.draw_obstacles()

        EE = self.model.ee

        # draw lines to connect the points
        for i in range(len(self.model.points)-1):
            self.draw_line_3D(self.model.points[i], self.model.points[i+1], "k-")

        for i in range(len(self.model.points)-1):
            self.draw_cylinder_3D(self.model.points[i], self.model.points[i+1], radius=0.03, resolution=28, alpha=0.5)

        # draw the points
        for i in range(len(self.model.points)):
            self.point_x.append(self.model.points[i][0])
            self.point_y.append(self.model.points[i][1])
            self.point_z.append(self.model.points[i][2])
        self.sub1.plot(self.point_x, self.point_y, self.point_z, marker='o', markerfacecolor='m', markersize=12)

        # draw the waypoints
        self.plot_waypoints()

        # draw the EE trajectory
        self.plot_ee_trajectory()

        # draw the EE
        self.sub1.plot(EE.x, EE.y, EE.z, 'bo')
        # draw the base reference frame
        self.draw_line_3D(self.origin, [self.origin[0] + self.axes_length, self.origin[1], self.origin[2]], format_type='r-')
        self.draw_line_3D(self.origin, [self.origin[0], self.origin[1] + self.axes_length, self.origin[2]], format_type='g-')
        self.draw_line_3D(self.origin, [self.origin[0], self.origin[1], self.origin[2] + self.axes_length], format_type='b-')
        # draw the EE reference frame
        self.draw_line_3D([EE.x, EE.y, EE.z], self.model.EE_axes[0], format_type='r-')
        self.draw_line_3D([EE.x, EE.y, EE.z], self.model.EE_axes[1], format_type='g-')
        self.draw_line_3D([EE.x, EE.y, EE.z], self.model.EE_axes[2], format_type='b-')
        # draw reference / trace lines
        self.draw_ref_line([EE.x, EE.y, EE.z], self.sub1, ref='xyz')

        # add text at bottom of window
        pose_text = "End-effector Pose:      [ "
        pose_text += f"X: {round(EE.x,4)},  "
        pose_text += f"Y: {round(EE.y,4)},  "
        pose_text += f"Z: {round(EE.z,4)},  "
        pose_text += f"RotX: {round(EE.rotx,4)},  "
        pose_text += f"RotY: {round(EE.roty,4)},  "
        pose_text += f"RotZ: {round(EE.rotz,4)}  "
        pose_text += " ]"

        joint_values_text = "Joint Positions (deg/m):     ["
        for i in range(self.num_joints):
            joint_values_text += f" {round(np.rad2deg(self.get_joint_values()[i]),2)}, "
        joint_values_text += " ]"
        
        textstr = pose_text + "\n" + joint_values_text
        self.sub1.text2D(0.2, 0.02, textstr, fontsize=13, transform=self.fig.transFigure)

        self.sub1.set_xlim(-self.plot_limits[0], self.plot_limits[0])
        self.sub1.set_ylim(-self.plot_limits[1], self.plot_limits[1])
        self.sub1.set_zlim(0, self.plot_limits[2])
        self.sub1.set_xlabel('x [m]')
        self.sub1.set_ylabel('y [m]')


    def get_joint_values(self) -> List[float]:
        """
        Return a copy of the model's current joint values.

        Returns:
            list[float]: Joint values (radians internally).
        """
        return self.model.joint_values.copy()


    def cylinder_between(self, p1, p2, radius=0.015, resolution=24):
        p1 = p1[:3]
        p2 = p2[:3]

        v = p2 - p1
        L = np.linalg.norm(v)
        if L < 1e-9:
            # Degenerate link: return nothing
            return None

        v_hat = v / L

        # Pick a helper vector not parallel to v_hat
        a = np.array([0.0, 0.0, 1.0])
        if abs(np.dot(v_hat, a)) > 0.95:
            a = np.array([0.0, 1.0, 0.0])

        n1 = np.cross(v_hat, a)
        n1 /= np.linalg.norm(n1)
        n2 = np.cross(v_hat, n1)  # already unit length if v_hat, n1 are unit

        theta = np.linspace(0, 2*np.pi, resolution)
        circle = (np.cos(theta)[:, None] * n1[None, :] +
                np.sin(theta)[:, None] * n2[None, :]) * radius  # (res,3)

        # Two rings: at p1 and p2
        ring1 = p1[None, :] + circle
        ring2 = p2[None, :] + circle

        X = np.vstack([ring1[:, 0], ring2[:, 0]])
        Y = np.vstack([ring1[:, 1], ring2[:, 1]])
        Z = np.vstack([ring1[:, 2], ring2[:, 2]])
        return X, Y, Z


    def draw_cylinder_3D(self, p1, p2, radius=0.03, resolution=24, alpha=1.0):
        mesh = self.cylinder_between(p1, p2, radius=radius, resolution=resolution)
        if mesh is None:
            return
        X, Y, Z = mesh
        self.sub1.plot_surface(X, Y, Z, alpha=alpha, linewidth=0, color='b', antialiased=True)


    def clear_obstacles(self) -> None:
        """Remove all obstacles."""
        self.obstacle_list.clear()


    def add_cylinder_obstacle(
        self,
        center,
        radius: float = 0.05,
        height: float = 0.20,
        axis: str = "z",
        resolution: int = 28,
        alpha: float = 0.35,
    ) -> None:
        """
        Add a visual cylinder obstacle.

        Args:
            center: [x,y,z] center of the cylinder.
            radius: meters.
            height: meters.
            axis: 'x' | 'y' | 'z' cylinder axis direction.
            resolution: circumferential mesh resolution.
            alpha: transparency.
        """
        self.obstacle_list.append({
            "type": "cylinder",
            "center": list(center),
            "radius": float(radius),
            "height": float(height),
            "axis": axis.lower(),
            "resolution": int(resolution),
            "alpha": float(alpha),
        })


    def add_box_obstacle(
        self,
        center,
        size=0.12,
        alpha: float = 0.25,
    ) -> None:
        """
        Add a visual axis-aligned box (cube if size is a float).

        Args:
            center: [x,y,z] center.
            size: float (cube edge) or [sx, sy, sz] (box dims), meters.
            alpha: transparency.
        """
        if isinstance(size, (int, float)):
            sx = sy = sz = float(size)
        else:
            sx, sy, sz = [float(s) for s in size]

        self.obstacle_list.append({
            "type": "box",
            "center": list(center),
            "size": [sx, sy, sz],
            "alpha": float(alpha),
        })


    def cylinder_mesh(self, center, radius, height, axis="z", resolution=32):
        """
        Returns X,Y,Z for a cylinder surface centered at `center`.
        Axis-aligned cylinder.
        """
        cx, cy, cz = map(float, center)
        theta = np.linspace(0, 2*np.pi, resolution)
        z_lin = np.array([-height/2, height/2])  # 2 rings

        # circle in xy
        x = radius * np.cos(theta)
        y = radius * np.sin(theta)

        # Build surface for axis = z, then rotate by swapping axes if needed
        X = np.vstack([x, x]) + cx
        Y = np.vstack([y, y]) + cy
        Z = np.vstack([np.full_like(theta, z_lin[0]), np.full_like(theta, z_lin[1])]) + cz

        axis = axis.lower()
        if axis == "z":
            return X, Y, Z
        elif axis == "x":
            # cylinder along x: swap (X,Z) roles
            # Z currently varies with height, so map Z -> X; X -> Z
            return Z, Y, X
        elif axis == "y":
            # cylinder along y: map Z -> Y; Y -> Z
            return X, Z, Y
        else:
            raise ValueError("axis must be 'x', 'y', or 'z'")


    def box_faces(self, center, size):
        """
        Create 6 surfaces (faces) for an axis-aligned box.
        Returns list of (X,Y,Z) each 2x2.
        """
        cx, cy, cz = map(float, center)
        sx, sy, sz = map(float, size)
        hx, hy, hz = sx/2, sy/2, sz/2

        x0, x1 = cx - hx, cx + hx
        y0, y1 = cy - hy, cy + hy
        z0, z1 = cz - hz, cz + hz

        faces = []

        # bottom (z=z0)
        faces.append((
            np.array([[x0, x1], [x0, x1]]),
            np.array([[y0, y0], [y1, y1]]),
            np.array([[z0, z0], [z0, z0]]),
        ))

        # top (z=z1)
        faces.append((
            np.array([[x0, x1], [x0, x1]]),
            np.array([[y0, y0], [y1, y1]]),
            np.array([[z1, z1], [z1, z1]]),
        ))

        # left (x=x0)
        faces.append((
            np.array([[x0, x0], [x0, x0]]),
            np.array([[y0, y1], [y0, y1]]),
            np.array([[z0, z0], [z1, z1]]),
        ))

        # right (x=x1)
        faces.append((
            np.array([[x1, x1], [x1, x1]]),
            np.array([[y0, y1], [y0, y1]]),
            np.array([[z0, z0], [z1, z1]]),
        ))

        # front (y=y0)
        faces.append((
            np.array([[x0, x1], [x0, x1]]),
            np.array([[y0, y0], [y0, y0]]),
            np.array([[z0, z0], [z1, z1]]),
        ))

        # back (y=y1)
        faces.append((
            np.array([[x0, x1], [x0, x1]]),
            np.array([[y1, y1], [y1, y1]]),
            np.array([[z0, z0], [z1, z1]]),
        ))

        return faces


    def toggle_obstacles(self) -> None:
        if self.obstacle_list:
            self.clear_obstacles()
        else:
            # self.add_cylinder_obstacle(center=[-0.2, 0.15, 0.2], radius=0.1, height=0.4, axis="z", alpha=0.35)
            # self.add_box_obstacle(center=[0.2, -0.25, 0.25], size=[0.2, 0.2, 0.5], alpha=0.25)
            self.add_box_obstacle(center=[0.24, -0.0, 0.1], size=[0.2, 0.2, 0.2], alpha=0.25)
    
    
    def draw_obstacles(self) -> None:
        """
        Draw all stored obstacles.
        """
        if not self.obstacle_list:
            return
        
        for obs in self.obstacle_list:
            if obs["type"] == "cylinder":
                X, Y, Z = self.cylinder_mesh(
                    obs["center"], obs["radius"], obs["height"],
                    axis=obs.get("axis", "z"),
                    resolution=obs.get("resolution", 32),
                )
                self.sub1.plot_surface(X, Y, Z, alpha=obs.get("alpha", 0.35), linewidth=0, antialiased=True)

            elif obs["type"] == "box":
                faces = self.box_faces(obs["center"], obs["size"])
                for X, Y, Z in faces:
                    self.sub1.plot_surface(X, Y, Z, alpha=obs.get("alpha", 0.25), linewidth=0, antialiased=True)