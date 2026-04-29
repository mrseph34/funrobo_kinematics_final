import tkinter as tk
from tkinter import ttk, messagebox
import socket
import json
import threading
import subprocess
import sys
import time
import os

DETECT_OFFSET_MM = (0, 0, 0) # (-10, 0, 175) 

PI_HOST = "192.168.16.154"
PI_PORT = 9698
MOVE_SPEED_MMS = 100

# Pre-defined obstacle boxes: (x1_mm, y1_mm, z1_mm, x2_mm, y2_mm, z2_mm)
# Two corners in mm matching the GUI XYZ coordinate space.
INIT_BOXES = [
    (-100, 0, -200, 100, 100, -100),
]


########### TESSSST BASE TO EE TRANSFORMS ###########
import numpy as np

# def transform(cam_x, cam_y, cam_z, pitch, yaw, roll, x_c, y_c, z_c):
#     p, y, r = np.radians(-pitch), np.radians(yaw), np.radians(roll)
#     Rx = np.array([[1,0,0],[0,np.cos(p),-np.sin(p)],[0,np.sin(p),np.cos(p)]])
#     Ry = np.array([[np.cos(y),0,np.sin(y)],[0,1,0],[-np.sin(y),0,np.cos(y)]])
#     Rz = np.array([[np.cos(r),-np.sin(r),0],[np.sin(r),np.cos(r),0],[0,0,1]])
#     T_cam = np.eye(4)
#     T_cam[:3,:3] = Ry @ Rx @ Rz
#     T_cam[0,3], T_cam[1,3], T_cam[2,3] = cam_x, cam_y, cam_z
#     p_base_h = T_cam @ np.array([x_c/1000, y_c/1000, z_c/1000, 1.0])
#     return p_base_h[:3] * 1000

# tests = [
#     # (cam_x, cam_y, cam_z, pitch, yaw, roll, x_c, y_c, z_c, expected_x, expected_y, expected_z, label)
#     # pitch=-90 straight down, block at ground
#     (0.0, 0.094, 0.095, -90, 0, 0,  0,  0, 94,   0,  0, 95, "base: straight down, ground"),
#     (0.0, 0.094, 0.095, -90, 0, 0, 10,  0, 94,  10,  0, 95, "base: straight down, 10mm right"),
#     # cam_x +10mm
#     (0.01, 0.094, 0.095, -90, 0, 0, 0, 0, 94, 10, 0, 95, "cam_x+10mm"),
#     # cam_y +10mm
#     (0.0, 0.104, 0.095, -90, 0, 0, 0,  0, 104,  0,  0, 95, "cam_y+10mm, depth+10"),
#     # cam_z +10mm
#     (0.0, 0.094, 0.105, -90, 0, 0, 0,  0, 94,   0,  0, 105, "cam_z+10mm"),
#     # pitch=-60
#     (0.0, 0.094, 0.095, -60, 0, 0, 0,  0, 100,  0, 94-100*np.sin(np.radians(60)), 95+100*np.cos(np.radians(60)), "pitch=-60, depth=100"),
#     # pitch=-120
#     (0.0, 0.094, 0.095, -120, 0, 0, 0, 0, 100,  0, 94-100*np.sin(np.radians(120)), 95+100*np.cos(np.radians(120)), "pitch=-120, depth=100"),
    
    

#     # yaw=90
#     (0.095, 0.094, 0.0, -90, 90, 0, 0, 0, 94, 95, 0, 0, "yaw=90, camera to the right"),
#     # yaw=-90
#     (-0.095, 0.094, 0.0, -90, -90, 0, 0, 0, 94, -95, 0, 0, "yaw=-90, camera to the left"),
#     # yaw=90, pitch=-60, block forward in cam
#     (0.0, 0.094, 0.095, -60, 90, 0, 0, 0, 100, 50, 7.4, 95, "yaw=90, pitch=-60, depth=100"),
#     # yaw=180, camera facing backward, block forward in cam -> backward in base
#     (0.0, 0.094, 0.095, -90, 180, 0, 0, 0, 94, 0, 0, 95, "yaw=180, block below, facing backward"),
    
    
#     # roll=90, block directly below
#     (0.0, 0.094, 0.095, -90, 0, 90, 0, 0, 94, 0, 0, 95, "roll=90, block directly below"),
#     # roll=90, block 10mm right in cam -> now 10mm further in cam-Z -> forward in base
#     (0.0, 0.094, 0.095, -90, 0, 90, 10, 0, 94, 0, 0, 105, "roll=90, block 10mm right in cam"),
#     # roll=-90, block 10mm right in cam -> 10mm backward in cam-Z
#     (0.0, 0.094, 0.095, -90, 0, -90, 10, 0, 94, 0, 0, 85, "roll=-90, block 10mm right in cam"),
# ]

# for t in tests:
#     cam_x, cam_y, cam_z, pitch, yaw, roll, x_c, y_c, z_c, ex, ey, ez, label = t
#     res = transform(cam_x, cam_y, cam_z, pitch, yaw, roll, x_c, y_c, z_c)
#     ok = "✓" if abs(res[0]-ex)<1 and abs(res[1]-ey)<1 and abs(res[2]-ez)<1 else "✗"
#     print(f"{ok} {label}")
#     print(f"   cam pos:   ({cam_x*1000:.0f}, {cam_y*1000:.0f}, {cam_z*1000:.0f}) mm")
#     print(f"   cam block: ({x_c:.0f}, {y_c:.0f}, {z_c:.0f}) mm")
#     print(f"   got:       ({res[0]:.1f}, {res[1]:.1f}, {res[2]:.1f}) mm")
#     print(f"   expected:  ({ex:.1f}, {ey:.1f}, {ez:.1f}) mm")
#     print()

class ArmControlGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("5-DOF Arm Control")
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)
        self.root.configure(bg="#0f1117")
        self.root.resizable(False, False)

        self.sock = None
        self.buf = ""
        self.pi_host = tk.StringVar(value=PI_HOST)
        self.use_aik = tk.BooleanVar(value=True)
        self.speed_mms = tk.DoubleVar(value=MOVE_SPEED_MMS)
        self.traj_method = tk.StringVar(value="Trapezoidal")
        self.use_xyz = tk.BooleanVar(value=True)
        self.status = tk.StringVar(value="Not connected — click Connect")

        self.use_phi = tk.BooleanVar(value=False)
        self.phi_val = tk.StringVar(value="-3.14")
        self.fight_obstacles = tk.BooleanVar(value=False)

        self._sim_enabled = tk.BooleanVar(value=False)
        self._sim_proc = None
        self._offset_x = tk.StringVar(value=str(DETECT_OFFSET_MM[0]))
        self._offset_y = tk.StringVar(value=str(DETECT_OFFSET_MM[1]))
        self._offset_z = tk.StringVar(value=str(DETECT_OFFSET_MM[2]))
        self._sim_viz_proc = None
        self._sim_sock = None
        self._sim_buf = ""
        self._viz_sock = None

        self._brain_proc = None
        self._obstacles = list(INIT_BOXES)
        self._mvp_detect_result = None
        self._mvp_waiting_detect = False
        self._mvp_move_result = None
        self._sim_blocks = [(0, 0, 100)]

        self._build_ui()
        self._launch_brain()

    def _build_ui(self):
        BG = "#0f1117"
        PANEL = "#1a1d27"
        ACC = "#00e5ff"
        TXT = "#e0e0e0"

        style = ttk.Style()
        style.theme_use("clam")
        style.configure("Dark.TFrame", background=PANEL)
        style.configure("Dark.TLabel", background=PANEL, foreground=TXT, font=("Courier", 10))
        style.configure("Title.TLabel", background=PANEL, foreground=ACC, font=("Courier", 11, "bold"))
        style.configure("Accent.TButton", background=ACC, foreground="#000000", font=("Courier", 10, "bold"), padding=6)
        style.map("Accent.TButton", background=[("active", "#00b8cc")])
        style.configure("Dim.TButton", background=PANEL, foreground=TXT, font=("Courier", 9), padding=4)
        style.map("Dim.TButton", background=[("active", "#2a2d3d")])

        outer = tk.Frame(self.root, bg=BG, padx=24, pady=24)
        outer.pack()

        tk.Label(outer, text="5-DOF ARM CONTROL", bg=BG, fg=ACC,
                 font=("Courier", 14, "bold")).pack(anchor="w", pady=(0, 12))

        panel = ttk.Frame(outer, style="Dark.TFrame", padding=16)
        panel.pack()

        def section(label):
            f = ttk.Frame(panel, style="Dark.TFrame")
            f.pack(fill="x", pady=(10, 4))
            ttk.Label(f, text=label, style="Title.TLabel").pack(anchor="w")
            tk.Frame(panel, bg=ACC, height=1).pack(fill="x", pady=(0, 6))

        section("CONNECTION")
        conn_row = ttk.Frame(panel, style="Dark.TFrame")
        conn_row.pack(fill="x", pady=2)
        ttk.Label(conn_row, text="Pi host:", style="Dark.TLabel").pack(side="left")
        tk.Entry(conn_row, textvariable=self.pi_host, width=16, bg="#2a2d3d", fg=TXT,
                 insertbackground=ACC, font=("Courier", 10), bd=0).pack(side="left", padx=6)
        sim_col = ttk.Frame(conn_row, style="Dark.TFrame")
        sim_col.pack(side="right", padx=4)
        tk.Checkbutton(sim_col, text="Simulate", variable=self._sim_enabled,
                       bg=PANEL, fg=TXT, selectcolor="#2a2d3d", activebackground=PANEL,
                       activeforeground=TXT, font=("Courier", 9),
                       command=self._on_sim_toggle).pack(anchor="e")

        btn_row = ttk.Frame(panel, style="Dark.TFrame")
        btn_row.pack(fill="x", pady=4)
        ttk.Button(btn_row, text="Connect", style="Accent.TButton",
                   command=self._connect).pack(side="left", padx=(0, 4))
        ttk.Button(btn_row, text="Disconnect", style="Dim.TButton",
                   command=self._disconnect).pack(side="left")
        ttk.Button(btn_row, text="Refresh Brain", style="Dim.TButton",
                   command=self._refresh_motion, width=14).pack(side="right")

        self.conn_badge = tk.Label(panel, text="● disconnected", bg=PANEL, fg="#ff4466",
                                   font=("Courier", 9, "bold"))
        self.conn_badge.pack(anchor="w", pady=2)

        section("MOVE TO POSITION")
        mode_row = ttk.Frame(panel, style="Dark.TFrame")
        mode_row.pack(fill="x", pady=2)
        ttk.Label(mode_row, text="Mode:", style="Dark.TLabel").pack(side="left")
        for txt, val in [("XYZ", True), ("Joints", False)]:
            tk.Radiobutton(mode_row, text=txt, variable=self.use_xyz, value=val,
                           bg=PANEL, fg=TXT, selectcolor="#2a2d3d",
                           activebackground=PANEL, font=("Courier", 9),
                           command=self._on_mode_change).pack(side="left", padx=4)
        tk.Checkbutton(mode_row, text="Fight Obstacles", variable=self.fight_obstacles,
                       bg=PANEL, fg=TXT, selectcolor="#2a2d3d",
                       activebackground=PANEL, font=("Courier", 9)).pack(side="right", padx=4)

        self.xyz_row = ttk.Frame(panel, style="Dark.TFrame")
        xyz_row = self.xyz_row
        xyz_row.pack(fill="x", pady=4)
        self.xyz_entries = {}
        for label, default in [("X (mm)", "0"), ("Y (mm)", "174"), ("Z (mm)", "233")]:
            col = ttk.Frame(xyz_row, style="Dark.TFrame")
            col.pack(side="left", padx=4)
            ttk.Label(col, text=label, style="Dark.TLabel").pack()
            entry = tk.Entry(col, width=6, bg="#2a2d3d", fg=TXT,
                             insertbackground=ACC, font=("Courier", 12), bd=0, justify="center")
            entry.insert(0, default)
            entry.pack()
            self.xyz_entries[label[0]] = entry

        self._detect_btn = ttk.Button(xyz_row, text="Detect", style="Dim.TButton",
                                      command=self._detect)
        self._detect_btn.pack(side="right", padx=8)

        self._offset_row = ttk.Frame(panel, style="Dark.TFrame")
        ttk.Label(self._offset_row, text="offset:", style="Dark.TLabel").pack(side="left", padx=(4,2))
        for lbl, var in [("X", self._offset_x), ("Y", self._offset_y), ("Z", self._offset_z)]:
            ttk.Label(self._offset_row, text=lbl, style="Dark.TLabel").pack(side="left")
            tk.Entry(self._offset_row, textvariable=var, width=5, bg="#2a2d3d", fg=TXT,
                     insertbackground=ACC, font=("Courier", 9), bd=0, justify="center").pack(side="left", padx=(0,6))

        self.joint_row = ttk.Frame(panel, style="Dark.TFrame")
        joint_row = self.joint_row
        joint_row.pack(fill="x", pady=4)
        self.joint_entries = {}
        for i, default in enumerate([0, 0, 90, -30, 0], start=1):
            col = ttk.Frame(joint_row, style="Dark.TFrame")
            col.pack(side="left", padx=4)
            ttk.Label(col, text=f"J{i}°", style="Dark.TLabel").pack()
            entry = tk.Entry(col, width=6, bg="#2a2d3d", fg=TXT,
                             insertbackground=ACC, font=("Courier", 11), bd=0, justify="center")
            entry.insert(0, str(default))
            entry.pack()
            self.joint_entries[i] = entry

        self._speed_row = ttk.Frame(panel, style="Dark.TFrame")
        speed_row = self._speed_row
        speed_row.pack(fill="x", pady=4)
        ttk.Label(speed_row, text="Speed (mm/s):", style="Dark.TLabel").pack(side="left")
        tk.Scale(speed_row, variable=self.speed_mms, from_=5, to=1000, resolution=5,
                 orient="horizontal", length=160, bg=PANEL, fg=TXT,
                 troughcolor="#2a2d3d", activebackground=ACC,
                 highlightthickness=0, bd=0, showvalue=True,
                 font=("Courier", 9)).pack(side="left", padx=8)

        ik_row = ttk.Frame(panel, style="Dark.TFrame")
        ik_row.pack(fill="x", pady=4)
        ttk.Label(ik_row, text="IK solver:", style="Dark.TLabel").pack(side="left")
        for txt, val in [("AIK", True), ("NIK", False)]:
            tk.Radiobutton(ik_row, text=txt, variable=self.use_aik, value=val,
                           bg=PANEL, fg=TXT, selectcolor="#2a2d3d",
                           activebackground=PANEL, font=("Courier", 9),
                           command=self._on_ik_change).pack(side="left", padx=4)
        self.phi_entry = tk.Entry(ik_row, textvariable=self.phi_val, width=6,
                                   bg="#2a2d3d", fg=TXT, insertbackground=ACC,
                                   font=("Courier", 10), bd=0, justify="center")
        self.phi_entry.pack(side="right", padx=(0, 4))
        self.phi_check = tk.Checkbutton(ik_row, text="Pitch:", variable=self.use_phi,
                                         bg=PANEL, fg=TXT, selectcolor="#2a2d3d",
                                         activebackground=PANEL, activeforeground=TXT,
                                         font=("Courier", 9), command=self._on_ik_change)
        self.phi_check.pack(side="right", padx=(8, 2))

        traj_row = ttk.Frame(panel, style="Dark.TFrame")
        traj_row.pack(fill="x", pady=4)
        ttk.Label(traj_row, text="Traj method:", style="Dark.TLabel").pack(side="left")
        for txt in ["Trapezoidal", "Cubic", "Quintic"]:
            tk.Radiobutton(traj_row, text=txt, variable=self.traj_method, value=txt,
                           bg=PANEL, fg=TXT, selectcolor="#2a2d3d",
                           activebackground=PANEL, font=("Courier", 9)).pack(side="left", padx=4)

        OFF_BG = "#1f2235"
        OBS_BG = "#1f2235"
        OBS_INNER = "#252840"

        obs_outer = tk.Frame(panel, bg=OBS_BG, bd=0, highlightthickness=1,
                             highlightbackground="#3a3d55")
        obs_outer.pack(fill="x", pady=(10, 4))

        self._obs_open = False
        obs_header = tk.Frame(obs_outer, bg=OBS_BG, cursor="hand2")
        obs_header.pack(fill="x")
        self._obs_arrow = tk.Label(obs_header, text="▶  OBSTACLES  (mm)", bg=OBS_BG,
                                   fg="#aaaacc", font=("Courier", 9, "bold"), anchor="w",
                                   padx=10, pady=6)
        self._obs_arrow.pack(side="left", fill="x", expand=True)
        self._obs_count_lbl = tk.Label(obs_header, text="0 boxes", bg=OBS_BG,
                                       fg="#666688", font=("Courier", 8), padx=10)
        self._obs_count_lbl.pack(side="right")

        self._obs_body = tk.Frame(obs_outer, bg=OBS_INNER)

        obs_list_frame = tk.Frame(self._obs_body, bg=OBS_INNER, padx=8, pady=6)
        obs_list_frame.pack(fill="x")
        scrollbar = tk.Scrollbar(obs_list_frame, orient="vertical", bg=OBS_INNER,
                                 troughcolor="#1a1d2e", width=10)
        self._obs_listbox = tk.Listbox(obs_list_frame, height=4, bg="#1a1d2e", fg="#c0c0e0",
                                        selectbackground=ACC, selectforeground="#000",
                                        font=("Courier", 8), bd=0, highlightthickness=0,
                                        yscrollcommand=scrollbar.set, activestyle="none")
        scrollbar.config(command=self._obs_listbox.yview)
        scrollbar.pack(side="right", fill="y")
        self._obs_listbox.pack(side="left", fill="x", expand=True)
        for box in self._obstacles:
            self._obs_listbox.insert("end", self._fmt_box(box))

        obs_divider = tk.Frame(self._obs_body, bg="#2e3150", height=1)
        obs_divider.pack(fill="x", padx=8)

        obs_add_row = tk.Frame(self._obs_body, bg=OBS_INNER, padx=8, pady=6)
        obs_add_row.pack(fill="x")
        self._obs_entries = []
        for lbl in ["x1", "y1", "z1", "x2", "y2", "z2"]:
            col = tk.Frame(obs_add_row, bg=OBS_INNER)
            col.pack(side="left", padx=3)
            tk.Label(col, text=lbl, bg=OBS_INNER, fg="#8888aa",
                     font=("Courier", 7)).pack()
            e = tk.Entry(col, width=5, bg="#1a1d2e", fg="#e0e0ff",
                         insertbackground=ACC, font=("Courier", 9), bd=0,
                         justify="center", highlightthickness=1,
                         highlightbackground="#3a3d55", highlightcolor=ACC)
            e.pack()
            self._obs_entries.append(e)

        obs_btn_row = tk.Frame(self._obs_body, bg=OBS_INNER, padx=8)
        obs_btn_row.pack(fill="x", pady=(0, 8))
        ttk.Button(obs_btn_row, text="+ Add Box", style="Dim.TButton",
                   command=self._obs_add).pack(side="left", padx=(0, 6))
        ttk.Button(obs_btn_row, text="✕ Remove", style="Dim.TButton",
                   command=self._obs_remove).pack(side="left")

        def _toggle_obs(e=None):
            self._obs_open = not self._obs_open
            if self._obs_open:
                self._obs_body.pack(fill="x")
                self._obs_arrow.config(text="▼  OBSTACLES  (mm)")
            else:
                self._obs_body.pack_forget()
                self._obs_arrow.config(text="▶  OBSTACLES  (mm)")
        obs_header.bind("<Button-1>", _toggle_obs)
        self._obs_arrow.bind("<Button-1>", _toggle_obs)
        self._obs_count_lbl.bind("<Button-1>", _toggle_obs)
        self._toggle_obs_count = lambda: self._obs_count_lbl.config(
            text=f"{len(self._obstacles)} box{'es' if len(self._obstacles) != 1 else ''}"
        )
        self._toggle_obs_count()

        SB_BG = "#1a2235"
        SB_INNER = "#1e2840"
        self._sb_row = tk.Frame(panel, bg=SB_INNER, bd=0, highlightthickness=1,
                                highlightbackground="#2a4060")
        tk.Label(self._sb_row, text="SIM BLOCK (mm)", bg=SB_INNER, fg="#66aacc",
                 font=("Courier", 8, "bold"), padx=8).pack(side="left")
        self._sb_entries = []
        for lbl, default in zip(["x", "y", "z"], self._sim_blocks[0]):
            col = tk.Frame(self._sb_row, bg=SB_INNER)
            col.pack(side="left", padx=3, pady=4)
            tk.Label(col, text=lbl, bg=SB_INNER, fg="#556688",
                     font=("Courier", 7)).pack()
            e = tk.Entry(col, width=6, bg="#151c2e", fg="#88ccee",
                         insertbackground="#00e5ff", font=("Courier", 9), bd=0,
                         justify="center", highlightthickness=1,
                         highlightbackground="#2a4060", highlightcolor="#00e5ff")
            e.insert(0, str(int(default)))
            e.pack()
            self._sb_entries.append(e)
        ttk.Button(self._sb_row, text="Sim Detect", style="Dim.TButton",
                   command=self._sim_detect).pack(side="right", padx=6)
        self._update_sb_count = lambda: None

        gripper_row = ttk.Frame(panel, style="Dark.TFrame")
        gripper_row.pack(fill="x", pady=4)
        ttk.Button(gripper_row, text="CLOSE E.E.", style="Dim.TButton",
                   command=self._close_gripper).pack(side="left", expand=True, fill="x", padx=(0, 2))
        ttk.Button(gripper_row, text="OPEN E.E.", style="Dim.TButton",
                   command=self._open_gripper).pack(side="left", expand=True, fill="x", padx=(0, 2))
        ttk.Button(gripper_row, text="SET E.E.", style="Dim.TButton",
                   command=self._set_gripper).pack(side="left", padx=(0, 4))
        self.gripper_val = tk.Entry(gripper_row, width=5, bg="#2a2d3d", fg=TXT,
                                    insertbackground=ACC, font=("Courier", 10), bd=0, justify="center")
        self.gripper_val.insert(0, "50")
        self.gripper_val.pack(side="left")

        ttk.Button(panel, text="⊕  MOVE TO DROPOFF", style="Dim.TButton",
                   command=self._move_dropoff).pack(fill="x", pady=4)
        ttk.Button(panel, text="▶  MOVE TO POSITION", style="Accent.TButton",
                   command=self._move_xyz).pack(fill="x", pady=4)
        ttk.Button(panel, text="⌂  HOME", style="Dim.TButton",
                   command=self._home).pack(fill="x", pady=2)
        ttk.Button(panel, text="RUN MVP", style="Accent.TButton",
                   command=self._run_mvp).pack(fill="x", pady=4)
        ttk.Button(panel, text="RUN MVP2", style="Accent.TButton",
                   command=self._run_mvp2).pack(fill="x", pady=4)
        self._sim_mvp = tk.BooleanVar(value=False)
        self._sim_mvp_frame = tk.Frame(panel, bg=SB_INNER, bd=0, highlightthickness=1,
                                       highlightbackground="#2a4060")
        sim_mvp_left = tk.Frame(self._sim_mvp_frame, bg=SB_INNER)
        sim_mvp_left.pack(side="left", padx=6, pady=4)
        tk.Checkbutton(sim_mvp_left, text="Sim MVP", variable=self._sim_mvp,
                       bg=SB_INNER, fg="#66aacc", selectcolor=SB_INNER,
                       activebackground=SB_INNER, font=("Courier", 8)).pack(side="left")
        self._sim_mvp_scan_var = tk.StringVar()
        self._sim_mvp_dropdown = ttk.Combobox(self._sim_mvp_frame, textvariable=self._sim_mvp_scan_var,
                                               state="readonly", font=("Courier", 8), width=28)
        self._sim_mvp_dropdown["values"] = [
            "none (fail)",
            "mvp1: [0,-30,50,-120,0]",
            "mvp1: [-45,-30,50,-120,0]",
            "mvp1: [45,-30,50,-120,0]",
            "mvp2: [0,-30,60,-120,0]",
            "mvp2: [0,-50,40,-40,0]",
            "mvp2: [-90,-30,60,-120,0]",
            "mvp2: [-90,-50,40,-40,0]",
            "mvp2: [90,-30,60,-120,0]",
            "mvp2: [90,-50,40,-40,0]",
        ]
        self._sim_mvp_dropdown.current(0)
        self._sim_mvp_dropdown.pack(side="left", padx=6, pady=4)

        tk.Label(outer, textvariable=self.status, bg=BG, fg="#888899",
                 font=("Courier", 9), anchor="w").pack(fill="x", pady=(12, 0))

        self._on_mode_change()

    def _fmt_block(self, blk):
        return "({:.0f}, {:.0f}, {:.0f})".format(*blk)

    def _sb_add(self):
        pass

    def _sb_remove(self):
        pass

    def _send_sim_blocks(self):
        if self._sim_sock:
            try:
                self._sim_sock.sendall((json.dumps({"cmd": "set_sim_blocks", "blocks": list(self._sim_blocks)}) + "\n").encode())
            except Exception:
                pass

    def _sim_detect(self):
        try:
            vals = [float(e.get()) for e in self._sb_entries]
        except ValueError:
            self.status.set("Invalid sim block values.")
            return
        self._sim_blocks[0] = tuple(vals)
        self._send_sim_blocks()
        bx, by, bz = self._sim_blocks[0]
        print(f"[SIM DETECT] block=({bx},{by},{bz})")
        if self._sim_sock:
            try:
                self._sim_sock.sendall((json.dumps({"cmd": "sim_detect", "x_mm": bx, "y_mm": by, "z_mm": bz}) + "\n").encode())
            except Exception as e:
                self.status.set(f"Sim detect error: {e}")
        else:
            self.status.set("Brain not connected.")

    def _fmt_box(self, box):
        return "({:.0f},{:.0f},{:.0f}) → ({:.0f},{:.0f},{:.0f})".format(*box)

    def _send_obstacles(self):
        msg = {"cmd": "set_obstacles", "boxes": list(self._obstacles)}
        self._send(msg)

    def _obs_add(self):
        try:
            vals = [float(e.get()) for e in self._obs_entries]
        except ValueError:
            messagebox.showerror("Invalid input", "All 6 fields must be numbers.")
            return
        box = tuple(vals)
        self._obstacles.append(box)
        self._obs_listbox.insert("end", self._fmt_box(box))
        self._toggle_obs_count()
        self._send_obstacles()

    def _obs_remove(self):
        sel = self._obs_listbox.curselection()
        if not sel:
            return
        idx = sel[0]
        self._obstacles.pop(idx)
        self._obs_listbox.delete(idx)
        self._toggle_obs_count()
        self._send_obstacles()

    def _on_ik_change(self):
        aik = self.use_aik.get()
        state = "normal" if aik and self.use_phi.get() else "disabled"
        phi_state = "normal" if aik else "disabled"
        self.phi_entry.config(state=state)
        self.phi_check.config(state=phi_state)

    def _on_mode_change(self):
        if self.use_xyz.get():
            self.joint_row.pack_forget()
            self.xyz_row.pack(fill="x", pady=4, before=self._speed_row)
            self._offset_row.pack(fill="x", pady=(0,4), before=self._speed_row)
        else:
            self.xyz_row.pack_forget()
            self._offset_row.pack_forget()
            self.joint_row.pack(fill="x", pady=4, before=self._speed_row)

    def _get_phi(self):
        if not self.use_aik.get() or not self.use_phi.get():
            return None
        try:
            return float(self.phi_val.get())
        except ValueError:
            return None

    def _on_sim_toggle(self):
        if self._sim_enabled.get():
            self._start_sim()
        else:
            self._stop_sim()

    def _start_sim(self):
        if self._sim_proc is not None:
            return
        self._sb_row.pack(fill="x", pady=(10, 4))
        self._sim_mvp_frame.pack(fill="x", pady=(0, 4))
        try:
            script_dir = os.path.dirname(os.path.abspath(__file__))
            examples_dir = os.path.join(os.path.dirname(script_dir), "examples")
            env = os.environ.copy()
            existing = env.get("PYTHONPATH", "")
            env["PYTHONPATH"] = (existing + os.pathsep if existing else "") + script_dir + os.pathsep + examples_dir
            self._sim_viz_proc = subprocess.Popen(
                [sys.executable, os.path.join(script_dir, "pp_local_server.py")],
                cwd=script_dir, env=env,
            )
            self._sim_proc = self._sim_viz_proc  # just tracks viz for poll
        except Exception as e:
            self.status.set(f"Sim launch failed: {e}")
            self._sim_enabled.set(False)
            return

        def _connect_sim():
            for _ in range(20):
                time.sleep(0.3)
                try:
                    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                    s.connect(("localhost", 9999))
                    s.settimeout(0.1)
                    self._sim_sock = s
                    self.root.after(0, lambda: self.status.set("Simulator running."))
                    threading.Thread(target=self._listen_sim, daemon=True).start()
                    def _send_one_by_one():
                        time.sleep(1.0)
                        for i in range(len(self._obstacles)):
                            msg = {"cmd": "set_obstacles", "boxes": list(self._obstacles[:i+1])}
                            self._send(msg)
                            time.sleep(0.1)
                        self._send_sim_blocks()
                    threading.Thread(target=_send_one_by_one, daemon=True).start()
                    self.root.after(500, self._poll_sim)
                    return
                except Exception:
                    pass
            self.root.after(0, lambda: self.status.set("Sim failed to start"))
            self._sim_enabled.set(False)

        threading.Thread(target=_connect_sim, daemon=True).start()

    def _poll_sim(self):
        if self._sim_viz_proc is None:
            return
        viz_dead = self._sim_viz_proc.poll() is not None
        if viz_dead:
            self._stop_sim()
            self._sim_enabled.set(False)
            self.status.set("Simulator stopped.")
            return
        self.root.after(500, self._poll_sim)

    def _launch_brain(self):
        if self._brain_proc is not None and self._brain_proc.poll() is None:
            self._brain_proc.kill()
            self._brain_proc = None
        try:
            import signal
            result = subprocess.run(["lsof", "-ti", "tcp:9999"], capture_output=True, text=True)
            for pid in result.stdout.strip().split():
                try:
                    os.kill(int(pid), signal.SIGKILL)
                except Exception:
                    pass
        except Exception:
            pass
        script_dir = os.path.dirname(os.path.abspath(__file__))
        examples_dir = os.path.join(os.path.dirname(script_dir), "examples")
        env = os.environ.copy()
        existing = env.get("PYTHONPATH", "")
        env["PYTHONPATH"] = (existing + os.pathsep if existing else "") + script_dir + os.pathsep + examples_dir
        try:
            self._brain_proc = subprocess.Popen(
                [sys.executable, os.path.join(script_dir, "path_planner_brain.py")],
                cwd=script_dir, env=env,
            )
            print(f"[GUI] Brain launched (pid {self._brain_proc.pid})")
        except Exception as e:
            print(f"[GUI] Brain launch failed: {e}")

    def _on_close(self):
        self._stop_sim()
        if self._brain_proc:
            self._brain_proc.kill()
            self._brain_proc = None
        if self.sock:
            try:
                self.sock.close()
            except Exception:
                pass
        self.root.destroy()

    def _stop_sim(self):
        self._sb_row.pack_forget()
        self._sim_mvp_frame.pack_forget()
        if self._viz_sock:
            try:
                self._viz_sock.close()
            except Exception:
                pass
            self._viz_sock = None
        if self._sim_sock:
            try:
                self._sim_sock.close()
            except Exception:
                pass
            self._sim_sock = None
        if self._sim_viz_proc:
            self._sim_viz_proc.kill()
            self._sim_viz_proc = None
        self._sim_proc = None

    def _listen_sim(self):
        while self._sim_sock:
            try:
                data = self._sim_sock.recv(4096).decode()
                if data:
                    self._sim_buf += data
                    while "\n" in self._sim_buf:
                        line, self._sim_buf = self._sim_buf.split("\n", 1)
                        msg = json.loads(line)
                        if msg.get("status") == "joints":
                            if self.sock:
                                try:
                                    # print(f"[GUI] brain -> pi: set_joints {msg['joints']}")
                                    self.sock.sendall((json.dumps({"cmd": "set_joints", "joints": msg["joints"]}) + "\n").encode())
                                except Exception as e:
                                    pass  # print(f"[GUI] pi forward error: {e}")
                        elif msg.get("status") == "done":
                            self._mvp_move_result = "done"
                        elif msg.get("status") == "ik_failed":
                            self.root.after(0, lambda: self.status.set("IK failed."))
                            self._mvp_move_result = "ik_failed"
                        elif msg.get("status") == "sim":
                            self._print_sim(msg)
                        elif msg.get("cmd") == "transform_result":
                            ox = float(self._offset_x.get() or 0)
                            oy = float(self._offset_y.get() or 0)
                            oz = float(self._offset_z.get() or 0)
                            x = msg["x_mm"] + ox
                            y = msg["y_mm"] + oy
                            z = msg["z_mm"] + oz
                            if (ox, oy, oz) != (0, 0, 0):
                                print(f"[GUI RECV transform_result] transformed=({msg['x_mm']:.1f},{msg['y_mm']:.1f},{msg['z_mm']:.1f}) + offset({ox},{oy},{oz}) -> ({x:.1f},{y:.1f},{z:.1f}) mm")
                            else:
                                print(f"[GUI RECV transform_result] transformed=({x:.1f},{y:.1f},{z:.1f}) mm")
                            self._mvp_detect_result = {"cmd": "transform_result", "x_mm": x, "y_mm": y, "z_mm": z}
                            def _update(x=x, y=y, z=z):
                                self.xyz_entries["X"].delete(0, "end"); self.xyz_entries["X"].insert(0, f"{x:.1f}")
                                self.xyz_entries["Y"].delete(0, "end"); self.xyz_entries["Y"].insert(0, f"{y:.1f}")
                                self.xyz_entries["Z"].delete(0, "end"); self.xyz_entries["Z"].insert(0, f"{z:.1f}")
                                self.status.set(f"Detected: ({x:.1f}, {y:.1f}, {z:.1f}) mm")
                            self.root.after(0, _update)
                else:
                    break
            except socket.timeout:
                if self._sim_proc is not None and self._sim_proc.poll() is not None:
                    break
                continue
            except Exception:
                break
        if self._sim_sock:
            try:
                self._sim_sock.close()
            except Exception:
                pass
            self._sim_sock = None
        # only uncheck if viz is also gone
        viz_alive = self._sim_viz_proc is not None and self._sim_viz_proc.poll() is None
        if not viz_alive:
            self._sim_proc = None
            self.root.after(0, lambda: self._sim_enabled.set(False))

    def _reconnect_sim(self):
        def _do():
            for _ in range(20):
                time.sleep(0.3)
                try:
                    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                    s.connect(("localhost", 9999))
                    s.settimeout(0.1)
                    self._sim_sock = s
                    self.root.after(0, lambda: self.status.set("Motion server reconnected."))
                    threading.Thread(target=self._listen_sim, daemon=True).start()
                    self.root.after(0, self._send_obstacles)
                    return
                except Exception:
                    pass
            self.root.after(0, lambda: self.status.set("Motion reconnect failed."))
        threading.Thread(target=_do, daemon=True).start()
        self.root.after(0, lambda: self.status.set("Simulator stopped."))

    def _connect(self):
        self.status.set("Connecting...")
        self.conn_badge.config(text="● connecting…", fg="#ffaa00")
        def _do_connect():
            try:
                pi = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                pi.settimeout(2.5)
                pi.connect((self.pi_host.get(), PI_PORT))
                pi.settimeout(0.1)
                self.sock = pi
                threading.Thread(target=self._listen, daemon=True).start()
                br = None
                for _ in range(10):
                    try:
                        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                        s.settimeout(1.0)
                        s.connect(("localhost", 9999))
                        s.settimeout(0.1)
                        br = s
                        break
                    except Exception:
                        time.sleep(0.3)
                if br is None:
                    raise Exception("Brain not reachable on localhost:9999")
                self._sim_sock = br
                self.root.after(0, lambda: self.conn_badge.config(text="● connected", fg="#00ff88"))
                self.root.after(0, lambda: self.status.set(f"Connected to {self.pi_host.get()}"))
                threading.Thread(target=self._listen_sim, daemon=True).start()
                self.root.after(0, self._send_obstacles)
            except Exception as e:
                self.sock = None
                self._sim_sock = None
                err = str(e)
                self.root.after(0, lambda: self.conn_badge.config(text="● disconnected", fg="#ff4466"))
                self.root.after(0, lambda: self.status.set(f"Connection failed: {err}"))
        threading.Thread(target=_do_connect, daemon=True).start()

    def _disconnect(self):
        if self.sock:
            self.sock.close()
            self.sock = None
        self.conn_badge.config(text="● disconnected", fg="#ff4466")
        self.status.set("Disconnected.")

    def _refresh_motion(self):
        if self._brain_proc is not None:
            self._brain_proc.kill()
            self._brain_proc = None
        if self._sim_proc is not None:
            self._sim_proc.kill()
        self._launch_brain()
        self.status.set("Motion server reloaded.")
        self._reconnect_sim()

    def _listen(self):
        while self.sock:
            try:
                data = self.sock.recv(4096).decode()
                if data:
                    self.buf += data
                    while "\n" in self.buf:
                        line, self.buf = self.buf.split("\n", 1)
                        msg = json.loads(line)
                        if msg.get("status") == "done":
                            # print(f"[GUI] recv: done")
                            self.status.set("Move complete.")
                        elif msg.get("status") == "ik_failed":
                            self.root.after(0, lambda: self.status.set("IK failed."))
                            self._mvp_move_result = "ik_failed"
                        elif msg.get("status") == "homed":
                            # print(f"[GUI] recv: homed")
                            self.status.set("Homed.")
                        elif msg.get("status") == "sim":
                            self._print_sim(msg)
                        elif msg.get("status") == "error":
                            self.status.set(f"Error: {msg.get('msg', '')}")
                        elif msg.get("cmd") == "detect_result":
                            print(f"[GUI RECV detect_result] raw=({msg.get('x_mm')},{msg.get('y_mm')},{msg.get('z_mm')}) mm")
                            if "error" in msg:
                                self._mvp_detect_result = msg
                                self.root.after(0, lambda m=msg: self.status.set(f"Detect: {m['error']}"))
                            else:
                                try:
                                    self._sim_sock.sendall((json.dumps({"cmd": "transform_detect", "x_mm": msg["x_mm"], "y_mm": msg["y_mm"], "z_mm": msg["z_mm"]}) + "\n").encode())
                                except Exception as e:
                                    print(f"[GUI] transform_detect send error: {e}")
                else:
                    break
            except socket.timeout:
                continue
            except Exception as e:
                print(f"[GUI _listen ERROR] {e}")
                break
        if self.sock:
            try:
                self.sock.close()
            except Exception:
                pass
            self.sock = None
        self.root.after(0, lambda: self.conn_badge.config(text="● disconnected", fg="#ff4466"))
        self.root.after(0, lambda: self.status.set("Disconnected (connection lost)."))

    def _print_sim(self, msg):
        print("=" * 48)
        print(f"  SIMULATION SUMMARY")
        print(f"  IK:          {msg['ik']}")
        print(f"  Traj:        {msg['traj']}")
        print(f"  Distance:    {msg['dist_mm']:.1f} mm")
        print(f"  Steps:       {msg['n_steps']}")
        print(f"  Est. time:   {msg['est_time_s']:.2f} s")
        print(f"  Start EE:    x={msg['start'][0]:.4f}  y={msg['start'][1]:.4f}  z={msg['start'][2]:.4f}")
        print(f"  Target EE:   x={msg['target'][0]:.4f}  y={msg['target'][1]:.4f}  z={msg['target'][2]:.4f}")
        print(f"  IK success:  {msg['ik_ok']}")
        if msg.get('max_joint_delta_deg') is not None:
            print(f"  Max joint Δ: {msg['max_joint_delta_deg']:.2f} deg")
        print("=" * 48)
        self.status.set(f"Sim: {msg['est_time_s']:.2f}s  |  {msg['n_steps']} steps  |  {msg['ik']}  |  {msg['traj']}")

    def _send(self, msg):
        active = self._sim_sock or self.sock
        if not active:
            messagebox.showerror("Not connected", "Connect to the Pi or enable Simulate first.")
            return False
        target = "sim" if self._sim_sock else f"pi({self.pi_host.get()})"
        # print(f"[GUI] send -> {target}: {msg}")
        data = (json.dumps(msg) + "\n").encode()
        try:
            active.sendall(data)
        except Exception as e:
            pass  # print(f"[GUI] send error: {e}")
        return True

    def _send_atomic(self, stop_msg, move_msg):
        active = self._sim_sock or self.sock
        if not active:
            messagebox.showerror("Not connected", "Connect to the Pi or enable Simulate first.")
            return False
        target = "sim" if self._sim_sock else f"pi({self.pi_host.get()})"
        # print(f"[GUI] send -> {target}: {stop_msg}")
        # print(f"[GUI] send -> {target}: {move_msg}")
        data = (json.dumps(stop_msg) + "\n" + json.dumps(move_msg) + "\n").encode()
        try:
            active.sendall(data)
        except Exception as e:
            pass  # print(f"[GUI] send error: {e}")
        return True

    def _get_xyz(self):
        try:
            return (float(self.xyz_entries["X"].get()),
                    float(self.xyz_entries["Y"].get()),
                    float(self.xyz_entries["Z"].get()))
        except ValueError:
            messagebox.showerror("Invalid input", "X, Y, and Z must be numbers.")
            return None

    def _detect(self):
        if not self.sock:
            self.status.set("Not connected to Pi.")
            return
        self.status.set("Detecting...")
        try:
            self.sock.sendall((json.dumps({"cmd": "detect"}) + "\n").encode())
        except Exception as e:
            self.status.set(f"Detect error: {e}")

    def _move_dropoff(self):
        print(f"[GUI] _move_dropoff clicked speed={self.speed_mms.get()}mm/s")
        if self._send_atomic({"cmd": "stop"}, {
            "cmd": "move_xyz",
            "x_mm": 0, "y_mm": 200, "z_mm": -150,
            "speed_mms": self.speed_mms.get(),
            "use_aik": self.use_aik.get(),
            "phi_d": self._get_phi(),
            "traj_method": self.traj_method.get(),
            "fight_obstacles": self.fight_obstacles.get(),
        }):
            self.status.set("Moving to dropoff...")

    def _move_xyz(self):
        print(f"[GUI] _move_xyz clicked mode={'xyz' if self.use_xyz.get() else 'joints'} speed={self.speed_mms.get()}mm/s")
        if self.use_xyz.get():
            xyz = self._get_xyz()
            if xyz is None:
                return
            x, y, z = xyz
            if self._send_atomic({"cmd": "stop"}, {
                "cmd": "move_xyz",
                "x_mm": x, "y_mm": y, "z_mm": z,
                "speed_mms": self.speed_mms.get(),
                "use_aik": self.use_aik.get(),
                "phi_d": self._get_phi(),
                "traj_method": self.traj_method.get(),
                "fight_obstacles": self.fight_obstacles.get(),
            }):
                self.status.set(f"Moving to X={x:.1f}  Y={y:.1f}  Z={z:.1f} mm...")
        else:
            try:
                joints = [float(self.joint_entries[i].get()) for i in range(1, 6)]
            except ValueError:
                messagebox.showerror("Invalid input", "All joint angles must be numbers.")
                return
            if self._send_atomic({"cmd": "stop"}, {
                "cmd": "move_joints",
                "joints": joints,
                "speed_mms": self.speed_mms.get(),
                "traj_method": self.traj_method.get(),
                "fight_obstacles": self.fight_obstacles.get(),
            }):
                self.status.set(f"Moving to joints {joints}...")

    def _send_gripper(self, msg):
        active = self.sock or self._sim_sock
        if not active:
            messagebox.showerror("Not connected", "Connect to the Pi first.")
            return False
        try:
            active.sendall((json.dumps(msg) + "\n").encode())
            print(f"[GUI GRIPPER] sent: {msg}")
        except Exception as e:
            print(f"[GUI GRIPPER] send error: {e}")
            return False
        return True

    def _open_gripper(self):
        if self._send_gripper({"cmd": "gripper", "action": "open", "width": -100}):
            self.status.set("Opening gripper...")

    def _close_gripper(self):
        if self._send_gripper({"cmd": "gripper", "action": "close", "width": -10}):
            self.status.set("Closing gripper...")

    def _set_gripper(self):
        try:
            w = -abs(int(self.gripper_val.get()))
        except ValueError:
            messagebox.showerror("Invalid input", "Gripper value must be a number.")
            return
        if self._send_gripper({"cmd": "gripper", "action": "set", "width": w}):
            self.status.set(f"Gripper set to {abs(w)}...")

    def _run_mvp(self):
        if not self.sock and not (self._sim_mvp.get() and self._sim_sock):
            self.status.set("Not connected to Pi.")
            return
        self.status.set("MVP running...")
        import mvp
        import importlib; importlib.reload(mvp)
        import threading
        sim_pt = self._get_sim_mvp_pt() if self._sim_mvp.get() else None
        threading.Thread(target=mvp.run, args=(self,), kwargs={"sim_scan_pt": sim_pt}, daemon=True).start()

    def _run_mvp2(self):
        if not self.sock and not (self._sim_mvp.get() and self._sim_sock):
            self.status.set("Not connected to Pi.")
            return
        self.status.set("MVP2 running...")
        import mvp2
        import importlib; importlib.reload(mvp2)
        import threading
        sim_pt = self._get_sim_mvp_pt() if self._sim_mvp.get() else None
        threading.Thread(target=mvp2.run, args=(self,), kwargs={"sim_scan_pt": sim_pt}, daemon=True).start()

    def _get_sim_mvp_pt(self):
        val = self._sim_mvp_scan_var.get()
        try:
            import ast
            return ast.literal_eval(val.split(": ", 1)[1])
        except Exception:
            return None
    def _home(self):
        print(f"[GUI] _home clicked")
        if self._send_atomic({"cmd": "stop"}, {"cmd": "home"}):
            self.status.set("Homing...")


if __name__ == "__main__":
    root = tk.Tk()
    ArmControlGUI(root)
    root.mainloop()