import tkinter as tk
from tkinter import ttk, messagebox
import socket
import json
import threading

PI_HOST = "192.168.16.69"
PI_PORT = 9999
MOVE_SPEED_MMS = 30


class ArmControlGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("5-DOF Arm Control")
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

        self._build_ui()

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

        btn_row = ttk.Frame(panel, style="Dark.TFrame")
        btn_row.pack(fill="x", pady=4)
        ttk.Button(btn_row, text="Connect", style="Accent.TButton",
                   command=self._connect).pack(side="left", padx=(0, 4))
        ttk.Button(btn_row, text="Disconnect", style="Dim.TButton",
                   command=self._disconnect).pack(side="left")

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
                           activebackground=PANEL, font=("Courier", 9)).pack(side="left", padx=4)

        xyz_row = ttk.Frame(panel, style="Dark.TFrame")
        xyz_row.pack(fill="x", pady=4)
        self.xyz_entries = {}
        for label, default in [("X (mm)", "0.0"), ("Y (mm)", "-233.6"), ("Z (mm)", "174.0")]:
            col = ttk.Frame(xyz_row, style="Dark.TFrame")
            col.pack(side="left", padx=8)
            ttk.Label(col, text=label, style="Dark.TLabel").pack()
            entry = tk.Entry(col, width=8, bg="#2a2d3d", fg=TXT,
                             insertbackground=ACC, font=("Courier", 12), bd=0, justify="center")
            entry.insert(0, default)
            entry.pack()
            self.xyz_entries[label[0]] = entry

        joint_row = ttk.Frame(panel, style="Dark.TFrame")
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

        speed_row = ttk.Frame(panel, style="Dark.TFrame")
        speed_row.pack(fill="x", pady=4)
        ttk.Label(speed_row, text="Speed (mm/s):", style="Dark.TLabel").pack(side="left")
        tk.Scale(speed_row, variable=self.speed_mms, from_=5, to=100, resolution=5,
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
                           activebackground=PANEL, font=("Courier", 9)).pack(side="left", padx=4)

        traj_row = ttk.Frame(panel, style="Dark.TFrame")
        traj_row.pack(fill="x", pady=4)
        ttk.Label(traj_row, text="Traj method:", style="Dark.TLabel").pack(side="left")
        for txt in ["Trapezoidal", "Cubic", "Quintic"]:
            tk.Radiobutton(traj_row, text=txt, variable=self.traj_method, value=txt,
                           bg=PANEL, fg=TXT, selectcolor="#2a2d3d",
                           activebackground=PANEL, font=("Courier", 9)).pack(side="left", padx=4)

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

        ttk.Button(panel, text="⊕  SIMULATE", style="Dim.TButton",
                   command=self._simulate).pack(fill="x", pady=4)
        ttk.Button(panel, text="▶  MOVE TO POSITION", style="Accent.TButton",
                   command=self._move_xyz).pack(fill="x", pady=4)
        ttk.Button(panel, text="⌂  HOME", style="Dim.TButton",
                   command=self._home).pack(fill="x", pady=2)

        tk.Label(outer, textvariable=self.status, bg=BG, fg="#888899",
                 font=("Courier", 9), anchor="w").pack(fill="x", pady=(12, 0))

    def _connect(self):
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.connect((self.pi_host.get(), PI_PORT))
            self.sock.settimeout(0.1)
            self.conn_badge.config(text="● connected", fg="#00ff88")
            self.status.set(f"Connected to {self.pi_host.get()}")
            threading.Thread(target=self._listen, daemon=True).start()
        except Exception as e:
            self.status.set(f"Connection failed: {e}")

    def _disconnect(self):
        if self.sock:
            self.sock.close()
            self.sock = None
        self.conn_badge.config(text="● disconnected", fg="#ff4466")
        self.status.set("Disconnected.")

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
                            self.status.set("Move complete.")
                        elif msg.get("status") == "homed":
                            self.status.set("Homed.")
                        elif msg.get("status") == "sim":
                            self._print_sim(msg)
                        elif msg.get("status") == "error":
                            self.status.set(f"Error: {msg.get('msg', '')}")
            except socket.timeout:
                continue
            except Exception:
                break

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
        if not self.sock:
            messagebox.showerror("Not connected", "Connect to the Pi first.")
            return False
        self.sock.sendall((json.dumps(msg) + "\n").encode())
        return True

    def _get_xyz(self):
        try:
            return (float(self.xyz_entries["Y"].get()),
                    float(self.xyz_entries["X"].get()),
                    float(self.xyz_entries["Z"].get()))
        except ValueError:
            messagebox.showerror("Invalid input", "X, Y, and Z must be numbers.")
            return None

    def _simulate(self):
        xyz = self._get_xyz()
        if xyz is None:
            return
        x, y, z = xyz
        self._send({
            "cmd": "simulate",
            "x_mm": x, "y_mm": y, "z_mm": z,
            "speed_mms": self.speed_mms.get(),
            "use_aik": self.use_aik.get(),
            "traj_method": self.traj_method.get(),
        })
        self.status.set("Simulating...")

    def _move_xyz(self):
        if self.use_xyz.get():
            xyz = self._get_xyz()
            if xyz is None:
                return
            x, y, z = xyz
            if self._send({
                "cmd": "move_xyz",
                "x_mm": x, "y_mm": y, "z_mm": z,
                "speed_mms": self.speed_mms.get(),
                "use_aik": self.use_aik.get(),
                "traj_method": self.traj_method.get(),
            }):
                self.status.set(f"Moving to X={x:.1f}  Y={y:.1f}  Z={z:.1f} mm...")
        else:
            try:
                joints = [float(self.joint_entries[i].get()) for i in range(1, 6)]
            except ValueError:
                messagebox.showerror("Invalid input", "All joint angles must be numbers.")
                return
            if self._send({
                "cmd": "move_joints",
                "joints": joints,
                "speed_mms": self.speed_mms.get(),
                "traj_method": self.traj_method.get(),
            }):
                self.status.set(f"Moving to joints {joints}...")

    def _open_gripper(self):
        if self._send({"cmd": "gripper", "action": "open", "width": -100}):
            self.status.set("Opening gripper...")

    def _close_gripper(self):
        if self._send({"cmd": "gripper", "action": "close", "width": -10}):
            self.status.set("Closing gripper...")

    def _set_gripper(self):
        try:
            w = -abs(int(self.gripper_val.get()))
        except ValueError:
            messagebox.showerror("Invalid input", "Gripper value must be a number.")
            return
        if self._send({"cmd": "gripper", "action": "set", "width": w}):
            self.status.set(f"Gripper set to {abs(w)}...")

    def _home(self):
        if self._send({"cmd": "home"}):
            self.status.set("Homing...")


if __name__ == "__main__":
    root = tk.Tk()
    ArmControlGUI(root)
    root.mainloop()