"""
Path Planner Sim Server — runs locally on your laptop.
Start this first, then connect the GUI with host = "localhost".

Standalone matplotlib visualizer — no dependency on RobotSim/Visualizer.
"""

import sys
import os
sys.path.append(os.path.abspath("examples"))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import socket
import json
import time
import threading
import queue
import select
import numpy as np
import math
import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

from traj_gen import CubicPolynomial, QuinticPolynomial, TrapezoidalTrajectory
import funrobo_kinematics.core.utils as ut
from five_dof import FiveDOFRobot

HOST = "localhost"
PORT = 9999
CONTROL_HZ = 20
DT = 1.0 / CONTROL_HZ
JOINT_DELTA_CLAMP_DEG = 15.0

HOME_JOINTS_DEG = [0, 0, 90, -30, 0]
HOME_JOINTS_RAD = [np.deg2rad(j) for j in HOME_JOINTS_DEG]

model = None
curr_joints = None
running = False
plot_queue = queue.Queue()
_move_cancel = False
_move_thread = None


def remap(x_mm, y_mm, z_mm):
    return -z_mm / 1000.0, x_mm / 1000.0, y_mm / 1000.0


def make_ee(x, y, z):
    ee = ut.EndEffector()
    ee.x, ee.y, ee.z = x, y, z
    return ee


def solve_ik(ee, use_aik, phi_d=None):
    seed = [0.0, 0.0, 0.0, 0.0, 0.0]
    if use_aik:
        result = model.calc_inverse_kinematics(ee, seed, phi_d=phi_d)
    else:
        result = model._newton_raphson_step(
            seed, np.array([ee.x, ee.y, ee.z]), tol=0.001, max_iter=30
        )
    return None if (result is None or len(result) == 0) else result


def get_joint_positions(joints_rad):
    """Return list of [x,y,z] for base + each joint origin using cumulative FK transforms."""
    _, Hlist = model.calc_forward_kinematics(joints_rad)
    pts = [np.array([0.0, 0.0, 0.0])]
    H = np.eye(4)
    for Hi in Hlist:
        H = H @ Hi
        pts.append(H[:3, 3])
    return pts


def push_joints(joints_rad):
    plot_queue.put(list(joints_rad))


def _cancel_move():
    global _move_cancel
    _move_cancel = True
    if _move_thread and _move_thread.is_alive():
        _move_thread.join(timeout=0.3)


def move_to_traj(x_m, y_m, z_m, speed, use_aik, traj_method, phi_d=None):
    global curr_joints
    curr_ee, _ = model.calc_forward_kinematics(curr_joints)
    dx = x_m - curr_ee.x
    dy = y_m - curr_ee.y
    dz = z_m - curr_ee.z
    dist = math.sqrt(dx*dx + dy*dy + dz*dz)
    if dist < 0.001:
        return

    T = max(dist / speed, DT * 2)
    nsteps = max(2, int(T / DT))

    q0 = np.array(curr_joints)
    qf_list = solve_ik(make_ee(x_m, y_m, z_m), use_aik, phi_d=phi_d)
    if qf_list is None or len(qf_list) == 0:
        print(f"[SIM] IK failed for ({x_m:.4f}, {y_m:.4f}, {z_m:.4f})")
        return
    qf = np.array(qf_list)

    ndof = len(q0)
    if traj_method.lower() == "quintic":
        gen = QuinticPolynomial(ndof=ndof)
    elif traj_method.lower() == "trapezoidal":
        gen = TrapezoidalTrajectory(ndof=ndof)
    else:
        gen = CubicPolynomial(ndof=ndof)

    gen.solve(q0, qf, None, None, T)
    t_arr, X = gen.generate(t0=0, tf=T, nsteps=nsteps)

    for k in range(nsteps):
        if not running or _move_cancel:
            break
        curr_joints = X[:, 0, k].tolist()
        push_joints(curr_joints)
        time.sleep(T / nsteps)


def move_to_traj_joints(q_target, speed, traj_method):
    global curr_joints
    q0 = np.array(curr_joints)
    qf = np.array(q_target)
    dist = np.linalg.norm(qf - q0)
    if dist < 0.001:
        return
    T = max(dist / speed, DT * 2)
    nsteps = max(2, int(T / DT))
    ndof = len(q0)
    if traj_method == "Quintic":
        gen = QuinticPolynomial(ndof=ndof)
    elif traj_method == "Trapezoidal":
        gen = TrapezoidalTrajectory(ndof=ndof)
    else:
        gen = CubicPolynomial(ndof=ndof)
    gen.solve(q0, qf, None, None, T)
    t_arr, X = gen.generate(t0=0, tf=T, nsteps=nsteps)
    for k in range(nsteps):
        if not running or _move_cancel:
            break
        curr_joints = X[:, 0, k].tolist()
        push_joints(curr_joints)
        time.sleep(T / nsteps)


def handle(conn):
    global curr_joints, running

    curr_joints = HOME_JOINTS_RAD[:]
    push_joints(curr_joints)

    home_ee, _ = model.calc_forward_kinematics(curr_joints)
    print(f"[SIM] Home EE: x={home_ee.x:.4f}  y={home_ee.y:.4f}  z={home_ee.z:.4f}")

    buf = ""
    try:
        while True:
            ready, _, _ = select.select([conn], [], [], 0.02)
            if not ready:
                continue
            try:
                data = conn.recv(4096).decode()
            except Exception:
                break
            if not data:
                break
            buf += data
            while "\n" in buf:
                line, buf = buf.split("\n", 1)
                msg = json.loads(line)

                if msg["cmd"] == "stop":
                    _cancel_move()
                    conn.sendall(b'{"status":"stopped"}\n')

                elif msg["cmd"] == "home":
                    _cancel_move()
                    curr_joints = HOME_JOINTS_RAD[:]
                    push_joints(curr_joints)
                    conn.sendall(b'{"status":"homed"}\n')

                elif msg["cmd"] == "move_xyz":
                    _cancel_move()
                    x_m, y_m, z_m = remap(msg["x_mm"], msg["y_mm"], msg["z_mm"])
                    speed = msg["speed_mms"] / 1000.0
                    use_aik = msg["use_aik"]
                    traj_method = msg.get("traj_method", "Trapezoidal")
                    phi_d = msg.get("phi_d", None)
                    print(f"[SIM] move_xyz -> x={x_m:.4f}  y={y_m:.4f}  z={z_m:.4f}  traj={traj_method}")
                    def _do_xyz(x_m=x_m, y_m=y_m, z_m=z_m, speed=speed, use_aik=use_aik, traj_method=traj_method, phi_d=phi_d):
                        global _move_cancel
                        _move_cancel = False
                        move_to_traj(x_m, y_m, z_m, speed, use_aik, traj_method, phi_d=phi_d)
                        final_ee, _ = model.calc_forward_kinematics(curr_joints)
                        print(f"[SIM] joints -> {[round(np.rad2deg(j), 2) for j in curr_joints]}")
                        print(f"[SIM] EE -> x={final_ee.x:.4f}  y={final_ee.y:.4f}  z={final_ee.z:.4f}")
                        conn.sendall(b'{"status":"done"}\n')
                    threading.Thread(target=_do_xyz, daemon=True).start()

                elif msg["cmd"] == "move_joints":
                    _cancel_move()
                    joints_deg = msg["joints"]
                    speed = msg["speed_mms"] / 1000.0
                    traj_method = msg.get("traj_method", "Trapezoidal")
                    q_target = [np.deg2rad(j) for j in joints_deg]
                    print(f"[SIM] move_joints -> {joints_deg}  traj={traj_method}")
                    def _do_joints(q_target=q_target, speed=speed, traj_method=traj_method):
                        global _move_cancel
                        _move_cancel = False
                        move_to_traj_joints(q_target, speed, traj_method)
                        final_ee, _ = model.calc_forward_kinematics(curr_joints)
                        print(f"[SIM] EE -> x={final_ee.x:.4f}  y={final_ee.y:.4f}  z={final_ee.z:.4f}")
                        conn.sendall(b'{"status":"done"}\n')
                    threading.Thread(target=_do_joints, daemon=True).start()

                elif msg["cmd"] in ("simulate", "gripper"):
                    conn.sendall(b'{"status":"done"}\n')

    except Exception as e:
        print(f"[SIM ERROR] {e}")
    finally:
        conn.close()


def socket_thread():
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind((HOST, PORT))
        s.listen(1)
        print(f"[SIM] Listening on {HOST}:{PORT} — connect GUI with host = 'localhost'")
        while True:
            conn, addr = s.accept()
            print(f"[SIM] GUI connected: {addr}")
            handle(conn)
            print("[SIM] GUI disconnected, waiting...")


def run_visualizer():
    global model

    model = FiveDOFRobot()
    curr = HOME_JOINTS_RAD[:]

    fig = plt.figure(figsize=(9, 8))
    ax = fig.add_subplot(111, projection='3d')
    plt.ion()
    plt.show()

    threading.Thread(target=socket_thread, daemon=True).start()

    link_line, = ax.plot([], [], [], 'k-', linewidth=3)
    joint_dots, = ax.plot([], [], [], 'mo', markersize=10)
    ee_dot, = ax.plot([], [], [], 'bo', markersize=8)
    title = ax.set_title("")

    ax.set_xlim(-0.65, 0.65)
    ax.set_ylim(-0.65, 0.65)
    ax.set_zlim(0, 0.8)
    ax.set_xlabel('X [m]')
    ax.set_ylabel('Y [m]')
    ax.set_zlabel('Z [m]')

    def redraw(joints_rad):
        pts = get_joint_positions(joints_rad)
        xs = [p[0] for p in pts]
        ys = [p[1] for p in pts]
        zs = [p[2] for p in pts]
        link_line.set_data(xs, ys)
        link_line.set_3d_properties(zs)
        joint_dots.set_data(xs, ys)
        joint_dots.set_3d_properties(zs)
        ee_dot.set_data([xs[-1]], [ys[-1]])
        ee_dot.set_3d_properties([zs[-1]])
        ee, _ = model.calc_forward_kinematics(joints_rad)
        degs = [round(np.rad2deg(j), 1) for j in joints_rad]
        title.set_text(f"EE: ({ee.x:.3f}, {ee.y:.3f}, {ee.z:.3f}) m\nJoints (deg): {degs}")
        fig.canvas.draw_idle()
        fig.canvas.flush_events()

    redraw(curr)

    while plt.fignum_exists(fig.number):
        try:
            joints = plot_queue.get_nowait()
            curr = joints
            redraw(curr)
        except queue.Empty:
            pass
        plt.pause(0.01)


if __name__ == "__main__":
    run_visualizer()