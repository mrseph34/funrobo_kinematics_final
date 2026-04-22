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
_move_gen = 0
_obstacles = []  # list of (x1,y1,z1, x2,y2,z2) in robot frame meters


def _boxes_from_gui(boxes_mm):
    """Convert GUI mm boxes to robot-frame meter boxes."""
    result = []
    for b in boxes_mm:
        x1, y1, z1 = remap(b[0], b[1], b[2])
        x2, y2, z2 = remap(b[3], b[4], b[5])
        result.append((min(x1,x2), min(y1,y2), min(z1,z2), max(x1,x2), max(y1,y2), max(z1,z2)))
    return result


import pinocchio as pin
import pyroboplan.models.hiwonder as hiwonder

_pin_model, _pin_collision_model, _ = hiwonder.load_models()
_pin_data = _pin_model.createData()
_pin_collision_data = _pin_collision_model.createData()


def _collides(joints_rad):
    """Return True if pinocchio detects any collision (obstacles + self) for this config."""
    q = np.array(joints_rad, dtype=float)
    pin.computeCollisions(_pin_model, _pin_data, _pin_collision_model, _pin_collision_data, q, False)
    for res in _pin_collision_data.collisionResults:
        if res.isCollision():
            return True
    if not _obstacles:
        return False
    pts = get_joint_positions(joints_rad)
    for p in pts:
        for (x1,y1,z1,x2,y2,z2) in _obstacles:
            if x1 <= p[0] <= x2 and y1 <= p[1] <= y2 and z1 <= p[2] <= z2:
                return True
    return False


def _rrt_plan(q_start, q_goal, fight=True, max_iter=2000, step=0.15):
    """Simple RRT in joint space. Returns waypoint list or None if blocked and fight=False."""
    if not _obstacles and not _collides(q_goal):
        return [q_start, q_goal]
    lims_lo = np.array([lim[0] for lim in model.joint_limits])
    lims_hi = np.array([lim[1] for lim in model.joint_limits])
    q_start = np.array(q_start)
    q_goal = np.array(q_goal)
    tree = [q_start]
    parent = {0: -1}
    for i in range(max_iter):
        if np.random.rand() < 0.15:
            q_rand = q_goal.copy()
        else:
            q_rand = np.random.uniform(lims_lo, lims_hi)
        dists = [np.linalg.norm(q_rand - n) for n in tree]
        nearest_idx = int(np.argmin(dists))
        q_near = tree[nearest_idx]
        direction = q_rand - q_near
        norm = np.linalg.norm(direction)
        if norm < 1e-6:
            continue
        q_new = q_near + step * direction / norm
        q_new = np.clip(q_new, lims_lo, lims_hi)
        if _collides(q_new.tolist()):
            continue
        tree.append(q_new)
        new_idx = len(tree) - 1
        parent[new_idx] = nearest_idx
        if np.linalg.norm(q_new - q_goal) < step:
            if not _collides(q_goal.tolist()):
                tree.append(q_goal)
                parent[len(tree)-1] = new_idx
                path = []
                idx = len(tree)-1
                while idx != -1:
                    path.append(tree[idx])
                    idx = parent[idx]
                path.reverse()
                return path
    print("[RRT] Failed to find collision-free path" + (", using direct" if fight else ", aborting"))
    return [q_start, q_goal] if fight else None


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
            seed, np.array([ee.x, ee.y, ee.z]), tol=0.001, ilimit=30
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
    global _move_gen
    _move_gen += 1


def move_to_traj(x_m, y_m, z_m, speed, use_aik, traj_method, phi_d=None, gen=0, fight=True):
    global curr_joints
    curr_ee, _ = model.calc_forward_kinematics(curr_joints)
    dx = x_m - curr_ee.x
    dy = y_m - curr_ee.y
    dz = z_m - curr_ee.z
    dist = math.sqrt(dx*dx + dy*dy + dz*dz)
    if dist < 0.001:
        return

    qf_list = solve_ik(make_ee(x_m, y_m, z_m), use_aik, phi_d=phi_d)
    if qf_list is None or len(qf_list) == 0:
        print(f"[SIM] IK failed for ({x_m:.4f}, {y_m:.4f}, {z_m:.4f})")
        return

    waypoints = _rrt_plan(curr_joints, qf_list, fight=fight)
    if waypoints is None:
        print(f"[SIM] Blocked by obstacle, move aborted")
        return

    for wi in range(len(waypoints) - 1):
        if not running or _move_gen != gen:
            break
        q0 = np.array(waypoints[wi])
        qf = np.array(waypoints[wi + 1])
        seg_dist = np.linalg.norm(qf - q0)
        if seg_dist < 1e-4:
            continue
        T = max(seg_dist / speed, DT * 2)
        nsteps = max(2, int(T / DT))
        ndof = len(q0)
        if traj_method.lower() == "quintic":
            gen_ = QuinticPolynomial(ndof=ndof)
        elif traj_method.lower() == "trapezoidal":
            gen_ = TrapezoidalTrajectory(ndof=ndof)
        else:
            gen_ = CubicPolynomial(ndof=ndof)
        gen_.solve(q0, qf, None, None, T)
        t_arr, X = gen_.generate(t0=0, tf=T, nsteps=nsteps)
        for k in range(nsteps):
            if not running or _move_gen != gen:
                break
            curr_joints = X[:, 0, k].tolist()
            push_joints(curr_joints)
            time.sleep(T / nsteps)


def move_to_traj_joints(q_target, speed, traj_method, gen=0, fight=True):
    global curr_joints
    waypoints = _rrt_plan(curr_joints, q_target, fight=fight)
    if waypoints is None:
        print(f"[SIM] Blocked by obstacle, move aborted")
        return
    for wi in range(len(waypoints) - 1):
        if not running or _move_gen != gen:
            break
        q0 = np.array(waypoints[wi])
        qf = np.array(waypoints[wi + 1])
        dist = np.linalg.norm(qf - q0)
        if dist < 1e-4:
            continue
        T = max(dist / speed, DT * 2)
        nsteps = max(2, int(T / DT))
        ndof = len(q0)
        if traj_method == "Quintic":
            gen_ = QuinticPolynomial(ndof=ndof)
        elif traj_method == "Trapezoidal":
            gen_ = TrapezoidalTrajectory(ndof=ndof)
        else:
            gen_ = CubicPolynomial(ndof=ndof)
        gen_.solve(q0, qf, None, None, T)
        t_arr, X = gen_.generate(t0=0, tf=T, nsteps=nsteps)
        for k in range(nsteps):
            if not running or _move_gen != gen:
                break
            curr_joints = X[:, 0, k].tolist()
            push_joints(curr_joints)
            time.sleep(T / nsteps)


def handle(conn):
    global curr_joints, running

    running = True
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
                    fight = msg.get("fight_obstacles", False)
                    print(f"[SIM] move_xyz -> x={x_m:.4f}  y={y_m:.4f}  z={z_m:.4f}  traj={traj_method}")
                    my_gen = _move_gen
                    def _do_xyz(x_m=x_m, y_m=y_m, z_m=z_m, speed=speed, use_aik=use_aik, traj_method=traj_method, phi_d=phi_d, my_gen=my_gen, fight=fight):
                        move_to_traj(x_m, y_m, z_m, speed, use_aik, traj_method, phi_d=phi_d, gen=my_gen, fight=fight)
                        if _move_gen == my_gen:
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
                    fight = msg.get("fight_obstacles", False)
                    print(f"[SIM] move_joints -> {joints_deg}  traj={traj_method}")
                    my_gen = _move_gen
                    def _do_joints(q_target=q_target, speed=speed, traj_method=traj_method, my_gen=my_gen, fight=fight):
                        move_to_traj_joints(q_target, speed, traj_method, gen=my_gen, fight=fight)
                        if _move_gen == my_gen:
                            final_ee, _ = model.calc_forward_kinematics(curr_joints)
                            print(f"[SIM] EE -> x={final_ee.x:.4f}  y={final_ee.y:.4f}  z={final_ee.z:.4f}")
                            conn.sendall(b'{"status":"done"}\n')
                    threading.Thread(target=_do_joints, daemon=True).start()

                elif msg["cmd"] in ("simulate", "gripper"):
                    conn.sendall(b'{"status":"done"}\n')

                elif msg["cmd"] == "set_obstacles":
                    _obstacles.clear()
                    _obstacles.extend(_boxes_from_gui(msg.get("boxes", [])))
                    plot_queue.put({"obstacles": list(_obstacles)})
                    conn.sendall(b'{"status":"done"}\n')

    except Exception as e:
        print(f"[SIM ERROR] {e}")
    finally:
        running = False
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
    try:
        fig.canvas.manager.window.wm_geometry("+820+40")
    except Exception:
        pass

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

    box_patches = []

    def draw_boxes(ax):
        for p in box_patches:
            try:
                p.remove()
            except Exception:
                pass
        box_patches.clear()
        for (x1,y1,z1,x2,y2,z2) in _obstacles:
            verts = [
                [(x1,y1,z1),(x2,y1,z1),(x2,y2,z1),(x1,y2,z1)],
                [(x1,y1,z2),(x2,y1,z2),(x2,y2,z2),(x1,y2,z2)],
                [(x1,y1,z1),(x1,y1,z2),(x1,y2,z2),(x1,y2,z1)],
                [(x2,y1,z1),(x2,y1,z2),(x2,y2,z2),(x2,y2,z1)],
                [(x1,y1,z1),(x2,y1,z1),(x2,y1,z2),(x1,y1,z2)],
                [(x1,y2,z1),(x2,y2,z1),(x2,y2,z2),(x1,y2,z2)],
            ]
            from mpl_toolkits.mplot3d.art3d import Poly3DCollection
            poly = Poly3DCollection(verts, alpha=0.15, facecolor='red', edgecolor='red', linewidth=0.5)
            ax.add_collection3d(poly)
            box_patches.append(poly)

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
            item = plot_queue.get_nowait()
            if isinstance(item, dict) and "obstacles" in item:
                draw_boxes(ax)
                fig.canvas.draw_idle()
                fig.canvas.flush_events()
            else:
                curr = item
                redraw(curr)
        except queue.Empty:
            pass
        plt.pause(0.01)


if __name__ == "__main__":
    run_visualizer()