"""
Path Planner Sim Server — runs locally on your laptop.
Dumb visualizer: only understands set_joints and set_obstacles.
path_planner_motion.py connects to this and pushes joint states.
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
import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

import funrobo_kinematics.core.utils as ut
from five_dof import FiveDOFRobot

HOST = "localhost"
PORT = 9697
CONTROL_HZ = 20
DT = 1.0 / CONTROL_HZ

HOME_JOINTS_DEG = [0, 0, 90, -30, 0]
HOME_JOINTS_RAD = [np.deg2rad(j) for j in HOME_JOINTS_DEG]

_LINK_RADIUS = 0.03

model = None
plot_queue = queue.Queue()
_obstacles = []
_sim_blocks = []

BOX_COLORS = ["#ff4466", "#ffaa00", "#00e5ff", "#aa44ff", "#44ff88", "#ff8800", "#ff44ff"]


def remap(x_mm, y_mm, z_mm):
    return -z_mm / 1000.0, x_mm / 1000.0, y_mm / 1000.0


def _boxes_from_gui(boxes_mm):
    result = []
    for b in boxes_mm:
        x1, y1, z1 = remap(b[0], b[1], b[2])
        x2, y2, z2 = remap(b[3], b[4], b[5])
        result.append((min(x1,x2), min(y1,y2), min(z1,z2), max(x1,x2), max(y1,y2), max(z1,z2)))
    return result


def get_joint_positions(joints_rad):
    """Return list of [x,y,z] for base + each joint origin using cumulative FK transforms."""
    _, Hlist = model.calc_forward_kinematics(joints_rad)
    pts = [np.array([0.0, 0.0, 0.0])]
    H = np.eye(4)
    for Hi in Hlist:
        H = H @ Hi
        pts.append(H[:3, 3])
    return pts


def handle(conn):
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

                if msg["cmd"] == "set_joints":
                    joints_rad = [np.deg2rad(j) for j in msg["joints"]]
                    plot_queue.put(list(joints_rad))

                elif msg["cmd"] == "set_obstacles":
                    raw = msg.get("boxes", [])
                    _obstacles.clear()
                    _obstacles.extend(_boxes_from_gui(raw))
                    plot_queue.put({"obstacles": list(_obstacles)})

                elif msg["cmd"] == "set_sim_blocks":
                    raw = msg.get("blocks", [])
                    _sim_blocks.clear()
                    _sim_blocks.extend(raw)
                    plot_queue.put({"sim_blocks": list(_sim_blocks)})

    except Exception as e:
        print(f"[SIM ERROR] {e}")
    finally:
        conn.close()


def socket_thread():
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind((HOST, PORT))
        s.listen(1)
        print(f"[SIM] Listening on {HOST}:{PORT}")
        while True:
            conn, addr = s.accept()
            print(f"[SIM] Motion server connected: {addr}")
            handle(conn)
            print("[SIM] Motion server disconnected, waiting...")


def run_visualizer():
    global model

    model = FiveDOFRobot()
    curr = HOME_JOINTS_RAD[:]

    fig = plt.figure(figsize=(9, 8))
    ax = fig.add_subplot(111, projection='3d')
    plt.ion()
    try:
        fig.canvas.manager.window.wm_geometry("+820+40")
    except Exception:
        pass
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

    box_patches = []
    link_patches = []
    sim_block_patches = []

    def draw_sim_blocks():
        for p in sim_block_patches:
            try:
                p.remove()
            except Exception:
                pass
        sim_block_patches.clear()
        BLOCK_SIZE = 0.04  # 40mm cube
        for blk in _sim_blocks:
            bx, by, bz = blk[0], blk[1], blk[2]
            # remap base frame (x right, y up, z forward) to plot frame via remap()
            cx, cy, cz = remap(bx, by, bz)
            h = BLOCK_SIZE / 2
            x1, x2 = cx - h, cx + h
            y1, y2 = cy - h, cy + h
            z1, z2 = cz - h, cz + h
            verts = [
                [(x1,y1,z1),(x2,y1,z1),(x2,y2,z1),(x1,y2,z1)],
                [(x1,y1,z2),(x2,y1,z2),(x2,y2,z2),(x1,y2,z2)],
                [(x1,y1,z1),(x1,y1,z2),(x1,y2,z2),(x1,y2,z1)],
                [(x2,y1,z1),(x2,y1,z2),(x2,y2,z2),(x2,y2,z1)],
                [(x1,y1,z1),(x2,y1,z1),(x2,y1,z2),(x1,y1,z2)],
                [(x1,y2,z1),(x2,y2,z1),(x2,y2,z2),(x1,y2,z2)],
            ]
            poly = Poly3DCollection(verts, alpha=0.08, facecolor="#00e5ff", edgecolor="#00e5ff", linewidth=1.2)
            ax.add_collection3d(poly)
            sim_block_patches.append(poly)


    def draw_boxes():
        for p in box_patches:
            try:
                p.remove()
            except Exception:
                pass
        box_patches.clear()
        for i, (x1,y1,z1,x2,y2,z2) in enumerate(_obstacles):
            color = BOX_COLORS[i % len(BOX_COLORS)]
            verts = [
                [(x1,y1,z1),(x2,y1,z1),(x2,y2,z1),(x1,y2,z1)],
                [(x1,y1,z2),(x2,y1,z2),(x2,y2,z2),(x1,y2,z2)],
                [(x1,y1,z1),(x1,y1,z2),(x1,y2,z2),(x1,y2,z1)],
                [(x2,y1,z1),(x2,y1,z2),(x2,y2,z2),(x2,y2,z1)],
                [(x1,y1,z1),(x2,y1,z1),(x2,y1,z2),(x1,y1,z2)],
                [(x1,y2,z1),(x2,y2,z1),(x2,y2,z2),(x1,y2,z2)],
            ]
            poly = Poly3DCollection(verts, alpha=0.35, facecolor=color, edgecolor=color, linewidth=0.8)
            ax.add_collection3d(poly)
            box_patches.append(poly)

    def draw_link_boxes(pts):
        for p in link_patches:
            try:
                p.remove()
            except Exception:
                pass
        link_patches.clear()
        r = _LINK_RADIUS
        for i in range(len(pts) - 1):
            p0 = np.array(pts[i])
            p1 = np.array(pts[i + 1])
            seg = p1 - p0
            length = np.linalg.norm(seg)
            if length < 1e-6:
                continue
            axis = seg / length

            # build rotation matrix: local z -> axis
            z = np.array([0.0, 0.0, 1.0])
            v = np.cross(z, axis)
            c = np.dot(z, axis)
            if np.linalg.norm(v) < 1e-8:
                R = np.eye(3) if c > 0 else np.diag([1, -1, -1])
            else:
                vx = np.array([[0,-v[2],v[1]],[v[2],0,-v[0]],[-v[1],v[0],0]])
                R = np.eye(3) + vx + vx @ vx * (1 / (1 + c))

            # cylinder body
            theta = np.linspace(0, 2*np.pi, 16)
            zc = np.linspace(0, length, 8)
            tc, zz = np.meshgrid(theta, zc)
            xc = r * np.cos(tc)
            yc = r * np.sin(tc)
            shape = xc.shape
            pts_local = np.stack([xc.ravel(), yc.ravel(), zz.ravel()], axis=1)
            pts_world = (R @ pts_local.T).T + p0
            Xc = pts_world[:,0].reshape(shape)
            Yc = pts_world[:,1].reshape(shape)
            Zc = pts_world[:,2].reshape(shape)
            surf = ax.plot_surface(Xc, Yc, Zc, color="#00aaff", alpha=0.15, linewidth=0, antialiased=False)
            link_patches.append(surf)

            # end caps (hemispheres)
            for cap_center, cap_sign in [(p0, -1), (p1, 1)]:
                phi = np.linspace(0, np.pi/2, 6)
                th = np.linspace(0, 2*np.pi, 16)
                pp, tt = np.meshgrid(phi, th)
                xs = r * np.sin(pp) * np.cos(tt)
                ys = r * np.sin(pp) * np.sin(tt)
                zs = cap_sign * r * np.cos(pp)
                shape2 = xs.shape
                pts_local2 = np.stack([xs.ravel(), ys.ravel(), zs.ravel()], axis=1)
                pts_world2 = (R @ pts_local2.T).T + cap_center
                Xs = pts_world2[:,0].reshape(shape2)
                Ys = pts_world2[:,1].reshape(shape2)
                Zs = pts_world2[:,2].reshape(shape2)
                surf2 = ax.plot_surface(Xs, Ys, Zs, color="#00aaff", alpha=0.15, linewidth=0, antialiased=False)
                link_patches.append(surf2)

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
        draw_boxes()
        draw_sim_blocks()
        draw_link_boxes(pts)
        fig.canvas.draw_idle()
        fig.canvas.flush_events()

    redraw(curr)

    target = curr[:]

    while plt.fignum_exists(fig.number):
        try:
            item = plot_queue.get_nowait()
            if isinstance(item, dict) and "obstacles" in item:
                draw_boxes()
                fig.canvas.draw_idle()
                redraw(curr)
            elif isinstance(item, dict) and "sim_blocks" in item:
                draw_sim_blocks()
                fig.canvas.draw_idle()
                redraw(curr)
            else:
                curr = target[:]
                target = item
        except queue.Empty:
            pass
        curr = [c + 0.25 * (t - c) for c, t in zip(curr, target)]
        redraw(curr)
        plt.pause(0.005)


if __name__ == "__main__":
    run_visualizer()