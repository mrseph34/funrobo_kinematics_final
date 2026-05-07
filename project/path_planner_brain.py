import sys
import os
sys.path.append(os.path.abspath("examples"))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import socket
import json
import time
import threading
import select
import numpy as np
import math

from traj_gen import CubicPolynomial, QuinticPolynomial, TrapezoidalTrajectory
import funrobo_kinematics.core.utils as ut
from five_dof import FiveDOFRobot

HOST = "localhost"
PORT = 9999
SIM_HOST = "localhost"
SIM_PORT = 9697
CONTROL_HZ = 20
DT = 1.0 / CONTROL_HZ
CAM_OFFSET_M = (0, 0, 0)
JOINT_DELTA_CLAMP_DEG = 15.0
CLAMP_ENABLED = False

HOME_JOINTS_DEG = [0, 0, 90, -30, 0]
HOME_JOINTS_RAD = [np.deg2rad(j) for j in HOME_JOINTS_DEG]

_LINK_RADIUS = 0.01

model = None
curr_joints = None
running = False
_move_gen = 0
_obstacles = []
_raw_obstacles = []

_gripper_angle = 0.0
_sim_sock = None
_gui_conn = None
_sim_lock = threading.Lock()
_gui_lock = threading.Lock()



def _connect_sim():
    global _sim_sock
    if _sim_sock is not None:
        return
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.settimeout(0.3)
        s.connect((SIM_HOST, SIM_PORT))
        s.settimeout(None)
        _sim_sock = s
        print(f"[MOTION] Connected to sim at {SIM_HOST}:{SIM_PORT}")
        if _raw_obstacles:
            boxes_copy = list(_raw_obstacles)
            def _delayed_obs():
                time.sleep(1.0)
                _send(_sim_sock, _sim_lock, {"cmd": "set_obstacles", "boxes": boxes_copy})
            threading.Thread(target=_delayed_obs, daemon=True).start()
    except Exception:
        _sim_sock = None


def _send(sock, lock, msg_dict):
    global _sim_sock
    if sock is None:
        return
    data = (json.dumps(msg_dict) + "\n").encode()
    with lock:
        try:
            sock.sendall(data)
        except Exception:
            if sock is _sim_sock:
                _sim_sock = None


def push_joints(joints_rad):
    global _sim_sock
    if _sim_sock is None:
        _connect_sim()
    deg = [round(np.rad2deg(j), 4) for j in joints_rad]
    _send(_sim_sock, _sim_lock, {"cmd": "set_joints", "joints": deg})
    _send(_gui_conn, _gui_lock, {"status": "joints", "joints": deg})


def _boxes_from_gui(boxes_mm):
    result = []
    for b in boxes_mm:
        x1, y1, z1 = remap(b[0], b[1], b[2])
        x2, y2, z2 = remap(b[3], b[4], b[5])
        result.append((min(x1,x2), min(y1,y2), min(z1,z2), max(x1,x2), max(y1,y2), max(z1,z2)))
    return result


def _seg_seg_dist(p1, p2, p3, p4):
    """Minimum distance between two line segments p1-p2 and p3-p4."""
    d1 = p2 - p1
    d2 = p4 - p3
    r = p1 - p3
    a = np.dot(d1, d1)
    e = np.dot(d2, d2)
    f = np.dot(d2, r)
    if a < 1e-10 and e < 1e-10:
        return np.linalg.norm(r)
    if a < 1e-10:
        s, t = 0.0, np.clip(f / e, 0, 1)
    else:
        c = np.dot(d1, r)
        if e < 1e-10:
            s, t = np.clip(-c / a, 0, 1), 0.0
        else:
            b = np.dot(d1, d2)
            denom = a * e - b * b
            if abs(denom) > 1e-10:
                s = np.clip((b * f - c * e) / denom, 0, 1)
            else:
                s = 0.0
            t = (b * s + f) / e
            if t < 0:
                s, t = np.clip(-c / a, 0, 1), 0.0
            elif t > 1:
                s, t = np.clip((b - c) / a, 0, 1), 1.0
    closest1 = p1 + s * d1
    closest2 = p3 + t * d2
    return np.linalg.norm(closest1 - closest2)


def _self_collides(joints_rad):
    """Check capsule self-collision between non-adjacent link segments."""
    pts = [np.array(p) for p in get_joint_positions(joints_rad)]
    for i in range(len(pts) - 1):
        for j in range(i + 3, len(pts) - 1):
            d = _seg_seg_dist(pts[i], pts[i+1], pts[j], pts[j+1])
            if d < 2 * _LINK_RADIUS:
                return True
    return False


def _box_collides(joints_rad):
    """Check link segments against GUI obstacle boxes."""
    if not _obstacles:
        return False
    pts = [np.array(p) for p in get_joint_positions(joints_rad)]
    for (x1, y1, z1, x2, y2, z2) in _obstacles:
        cx, cy, cz = (x1+x2)/2, (y1+y2)/2, (z1+z2)/2
        hx, hy, hz = abs(x2-x1)/2, abs(y2-y1)/2, abs(z2-z1)/2
        for i in range(len(pts) - 1):
            for t in np.linspace(0, 1, 8):
                pt = pts[i] + t * (pts[i+1] - pts[i])
                if (abs(pt[0]-cx) <= hx + _LINK_RADIUS and
                        abs(pt[1]-cy) <= hy + _LINK_RADIUS and
                        abs(pt[2]-cz) <= hz + _LINK_RADIUS):
                    return True
    return False


def _collides(joints_rad):
    return _self_collides(joints_rad) or _box_collides(joints_rad)


def _rrt_plan(q_start, q_goal, fight=False, max_iter=2000, step=0.05):
    """Simple RRT in joint space. Returns waypoint list or None if blocked and fight=False."""
    if _collides(q_goal):
        print(f"[RRT] Goal config is in collision — move aborted")
        return [q_start, q_goal] if fight else None
    if not _obstacles:
        return [q_start, q_goal]
    # check if direct path is clear before running RRT
    q_s = np.array(q_start)
    q_g = np.array(q_goal)
    direct_clear = True
    for t in np.linspace(0, 1, 20):
        if _collides((q_s + t * (q_g - q_s)).tolist()):
            direct_clear = False
            break
    if direct_clear:
        print(f"[RRT] Direct path clear, skipping RRT")
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


def _cancel_move():
    global _move_gen
    _move_gen += 1


def move_to_traj(x_m, y_m, z_m, speed, use_aik, traj_method, phi_d=None, gen=0, fight=False):
    global curr_joints
    curr_ee, _ = model.calc_forward_kinematics(curr_joints)
    dx = x_m - curr_ee.x
    dy = y_m - curr_ee.y
    dz = z_m - curr_ee.z
    dist = math.sqrt(dx*dx + dy*dy + dz*dz)
    if dist < 0.001:
        return

    if _move_gen != gen:
        return
    qf_list = solve_ik(make_ee(x_m, y_m, z_m), use_aik, phi_d=phi_d)
    if qf_list is None or len(qf_list) == 0:
        print(f"[MOTION] IK failed for ({x_m:.4f}, {y_m:.4f}, {z_m:.4f})")
        return False
    for i in range(len(qf_list)):
        diff = qf_list[i] - curr_joints[i]
        if diff > np.pi:
            qf_list[i] -= 2 * np.pi
        elif diff < -np.pi:
            qf_list[i] += 2 * np.pi

    if _move_gen != gen:
        return
    T = max(dist / speed, DT * 2)
    ndof = len(curr_joints)

    print(f"[MOTION] move_xyz: dist={dist*1000:.1f}mm speed={speed*1000:.1f}mm/s T={T:.3f}s traj={traj_method}")

    waypoints = _rrt_plan(curr_joints, qf_list, fight=fight)
    if waypoints is None:
        print(f"[MOTION] Blocked by obstacle, move aborted")
        return

    if _move_gen != gen:
        return
    total_jdist = max(np.linalg.norm(np.array(waypoints[-1]) - np.array(waypoints[0])), 1e-6)
    print(f"[MOTION] move_xyz: {len(waypoints)-1} segment(s)")
    for wi in range(len(waypoints) - 1):
        if not running or _move_gen != gen:
            print(f"[MOTION] move_xyz: cancelled at segment {wi}")
            break
        q0 = np.array(waypoints[wi])
        qf = np.array(waypoints[wi + 1])
        seg_dist = np.linalg.norm(qf - q0)
        if seg_dist < 1e-4:
            continue
        seg_T = T * (seg_dist / total_jdist)
        seg_T = max(seg_T, DT * 2)
        max_joint_delta_deg = max(abs(np.rad2deg(qf[j] - q0[j])) for j in range(len(q0)))
        steps_for_clamp = max(2, int(np.ceil(max_joint_delta_deg / JOINT_DELTA_CLAMP_DEG * 2.0)))
        seg_steps = max(steps_for_clamp, int(seg_T / DT))
        step_dt = seg_T / seg_steps
        # print(f"[MOTION]   seg {wi}: seg_T={seg_T:.3f}s steps={seg_steps} step_dt={step_dt*1000:.1f}ms")
        if _move_gen != gen:
            break
        if traj_method.lower() == "quintic":
            gen_ = QuinticPolynomial(ndof=ndof)
        elif traj_method.lower() == "trapezoidal":
            gen_ = TrapezoidalTrajectory(ndof=ndof)
        else:
            gen_ = CubicPolynomial(ndof=ndof)
        gen_.solve(q0, qf, None, None, seg_T)
        t_arr, X = gen_.generate(t0=0, tf=seg_T, nsteps=seg_steps)
        next_t = time.time()
        for k in range(seg_steps):
            if not running or _move_gen != gen:
                break
            q_next = X[:, 0, k].tolist()
            if CLAMP_ENABLED:
                delta = max(abs(np.rad2deg(q_next[j] - curr_joints[j])) for j in range(len(curr_joints)))
                if delta > JOINT_DELTA_CLAMP_DEG:
                    # print(f"[MOTION]   step {k} clamped: delta={delta:.2f}deg")
                    continue
            curr_joints = q_next
            push_joints(curr_joints)
            next_t += step_dt
            time.sleep(max(0, next_t - time.time()))


def move_to_traj_joints(q_target, speed, traj_method, gen=0, fight=False):
    global curr_joints
    curr_ee, _ = model.calc_forward_kinematics(curr_joints)
    goal_ee, _ = model.calc_forward_kinematics(q_target)
    ee_dist = math.sqrt((goal_ee.x-curr_ee.x)**2 + (goal_ee.y-curr_ee.y)**2 + (goal_ee.z-curr_ee.z)**2)
    total_jdist = np.linalg.norm(np.array(q_target) - np.array(curr_joints))
    T = max(ee_dist / max(speed, 1e-6), DT * 2)
    ndof = len(curr_joints)

    print(f"[MOTION] move_joints: ee_dist={ee_dist*1000:.1f}mm speed={speed*1000:.1f}mm/s T={T:.3f}s traj={traj_method}")
    print(f"[MOTION] move_joints: target_deg={[round(math.degrees(j),1) for j in q_target]}")

    if _move_gen != gen:
        return
    waypoints = _rrt_plan(curr_joints, q_target, fight=fight)
    if waypoints is None:
        print(f"[MOTION] Blocked by obstacle, move aborted")
        return

    if _move_gen != gen:
        return
    total_jdist = max(np.linalg.norm(np.array(waypoints[-1]) - np.array(waypoints[0])), 1e-6)
    print(f"[MOTION] move_joints: {len(waypoints)-1} segment(s)")
    for wi in range(len(waypoints) - 1):
        if not running or _move_gen != gen:
            print(f"[MOTION] move_joints: cancelled at segment {wi}")
            break
        q0 = np.array(waypoints[wi])
        qf = np.array(waypoints[wi + 1])
        seg_dist = np.linalg.norm(qf - q0)
        if seg_dist < 1e-4:
            continue
        seg_T = T * (seg_dist / total_jdist)
        seg_T = max(seg_T, DT * 2)
        max_joint_delta_deg = max(abs(np.rad2deg(qf[j] - q0[j])) for j in range(len(q0)))
        steps_for_clamp = max(2, int(np.ceil(max_joint_delta_deg / JOINT_DELTA_CLAMP_DEG * 2.0)))
        seg_steps = max(steps_for_clamp, int(seg_T / DT))
        step_dt = seg_T / seg_steps
        # print(f"[MOTION]   seg {wi}: seg_T={seg_T:.3f}s steps={seg_steps} step_dt={step_dt*1000:.1f}ms")
        if _move_gen != gen:
            break
        if traj_method == "Quintic":
            gen_ = QuinticPolynomial(ndof=ndof)
        elif traj_method == "Trapezoidal":
            gen_ = TrapezoidalTrajectory(ndof=ndof)
        else:
            gen_ = CubicPolynomial(ndof=ndof)
        gen_.solve(q0, qf, None, None, seg_T)
        t_arr, X = gen_.generate(t0=0, tf=seg_T, nsteps=seg_steps)
        next_t = time.time()
        for k in range(seg_steps):
            if not running or _move_gen != gen:
                break
            q_next = X[:, 0, k].tolist()
            if CLAMP_ENABLED:
                delta = max(abs(np.rad2deg(q_next[j] - curr_joints[j])) for j in range(len(curr_joints)))
                if delta > JOINT_DELTA_CLAMP_DEG:
                    # print(f"[MOTION]   step {k} clamped: delta={delta:.2f}deg")
                    continue
            curr_joints = q_next
            push_joints(curr_joints)
            next_t += step_dt
            time.sleep(max(0, next_t - time.time()))


def handle(conn):
    global curr_joints, running, _gui_conn
    _gui_conn = conn

    running = True
    curr_joints = HOME_JOINTS_RAD[:]
    push_joints(curr_joints)

    home_ee, _ = model.calc_forward_kinematics(curr_joints)
    print(f"[MOTION] Home EE: x={home_ee.x:.4f}  y={home_ee.y:.4f}  z={home_ee.z:.4f}")

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
                    print(f"[MOTION] CMD: stop (gen {_move_gen} -> {_move_gen+1})")
                    _cancel_move()
                    conn.sendall(b'{"status":"stopped"}\n')

                elif msg["cmd"] == "home":
                    print(f"[MOTION] CMD: home")
                    _cancel_move()
                    my_gen = _move_gen
                    snap = curr_joints[:]
                    def _do_home(my_gen=my_gen, snap=snap):
                        global curr_joints
                        curr_joints = snap
                        move_to_traj_joints(HOME_JOINTS_RAD[:], 200.0/1000.0, "Trapezoidal", gen=my_gen, fight=False)
                        if _move_gen == my_gen:
                            conn.sendall(b'{"status":"homed"}\n')
                    threading.Thread(target=_do_home, daemon=True).start()

                elif msg["cmd"] == "move_xyz":
                    _cancel_move()
                    x_m, y_m, z_m = remap(msg["x_mm"], msg["y_mm"], msg["z_mm"])
                    speed = msg["speed_mms"] / 1000.0
                    use_aik = msg["use_aik"]
                    traj_method = msg.get("traj_method", "Trapezoidal")
                    phi_d = msg.get("phi_d", None)
                    fight = msg.get("fight_obstacles", False)
                    print(f"[MOTION] CMD: move_xyz raw=({msg['x_mm']},{msg['y_mm']},{msg['z_mm']})mm -> ({x_m:.4f},{y_m:.4f},{z_m:.4f})m speed={msg['speed_mms']}mm/s traj={traj_method}")
                    my_gen = _move_gen
                    snap = curr_joints[:]
                    def _do_xyz(x_m=x_m, y_m=y_m, z_m=z_m, speed=speed, use_aik=use_aik, traj_method=traj_method, phi_d=phi_d, my_gen=my_gen, fight=fight, snap=snap):
                        global curr_joints
                        curr_joints = snap
                        ok = move_to_traj(x_m, y_m, z_m, speed, use_aik, traj_method, phi_d=phi_d, gen=my_gen, fight=fight)
                        if _move_gen == my_gen:
                            if ok is False:
                                conn.sendall(b'{"status":"ik_failed"}\n')
                            else:
                                final_ee, _ = model.calc_forward_kinematics(curr_joints)
                                print(f"[MOTION] EE -> x={final_ee.x:.4f}  y={final_ee.y:.4f}  z={final_ee.z:.4f}")
                                conn.sendall(b'{"status":"done"}\n')
                    threading.Thread(target=_do_xyz, daemon=True).start()

                elif msg["cmd"] == "move_joints":
                    _cancel_move()
                    joints_deg = msg["joints"]
                    speed = msg["speed_mms"] / 1000.0
                    traj_method = msg.get("traj_method", "Trapezoidal")
                    q_target = [np.deg2rad(j) for j in joints_deg]
                    fight = msg.get("fight_obstacles", False)
                    print(f"[MOTION] CMD: move_joints {joints_deg} speed={msg['speed_mms']}mm/s traj={traj_method}")
                    my_gen = _move_gen
                    snap = curr_joints[:]
                    def _do_joints(q_target=q_target, speed=speed, traj_method=traj_method, my_gen=my_gen, fight=fight, snap=snap):
                        global curr_joints
                        curr_joints = snap
                        move_to_traj_joints(q_target, speed, traj_method, gen=my_gen, fight=fight)
                        if _move_gen == my_gen:
                            final_ee, _ = model.calc_forward_kinematics(curr_joints)
                            print(f"[MOTION] EE -> x={final_ee.x:.4f}  y={final_ee.y:.4f}  z={final_ee.z:.4f}")
                            conn.sendall(b'{"status":"done"}\n')
                    threading.Thread(target=_do_joints, daemon=True).start()

                # elif msg["cmd"] == "transform_detect":
                #     j1 = curr_joints[0]
                #     print(f"--------------------j1={np.degrees(j1):.2f}")
                #     x_c, y_c, z_c = msg["x_mm"]/1000, msg["y_mm"]/1000, msg["z_mm"]/1000
                #     print(f"[MOTION] EE CAMMMMMM: x={x_c:.4f} y={y_c:.4f} z={z_c:.4f}")
                #     x_gui_raw = x_c * 1000
                #     z_gui_raw = -y_c * 1000
                #     deg = round(np.degrees(j1))
                #     print(f'deggggggggg: {deg}')
                #     if deg == 45:
                #         x_gui = ((x_gui_raw - z_gui_raw) * 0.7071) - 120
                #         z_gui = ((x_gui_raw + z_gui_raw) * 0.7071) - 70
                #     elif deg == -45:
                #         x_gui = ((x_gui_raw + z_gui_raw) * 0.7071) + 120
                #         z_gui = ((-x_gui_raw + z_gui_raw) * 0.7071) - 50
                #     # if deg == 44:
                #     #     x_gui = ((x_gui_raw - z_gui_raw) * 0.7071) - 90
                #     #     z_gui = ((x_gui_raw + z_gui_raw) * 0.7071) - 50
                #     # elif deg == -44:
                #     #     x_gui = ((x_gui_raw + z_gui_raw) * 0.7071) + 80
                #     #     z_gui = ((-x_gui_raw + z_gui_raw) * 0.7071) - 30
                #     # elif deg == 1:
                #     #     x_gui = x_gui_raw
                #     #     z_gui = z_gui_raw - 50
                #     else:
                #         x_gui = x_gui_raw
                #         z_gui = z_gui_raw
                #     y_gui = 10
                #     print(f"[MOTION] transform_detect: raw=({msg['x_mm']:.1f},{msg['y_mm']:.1f},{msg['z_mm']:.1f}) -> transformed=({x_gui:.1f},{y_gui:.1f},{z_gui:.1f}) mm")
                #     conn.sendall((json.dumps({"cmd": "transform_result", "x_mm": x_gui, "y_mm": y_gui, "z_mm": z_gui}) + "\n").encode())
                
                elif msg["cmd"] == "get_ee":
                    ee, Hlist = model.calc_forward_kinematics(curr_joints)
                    cam_x, cam_y, cam_z = ee.y, ee.z, -ee.x
                    T = np.eye(4)
                    for Hi in Hlist:
                        T = T @ Hi
                    R = T[:3,:3]
                    pitch = (np.degrees(np.arctan2(R[0,2], R[0,0])) - 90) + 180
                    yaw = np.degrees(np.arcsin(np.clip(R[0,1], -1, 1)))
                    roll = 0
                    conn.sendall((json.dumps({"cmd": "ee_result", "cam_x": cam_x, "cam_y": cam_y, "cam_z": cam_z, "pitch": pitch, "yaw": yaw, "roll": roll}) + "\n").encode())
                
                elif msg["cmd"] == "sim_detect":
                    ee, Hlist = model.calc_forward_kinematics(curr_joints)
                    cam_x, cam_y, cam_z = ee.y, ee.z, -ee.x
                    T = np.eye(4)
                    for Hi in Hlist:
                        T = T @ Hi
                    R = T[:3,:3]
                    pitch = (np.degrees(np.arctan2(R[0,2], R[0,0])) - 90) + 180
                    yaw = np.degrees(np.arcsin(np.clip(R[0,1], -1, 1)))
                    roll = 0
                    p, yw, r = np.radians(-pitch), np.radians(yaw), np.radians(roll)
                    Rx = np.array([[1,0,0],[0,np.cos(p),-np.sin(p)],[0,np.sin(p),np.cos(p)]])
                    Ry = np.array([[np.cos(yw),0,np.sin(yw)],[0,1,0],[-np.sin(yw),0,np.cos(yw)]])
                    Rz = np.array([[np.cos(r),-np.sin(r),0],[np.sin(r),np.cos(r),0],[0,0,1]])
                    T_cam = np.eye(4)
                    T_cam[:3,:3] = Ry @ Rx @ Rz
                    T_cam[0,3], T_cam[1,3], T_cam[2,3] = cam_x, cam_y, cam_z
                    R_inv = T_cam[:3,:3].T
                    t_inv = -R_inv @ T_cam[:3,3]
                    p_base = np.array([msg["x_mm"]/1000, msg["y_mm"]/1000, msg["z_mm"]/1000])
                    p_cam = R_inv @ p_base + t_inv
                    x_mm, y_mm, z_mm = p_cam[0]*1000, p_cam[1]*1000, p_cam[2]*1000
                    print(f"[SIM DETECT] block=({msg['x_mm']},{msg['y_mm']},{msg['z_mm']}) -> cam=({x_mm:.1f},{y_mm:.1f},{z_mm:.1f}) mm")
                    x_c, y_c, z_c = x_mm/1000, y_mm/1000, z_mm/1000
                    print(f"[MOTION] EE POSSSSS: x={cam_x:.4f} y={cam_y:.4f} z={cam_z:.4f}")
                    print(f"[MOTION] EE ANGLES: pitch={pitch:.2f} yaw={yaw:.2f} roll={roll:.2f}")
                    print(f"[MOTION] EE CAMMMMMM: x={x_c:.4f} y={y_c:.4f} z={z_c:.4f}")
                    p_base_h = T_cam @ np.array([x_c, y_c, z_c, 1.0])
                    x_gui, y_gui, z_gui = p_base_h[0]*1000, p_base_h[1]*1000, p_base_h[2]*1000
                    print(f"[MOTION] transform_detect: raw=({x_mm:.1f},{y_mm:.1f},{z_mm:.1f}) -> transformed=({x_gui:.1f},{y_gui:.1f},{z_gui:.1f}) mm")
                    conn.sendall((json.dumps({"cmd": "transform_result", "x_mm": x_gui, "y_mm": y_gui, "z_mm": z_gui}) + "\n").encode())

                elif msg["cmd"] == "transform_detect":
                    ee, Hlist = model.calc_forward_kinematics(curr_joints)
                    # y, z, -x
                    cam_x, cam_y, cam_z = ee.y, ee.z, -ee.x
                    T = np.eye(4)
                    for Hi in Hlist:
                        T = T @ Hi
                    R = T[:3,:3]
                    # print(f"R rows: {np.degrees(np.arctan2(R[2,1],R[2,2])):.0f}, {np.degrees(np.arctan2(R[0,2],R[0,0])):.0f}, {np.degrees(np.arctan2(R[1,0],R[0,0])):.0f}, {np.degrees(np.arcsin(-R[2,0])):.0f}, {np.degrees(np.arcsin(R[1,2])):.0f}, {np.degrees(np.arcsin(-R[0,1])):.0f}")
                    # camera forward in base frame
                    pitch = (np.degrees(np.arctan2(R[0,2], R[0,0])) - 90) + 180
                    yaw = np.degrees(np.arcsin(np.clip(R[0,1], -1, 1)))
                    roll = 0 # -np.degrees(np.arctan2(R[2,1], R[2,2]))
                    # from scipy.spatial.transform import Rotation
                    # rot = Rotation.from_matrix(R)
                    # angles = rot.as_euler('xyz', degrees=True)
                    # pitch = angles[0]
                    # yaw = angles[1]  
                    # roll = angles[2]
                    p, yw, r = np.radians(-pitch), np.radians(yaw), np.radians(roll)
                    Rx = np.array([[1,0,0],[0,np.cos(p),-np.sin(p)],[0,np.sin(p),np.cos(p)]])
                    Ry = np.array([[np.cos(yw),0,np.sin(yw)],[0,1,0],[-np.sin(yw),0,np.cos(yw)]])
                    Rz = np.array([[np.cos(r),-np.sin(r),0],[np.sin(r),np.cos(r),0],[0,0,1]])
                    T_cam = np.eye(4)
                    T_cam[:3,:3] = Ry @ Rx @ Rz
                    T_cam[0,3], T_cam[1,3], T_cam[2,3] = cam_x, cam_y, cam_z
                    x_c, y_c, z_c = msg["x_mm"]/1000, msg["y_mm"]/1000, msg["z_mm"]/1000
                    print(f"[MOTION] EE POSSSSS: x={cam_x:.4f} y={cam_y:.4f} z={cam_z:.4f}")
                    print(f"[MOTION] EE ANGLES: pitch={pitch:.2f} yaw={yaw:.2f} roll={roll:.2f}")
                    print(f"[MOTION] EE CAMMMMMM: x={x_c:.4f} y={y_c:.4f} z={z_c:.4f}")
                    p_base_h = T_cam @ np.array([x_c, y_c, z_c, 1.0])
                    x_gui, y_gui, z_gui = p_base_h[0]*1000, p_base_h[1]*1000, p_base_h[2]*1000
                    print(f"[MOTION] transform_detect: raw=({msg['x_mm']:.1f},{msg['y_mm']:.1f},{msg['z_mm']:.1f}) -> transformed=({x_gui:.1f},{y_gui:.1f},{z_gui:.1f}) mm")
                    if yaw > 0:
                        x_gui -= yaw/2
                        z_gui += yaw/2
                    if yaw == 0:
                        x_gui -= 20
                        z_gui -= 50
                    if yaw < 0:
                        x_gui -= 20
                    conn.sendall((json.dumps({"cmd": "transform_result", "x_mm": x_gui, "y_mm": 0, "z_mm": z_gui}) + "\n").encode())

                elif msg["cmd"] == "detect":
                    def _do_detect(snap=curr_joints[:]):
                        try:
                            conn.sendall((json.dumps({"cmd": "detect_result", "error": "Pi not connected"}) + "\n").encode())
                        except Exception as e:
                            conn.sendall((json.dumps({"cmd": "detect_result", "error": str(e)}) + "\n").encode())
                    threading.Thread(target=_do_detect, daemon=True).start()

                elif msg["cmd"] == "gripper":
                    global _gripper_angle
                    action = msg.get("action")
                    width = msg.get("width", None)
                    if action == "open":
                        _gripper_angle = -100.0
                    elif action == "close":
                        _gripper_angle = -10.0
                    elif width is not None:
                        _gripper_angle = float(width)
                    conn.sendall(b'{"status":"done"}\n')

                elif msg["cmd"] == "simulate":
                    conn.sendall(b'{"status":"done"}\n')

                elif msg["cmd"] == "set_obstacles":
                    raw = msg.get("boxes", [])
                    _obstacles.clear()
                    _obstacles.extend(_boxes_from_gui(raw))
                    _raw_obstacles.clear()
                    _raw_obstacles.extend(raw)
                    _connect_sim()
                    _send(_sim_sock, _sim_lock, msg)
                    conn.sendall(b'{"status":"done"}\n')

                elif msg["cmd"] == "set_sim_blocks":
                    _connect_sim()
                    _send(_sim_sock, _sim_lock, msg)

    except Exception as e:
        print(f"[MOTION ERROR] {e}")
    finally:
        running = False
        _gui_conn = None
        conn.close()


def main():
    global model
    model = FiveDOFRobot()
    _connect_sim()
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind((HOST, PORT))
        s.listen(1)
        print(f"[MOTION] Listening on {HOST}:{PORT}")
        while True:
            conn, addr = s.accept()
            print(f"[MOTION] GUI connected: {addr}")
            handle(conn)
            print("[MOTION] GUI disconnected, waiting...")


if __name__ == "__main__":
    main()