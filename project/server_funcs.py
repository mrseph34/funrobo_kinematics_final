import socket
import json
import time
import math
import numpy as np
import pickle
import threading
import select

from funrobo_kinematics_final.project.five_dof import FiveDOFRobot
from funrobo_kinematics_final.funrobo_hiwonder.funrobo_hiwonder.core.hiwonder_nogamepad import HiwonderRobot
import funrobo_kinematics.core.utils as ut
from funrobo_kinematics_final.examples.traj_gen import CubicPolynomial, QuinticPolynomial, TrapezoidalTrajectory

HOST = "0.0.0.0"
PORT = 9999
CONTROL_HZ = 20
DT = 1.0 / CONTROL_HZ
JOINT_DELTA_CLAMP_DEG = 15.0
CART_STEP_M = 0.002

robot = None
model = None
curr_joints = None
gripper_angle = 0.0
_move_cancel = False
_move_thread = None


def remap(x_mm, y_mm, z_mm):
    return -z_mm / 1000.0, x_mm / 1000.0, y_mm / 1000.0


HOME_JOINTS_RAD = [0.0, 0.0, 0.0, 0.0, 0.0]
IK_TABLE_PATH = "/home/pi/ik_kdtree.pkl"
_ik_tree = None
_ik_joints = None

def load_ik_table():
    global _ik_tree, _ik_joints
    try:
        print(f"[IK] Loading KD-tree from {IK_TABLE_PATH}...")
        with open(IK_TABLE_PATH, "rb") as f:
            data = pickle.load(f)
        _ik_tree = data["tree"]
        _ik_joints = data["joints"]
        print(f"[IK] Loaded {len(_ik_joints):,} samples.")
    except Exception as e:
        print(f"[IK] KD-tree load failed: {e} — falling back to AIK/NIK")

def solve_ik_kdtree(ee):
    if _ik_tree is None:
        return None
    _, idx = _ik_tree.query([ee.x, ee.y, ee.z])
    return _ik_joints[idx].tolist()

def solve_ik(ee, use_aik):
    result = solve_ik_kdtree(ee)
    if result is not None:
        return result
    if use_aik:
        result = model.calc_inverse_kinematics(ee, HOME_JOINTS_RAD)
    else:
        result = model._newton_raphson_step(
            HOME_JOINTS_RAD, np.array([ee.x, ee.y, ee.z]), tol=0.001, ilimit=30
        )
    return None if (result is None or len(result) == 0) else result


def make_traj(method_name, ndof):
    if method_name == "Cubic":
        return CubicPolynomial(ndof=ndof)
    elif method_name == "Quintic":
        return QuinticPolynomial(ndof=ndof)
    else:
        return TrapezoidalTrajectory(ndof=ndof)


def _cancel_move():
    global _move_cancel
    _move_cancel = True
    if _move_thread and _move_thread.is_alive():
        _move_thread.join(timeout=0.3)


def move_to(x_m, y_m, z_m, speed, use_aik, traj_method, on_done=None):
    global curr_joints, _move_cancel, _move_thread

    curr_ee, _ = model.calc_forward_kinematics(curr_joints)
    dist = math.sqrt((x_m - curr_ee.x)**2 + (y_m - curr_ee.y)**2 + (z_m - curr_ee.z)**2)
    if dist < 0.002:
        return

    target_ee = ut.EndEffector()
    target_ee.x, target_ee.y, target_ee.z = x_m, y_m, z_m
    target_joints = solve_ik(target_ee, use_aik)
    if target_joints is None:
        print(f"[SERVER] IK failed for target ({x_m:.4f}, {y_m:.4f}, {z_m:.4f})")
        return

    T = max(dist / speed, 0.5)
    ndof = len(curr_joints)
    traj = make_traj(traj_method, ndof)
    traj.solve(q0=curr_joints, qf=target_joints, qd0=None, qdf=None, T=T)
    n_steps = max(2, int(T / DT))
    t_arr, X = traj.generate(t0=0, tf=T, nsteps=n_steps)

    _move_cancel = True
    if _move_thread and _move_thread.is_alive():
        _move_thread.join(timeout=0.3)
    _move_cancel = False

    def _run():
        global curr_joints
        for i in range(n_steps):
            if _move_cancel:
                break
            q = X[:, 0, i].tolist()
            curr_joints = q
            robot.set_joint_values([np.rad2deg(j) for j in curr_joints] + [gripper_angle], duration=DT, radians=False)
            time.sleep(DT)
        time.sleep(0.3)
        curr_joints = [np.deg2rad(j) for j in robot.get_joint_values()[:5]]
        if on_done:
            on_done()

    _move_thread = threading.Thread(target=_run, daemon=True)
    _move_thread.start()


def simulate(x_m, y_m, z_m, speed, use_aik, traj_method):
    curr_ee, _ = model.calc_forward_kinematics(curr_joints)
    dist = math.sqrt((x_m - curr_ee.x)**2 + (y_m - curr_ee.y)**2 + (z_m - curr_ee.z)**2)

    target_ee = ut.EndEffector()
    target_ee.x, target_ee.y, target_ee.z = x_m, y_m, z_m
    target_joints = solve_ik(target_ee, use_aik)
    ik_ok = target_joints is not None

    T = dist / speed if speed > 0 else 0
    n_steps = max(1, int(T / DT))

    max_delta = None
    if ik_ok:
        max_delta = float(max(abs(np.rad2deg(target_joints[j] - curr_joints[j])) for j in range(len(curr_joints))))

    return {
        "status": "sim",
        "ik": "AIK" if use_aik else "NIK",
        "traj": traj_method,
        "dist_mm": round(dist * 1000, 2),
        "n_steps": n_steps,
        "est_time_s": round(T, 3),
        "start": [round(curr_ee.x, 4), round(curr_ee.y, 4), round(curr_ee.z, 4)],
        "target": [round(x_m, 4), round(y_m, 4), round(z_m, 4)],
        "ik_ok": ik_ok,
        "max_joint_delta_deg": round(max_delta, 2) if max_delta is not None else None,
    }


def handle(conn):
    global robot, model, curr_joints

    robot = HiwonderRobot()
    model = FiveDOFRobot()
    load_ik_table()
    curr_joints = [np.deg2rad(j) for j in robot.get_joint_values()[:5]]

    home_ee, _ = model.calc_forward_kinematics(curr_joints)
    print(f"[SERVER] Home EE: x={home_ee.x:.4f}  y={home_ee.y:.4f}  z={home_ee.z:.4f}")

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

                if msg["cmd"] == "home":
                    _cancel_move()
                    def _do_home():
                        global curr_joints
                        robot.move_to_home_position()
                        time.sleep(0.5)
                        curr_joints = [np.deg2rad(j) for j in robot.get_joint_values()[:5]]
                        conn.sendall(b'{"status":"homed"}\n')
                    threading.Thread(target=_do_home, daemon=True).start()

                elif msg["cmd"] == "gripper":
                    global gripper_angle
                    width = msg.get("width", None)
                    if width is not None:
                        gripper_angle = float(width)
                        robot.set_gripper(gripper_angle)
                    elif msg["action"] == "open":
                        gripper_angle = robot.open_gripper_angle
                        robot.open_gripper()
                    else:
                        gripper_angle = robot.close_gripper_angle
                        robot.close_gripper()
                    conn.sendall(b'{"status":"done"}\n')

                elif msg["cmd"] == "simulate":
                    x_m, y_m, z_m = remap(msg["x_mm"], msg["y_mm"], msg["z_mm"])
                    speed = msg["speed_mms"] / 1000.0
                    result = simulate(x_m, y_m, z_m, speed, msg["use_aik"], msg["traj_method"])
                    conn.sendall((json.dumps(result) + "\n").encode())

                elif msg["cmd"] == "move_joints":
                    _cancel_move()
                    target_joints = [np.deg2rad(j) for j in msg["joints"]]
                    speed = msg["speed_mms"] / 1000.0
                    traj_method = msg["traj_method"]
                    print(f"[SERVER] move_joints -> {msg['joints']}")
                    ndof = len(curr_joints)
                    curr_ee, _ = model.calc_forward_kinematics(curr_joints)
                    target_ee, _ = model.calc_forward_kinematics(target_joints)
                    dist = math.sqrt((target_ee.x - curr_ee.x)**2 + (target_ee.y - curr_ee.y)**2 + (target_ee.z - curr_ee.z)**2)
                    T = max(dist / speed, 0.5)
                    traj = make_traj(traj_method, ndof)
                    traj.solve(q0=curr_joints, qf=target_joints, qd0=None, qdf=None, T=T)
                    n_steps = max(1, int(T / DT))
                    t_arr, X = traj.generate(t0=0, tf=T, nsteps=n_steps)
                    def _do_joints(X=X, n_steps=n_steps, ndof=ndof):
                        global curr_joints, _move_cancel
                        _move_cancel = False
                        for i in range(n_steps):
                            if _move_cancel:
                                break
                            q = X[:, 0, i].tolist()
                            if max(abs(np.rad2deg(q[j] - curr_joints[j])) for j in range(ndof)) > JOINT_DELTA_CLAMP_DEG:
                                continue
                            curr_joints = q
                            robot.set_joint_values([np.rad2deg(j) for j in curr_joints] + [gripper_angle], duration=DT, radians=False)
                            time.sleep(DT)
                        time.sleep(0.3)
                        curr_joints = [np.deg2rad(j) for j in robot.get_joint_values()[:5]]
                        final_ee, _ = model.calc_forward_kinematics(curr_joints)
                        print(f"[SERVER] EE -> x={final_ee.x:.4f}  y={final_ee.y:.4f}  z={final_ee.z:.4f}")
                        conn.sendall(b'{"status":"done"}\n')
                    threading.Thread(target=_do_joints, daemon=True).start()

                elif msg["cmd"] == "move_xyz":
                    x_m, y_m, z_m = remap(msg["x_mm"], msg["y_mm"], msg["z_mm"])
                    speed = msg["speed_mms"] / 1000.0
                    print(f"[SERVER] move_xyz -> x={x_m:.4f}  y={y_m:.4f}  z={z_m:.4f}  traj={msg['traj_method']}")
                    def _xyz_done():
                        print(f"[SERVER] joints -> {[round(np.rad2deg(j), 2) for j in curr_joints]}")
                    move_to(x_m, y_m, z_m, speed, msg["use_aik"], msg["traj_method"], on_done=_xyz_done)
                    conn.sendall(b'{"status":"done"}\n')

    except Exception as e:
        print(f"[ERROR] {e}")
    finally:
        robot.shutdown_robot()
        conn.close()


def main():
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind((HOST, PORT))
        s.listen(1)
        print(f"[SERVER] Listening on port {PORT}...")
        while True:
            conn, addr = s.accept()
            print(f"[SERVER] Connected: {addr}")
            handle(conn)
            print("[SERVER] Client disconnected, waiting...")


if __name__ == "__main__":
    main()