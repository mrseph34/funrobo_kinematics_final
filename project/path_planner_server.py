import socket
import json
import select
import time
import numpy as np

from funrobo_kinematics_final.funrobo_hiwonder.funrobo_hiwonder.core.hiwonder_nogamepad import HiwonderRobot
from object_detection import ArucoCameraTracker

HOST = "0.0.0.0"
PORT = 9698

robot = None
gripper_angle = 0.0
_moving = False
_last_joints = None
_joint_last_time = 0.0


def handle(conn):
    global robot, gripper_angle, _moving, _last_joints, _joint_last_time
    robot = HiwonderRobot()
    print("[PI] Robot ready")
    buf = ""
    try:
        while True:
            ready, _, _ = select.select([conn], [], [], 0.02)
            if not ready:
                if _joint_last_time > 0 and time.time() - _joint_last_time >= 1.0:
                    joints_str = "[" + ", ".join(f"{j:.1f}" for j in _last_joints) + "]"
                    print(f"\r[PI JOINTS] {joints_str}")
                    _joint_last_time = 0.0
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

                if msg["cmd"] in ("set_joints", "move_joints"):
                    joints_deg = msg["joints"]
                    if not _moving:
                        _moving = True
                        print("[PI] [STREAMING]", end="", flush=True)
                    _last_joints = joints_deg
                    _joint_last_time = time.time()
                    joints_str = "[" + ", ".join(f"{j:.1f}" for j in joints_deg) + "]"
                    print(f"\r[PI] [STREAMING] {joints_str}   ", end="", flush=True)
                    robot.set_joint_values(joints_deg + [gripper_angle], duration=1.0/20, radians=False)
                    conn.sendall(b'{"status":"done"}\n')

                elif msg["cmd"] == "stop":
                    if _joint_last_time > 0 and _last_joints is not None:
                        joints_str = "[" + ", ".join(f"{j:.1f}" for j in _last_joints) + "]"
                        print(f"\r[PI JOINTS] {joints_str}")
                        _joint_last_time = 0.0
                    _moving = False
                    _last_joints = None

                elif msg["cmd"] == "home":
                    if _moving and _last_joints is not None:
                        print(f"[PI MOVE END] {_last_joints}")
                    _moving = False
                    _last_joints = None
                    print("[PI RECV] home")
                    robot.move_to_home_position()
                    conn.sendall(b'{"status":"homed"}\n')

                elif msg["cmd"] == "set_obstacles":
                    pass

                elif msg["cmd"] == "detect":
                    try:
                        readings = []
                        for _ in range(3):
                            tracker = ArucoCameraTracker(video_id=0)
                            ids, rvecs, tvecs, _ = tracker.run(verbose=False)
                            if ids is not None and len(tvecs) > 0:
                                t = tvecs[0].flatten()
                                readings.append([float(t[0]*1000), float(t[1]*1000), float(t[2]*1000)])
                        if readings:
                            avg = [sum(v[i] for v in readings)/len(readings) for i in range(3)]
                            print(f"[PI DETECT] x={avg[0]:.1f} y={avg[1]:.1f} z={avg[2]:.1f} mm (avg of {len(readings)})")
                            reply = {"cmd": "detect_result", "x_mm": avg[0], "y_mm": avg[1], "z_mm": avg[2]}
                        else:
                            print("[PI DETECT] no marker detected")
                            reply = {"cmd": "detect_result", "error": "no marker"}
                    except Exception as e:
                        reply = {"cmd": "detect_result", "error": str(e)}
                    conn.sendall((json.dumps(reply) + "\n").encode())

                elif msg["cmd"] == "gripper":
                    width = msg.get("width", None)
                    if width is not None:
                        gripper_angle = float(width)
                        robot.set_gripper(gripper_angle)
                    elif msg.get("action") == "open":
                        gripper_angle = robot.open_gripper_angle
                        robot.open_gripper()
                    else:
                        gripper_angle = robot.close_gripper_angle
                        robot.close_gripper()

    except Exception as e:
        print(f"[PI ERROR] {e}")
    finally:
        robot.shutdown_robot()
        conn.close()


def main():
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind((HOST, PORT))
        s.listen(1)
        print(f"[PI] Listening on port {PORT}...")
        while True:
            conn, addr = s.accept()
            print(f"[PI] PC connected: {addr}")
            handle(conn)
            print("[PI] PC disconnected, waiting...")


if __name__ == "__main__":
    main()