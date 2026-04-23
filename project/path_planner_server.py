import socket
import json
import select
import numpy as np

from funrobo_kinematics_final.funrobo_hiwonder.funrobo_hiwonder.core.hiwonder_nogamepad import HiwonderRobot

HOST = "0.0.0.0"
PORT = 9998

robot = None
gripper_angle = 0.0


def handle(conn):
    global robot, gripper_angle
    robot = HiwonderRobot()
    print("[PI] Robot ready")
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
                    joints_deg = msg["joints"]
                    robot.set_joint_values(joints_deg + [gripper_angle], duration=1.0/20, radians=False)

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