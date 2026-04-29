import time

def _wait_move(gui, timeout=10):
    gui._mvp_move_result = None
    for _ in range(timeout * 10):
        time.sleep(0.1)
        if gui._mvp_move_result is not None:
            break
    result = gui._mvp_move_result
    gui._mvp_move_result = None
    return result

home = [0, 0, 90, -30, 0]
dropoff = [0, 200, -150]
speed = 300

# reference configs (unused but kept for later)
# [j1, 0, 90, -90, 0]
# [j1, -30, 60, -90, 0]
# [j1, -30, 50, -120, 0]
# [j1, 0, 90, -90, 0]

scan_pts = [
    [-90, -30, 60, -120, 0],
    [-90, -50, 40, -40, 0],
    [0, -30, 60, -120, 0],
    [0, -50, 40, -40, 0],
    [90, -30, 60, -120, 0],
    [90, -50, 40, -40, 0],
]

def _do_detect(gui):
    gui._mvp_detect_result = None
    gui._mvp_waiting_detect = True
    if getattr(gui, '_sim_mvp', None) and gui._sim_mvp.get() and gui._sim_sock:
        bx, by, bz = gui._sim_blocks[0]
        gui._sim_sock.sendall((__import__("json").dumps({"cmd": "sim_detect", "x_mm": bx, "y_mm": by, "z_mm": bz}) + "\n").encode())
    else:
        gui.sock.sendall((__import__("json").dumps({"cmd": "detect"}) + "\n").encode())
    for _ in range(50):
        time.sleep(0.1)
        if gui._mvp_detect_result is not None:
            break
    result = gui._mvp_detect_result
    gui._mvp_detect_result = None
    gui._mvp_waiting_detect = False
    return result

def _celebrate(gui):
    gui._send_atomic({"cmd": "stop"}, {"cmd": "move_joints", "joints": [0, 0, 0, 0, 0], "speed_mms": speed, "traj_method": "Cubic", "fight_obstacles": False})
    time.sleep(1.0)
    for _ in range(3):
        gui._send_atomic({"cmd": "stop"}, {"cmd": "move_joints", "joints": [0, 30, 30, -30, 0], "speed_mms": speed, "traj_method": "Cubic", "fight_obstacles": False})
        gui._send_gripper({"cmd": "gripper", "action": "open", "width": -100})
        time.sleep(0.6)
        gui._send_atomic({"cmd": "stop"}, {"cmd": "move_joints", "joints": [0, 30, 30, -120, 0], "speed_mms": speed, "traj_method": "Cubic", "fight_obstacles": False})
        gui._send_gripper({"cmd": "gripper", "action": "close", "width": -10})
        time.sleep(0.6)
    gui._send_atomic({"cmd": "stop"}, {"cmd": "move_joints", "joints": home, "speed_mms": speed, "traj_method": "Cubic", "fight_obstacles": False})

def _no_block_dance(gui):
    gui.root.after(0, lambda: gui.status.set("MVP2: no block found, searching..."))
    for _ in range(3):
        gui._send_atomic({"cmd": "stop"}, {"cmd": "move_joints", "joints": [-30, -30, 60, -120, 0], "speed_mms": speed, "traj_method": "Cubic", "fight_obstacles": False})
        time.sleep(0.5)
        gui._send_atomic({"cmd": "stop"}, {"cmd": "move_joints", "joints": [30, -30, 60, -120, 0], "speed_mms": speed, "traj_method": "Cubic", "fight_obstacles": False})
        time.sleep(0.5)
    gui._send_atomic({"cmd": "stop"}, {"cmd": "move_joints", "joints": home, "speed_mms": speed, "traj_method": "Cubic", "fight_obstacles": False})

def run(gui, sim_scan_pt=None):
    block_found = 0
    block_pos = []

    gui._send_atomic({"cmd": "stop"}, {"cmd": "move_joints", "joints": home, "speed_mms": speed, "traj_method": "Cubic", "fight_obstacles": False})
    time.sleep(0.5)

    for pt in scan_pts:
        gui._send_atomic({"cmd": "stop"}, {"cmd": "move_joints", "joints": pt, "speed_mms": speed, "traj_method": "Cubic", "fight_obstacles": False})
        time.sleep(0.5)
        if getattr(gui, '_sim_mvp', None) and gui._sim_mvp.get():
            result = _do_detect(gui) if (sim_scan_pt is not None and pt == sim_scan_pt) else None
        else:
            result = _do_detect(gui)
        if result and "x_mm" in result:
            block_pos = [result["x_mm"], result["y_mm"], result["z_mm"]]
            block_found = 1
            break

    if block_found:
        gui._send_gripper({"cmd": "gripper", "action": "open", "width": -100})
        time.sleep(0.5)
        gui._send_atomic({"cmd": "stop"}, {"cmd": "move_xyz", "x_mm": block_pos[0], "y_mm": block_pos[1], "z_mm": block_pos[2], "speed_mms": speed, "use_aik": True, "phi_d": None, "traj_method": "Cubic", "fight_obstacles": False})
        move_ok = _wait_move(gui, timeout=10)
        if move_ok == "ik_failed":
            block_found = 0

    if block_found:
        gui._send_gripper({"cmd": "gripper", "action": "close", "width": -10})
        time.sleep(1)
        gui._send_atomic({"cmd": "stop"}, {"cmd": "move_xyz", "x_mm": dropoff[0], "y_mm": dropoff[1], "z_mm": dropoff[2], "speed_mms": speed, "use_aik": True, "phi_d": None, "traj_method": "Cubic", "fight_obstacles": False})
        _wait_move(gui, timeout=10)
        gui._send_gripper({"cmd": "gripper", "action": "open", "width": -100})
        time.sleep(0.5)
        _celebrate(gui)
    else:
        _no_block_dance(gui)