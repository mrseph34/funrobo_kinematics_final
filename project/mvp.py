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

block_found = 0
scan_pts = [[0, -30, 50, -120, 0], [-45, -30, 50, -120, 0], [45, -30, 50, -120, 0]] #[[1, 0, 90, -90, 0], [0, -30, 50, -120, 0], [-44, 0, 90, -90, 0], [-45, -30, 50, -120, 0], [44, 0, 90, -90, 0], [45, -30, 50, -120, 0]] #scan joint positions
home = [0, 0, 90, -30, 0] #home joints
dropoff = [0, 200, -150] #xyz dropoff
block_pos = [] #xyz to be filled in during CV pipeline
speed = 300 #mm/s


def run(gui):
    global block_found, block_pos
    block_found = 0
    block_pos = []

    gui._send_atomic({"cmd": "stop"}, {"cmd": "move_joints", "joints": home, "speed_mms": speed, "traj_method": "Cubic", "fight_obstacles": False})
    time.sleep(0.5)

    for i, pt in enumerate(scan_pts):
        gui._send_atomic({"cmd": "stop"}, {"cmd": "move_joints", "joints": pt, "speed_mms": speed, "traj_method": "Cubic", "fight_obstacles": False}) #pt
        time.sleep(0.5)
        gui._mvp_detect_result = None
        gui._mvp_waiting_detect = True
        gui.sock.sendall(((__import__("json").dumps({"cmd": "detect"})) + "\n").encode())
        #get pos
        for _ in range(100):
            time.sleep(0.1)
            if gui._mvp_detect_result is not None:
                break
        result = gui._mvp_detect_result
        gui._mvp_detect_result = None
        gui._mvp_waiting_detect = False
        if result and "x_mm" in result:
            block_pos = [result["x_mm"], result["y_mm"], result["z_mm"]]
            block_found = 1
            break

    if block_found:
        gui._send_gripper({"cmd": "gripper", "action": "open", "width": -100}) #open gripper
        time.sleep(0.5)
        gui._send_atomic({"cmd": "stop"}, {"cmd": "move_xyz", "x_mm": block_pos[0], "y_mm": block_pos[1], "z_mm": block_pos[2], "speed_mms": speed, "use_aik": True, "phi_d": None, "traj_method": "Cubic", "fight_obstacles": False}) #block_pos
        move_ok = _wait_move(gui, timeout=10)
        if move_ok == "ik_failed":
            block_found = 0

    if block_found:
        gui._send_gripper({"cmd": "gripper", "action": "close", "width": -10})
        time.sleep(1)
        gui._send_atomic({"cmd": "stop"}, {"cmd": "move_xyz", "x_mm": dropoff[0], "y_mm": dropoff[1], "z_mm": dropoff[2], "speed_mms": speed, "use_aik": True, "phi_d": None, "traj_method": "Cubic", "fight_obstacles": False}) #dropoff
        _wait_move(gui, timeout=10)
        gui._send_gripper({"cmd": "gripper", "action": "open", "width": -100}) #open gripper
        time.sleep(0.5)
        gui._send_atomic({"cmd": "stop"}, {"cmd": "move_joints", "joints": home, "speed_mms": speed, "traj_method": "Cubic", "fight_obstacles": False}) #home
    else:
        gui.root.after(0, lambda: gui.status.set("MVP: no block found, homing..."))
        gui._send_gripper({"cmd": "gripper", "action": "close", "width": -10}) #close gripper
        time.sleep(0.5)
        gui._send_atomic({"cmd": "stop"}, {"cmd": "move_joints", "joints": home, "speed_mms": speed, "traj_method": "Cubic", "fight_obstacles": False}) #home