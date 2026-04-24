from project.object_detection import ArucoCameraTracker
import server_funcs as sf

tracker = ArucoCameraTracker(video_id=0)
block_found = 0
scan_pts = [[],[]] #i just put 2 in here as placeholder FILL IN
home = [] #xyz FILL IN
reverse_home = [] #xyz FILL IN
straight_up = [] #xyz
block_pos = [] #xyz to be filled in during CV pipeline
speed = 0 #use an actual speed here


sf.move_to(home[0], home[1], home[2], speed, True, "Cubic")
sf.robot.open_gripper()
for i,pt in enumerate(scan_pts):
    sf.move_to(pt[0], pt[1], pt[2], speed, True, "Cubic") #pt
    ids, rvecs, tvecs, image = tracker.run()
    #get pos
    if block_pos is not None:
        block_found = 1

if block_found:
    sf.move_to(block_pos[0], block_pos[1], block_pos[2], speed, True, "Cubic") #block_pos
    sf.robot.close_gripper()
    sf.move_to(reverse_home[0], reverse_home[1], reverse_home[2], speed, True, "Cubic") #reverse_home
    sf.robot.open_gripper() #open gripper
    sf.move_to(home[0], home[1], home[2], speed, True, "Cubic") #home
else:
    sf.move_to(straight_up[0], straight_up[1], straight_up[2], speed, True, "Cubic") #straight up 

