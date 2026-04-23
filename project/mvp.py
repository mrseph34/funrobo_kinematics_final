from project.object_detection import ArucoCameraTracker

tracker = ArucoCameraTracker(video_id=0)
block_found = 0
scan_pts = [[],[]] #i just put 2 in here as placeholder FILL IN
home = [] #xyz FILL IN
reverse_home = [] #xyz FILL IN
straight_up = [] #xyz
block_pos = [] #xyz to be filled in during CV pipeline

move_to() #home
set_joints() #open gripper
for i,pt in enumerate(scan_pts):
    move_to() #pt
    ids, rvecs, tvecs, image = tracker.run()
    #get pos
    if block_pos is not None:
        block_found = 1

if block_found:
    move_to() #block_pos
    set_joints() #close gripper
    move_to() #reverse_home
    set_joints() #open gripper
    move_to() #home
else:
    move_to() #straight up 

