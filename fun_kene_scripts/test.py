import time
import cv2
import numpy as np
import matplotlib.pyplot as plt
from object_detection import ArucoCameraTracker

tracker = ArucoCameraTracker(video_id=0)
ids, rvecs, tvecs, image = tracker.run()

if ids is not None:
    print("Use tvecs for robot IK / pick-and-place")