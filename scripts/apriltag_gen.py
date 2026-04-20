import cv2
import numpy as np

aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36H11)

marker_id = 1        
marker_size = 400    

marker_img = cv2.aruco.generateImageMarker(aruco_dict, marker_id, marker_size)

cv2.imwrite("/Users/hchiang/Desktop/apriltag.png", marker_img)
print("Saved marker as apriltag.png")