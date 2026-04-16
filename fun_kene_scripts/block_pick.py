import numpy as np
from scipy import linalg
import matplotlib.pyplot as plt
np.set_printoptions(
    linewidth=120, formatter={
        'float': lambda x: f"{0:8.4g}" if abs(x) < 1e-10 else f"{x:8.4g}"})
np.random.seed(0)
from machinevisiontoolbox.base import *
from machinevisiontoolbox import *
from spatialmath.base import *
from spatialmath import *
import cv2


import cv2
import numpy as np

video_id = 1 # this is setup-dependent and would need to change. Ranges from 0-10+
cap = cv2.VideoCapture(video_id) 
    

while True:
    ret, frame = cap.read()
    if ret:
        cv2.imshow("frame", frame)

        if cv2.waitKey(1) == ord('q'): # saves the current frame each time "q" is pressed
            print("Saving to frame.png")
            frame_str = './img/img'+str(1)+'.png'
            #frame_str = './img/calibration_imgs/calibration_img'+str(i)+'.png'
            cv2.imwrite(frame_str, frame)
            break

    else:
        print("Failed to capture frame")
        break

cap.release()

#coefs from camera calibration
#Camera intrinsic matrix, K: 
K = np.array([[527.5, 0, 305.7],
    [0, 528.4, 251.6],
    [ 0, 0, 1]])

#Camera extrinsic (transformation) matrix, T:
T = np.array([[0.9737, -0.1124, -0.1981, -0.0554],
 [0.2273, 0.5377, 0.8119, -0.05759],
 [0.01524, -0.8356, 0.5491, 0.268],
 [0, 0, 0, 1]])

#The distortion parameters: 
distortion = np.array([ -0.531, 0.5044, -0.006695, -0.001467, -0.3957])

# Import aruco cube pic
img = cv2.imread(frame, cv2.IMREAD_COLOR_RGB)
plt.imshow(img)

#Undistort the frame using the parameters derived during calibration
undistorted_img = cv2.undistort(img, K, distortion, None, None)
titles = ['Original Image', 'Undistorted Image']
plt.figure(figsize=(16, 6))
plt.subplot(1,2,1)
plt.imshow(img), plt.title(titles[0])
plt.subplot(1,2,2)
plt.imshow(undistorted_img), plt.title(titles[1])

# Detect and estimate pose of the markers in the frame
# N.B.: the dictionary of the specific AruCo marker used, we're using the AprilTag 36h11 marker in this case
gray = cv2.cvtColor(undistorted_img, cv2.COLOR_BGR2GRAY)
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36H11)
parameters = cv2.aruco.DetectorParameters()

# Create the ArUco detector
detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)

# Detect the markers
corners, ids, rejected = detector.detectMarkers(gray)

# Print the detected markers
print("Detected markers:", ids)
# if ids is not None:
#     cv2.aruco.drawDetectedMarkers(undistorted_img, corners, ids)
#     plt.imshow(undistorted_img)
#     plt.show()

# Estimate Pose
marker_length = 0.05 # Meters (actual size of your marker)

if ids is not None:
    print("here")
    rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
        corners, marker_length, K, distortion
    )
    print("here2")
    # Draw axis for each marker
    axis_length = 0.05 # Length of the axes to be drawn
    #for i in range(len(ids)):
        #cv2.drawFrameAxes(undistorted_img, K, distortion, rvecs[i], tvecs[i], axis_length)

    # plt.imshow(undistorted_img)
    # plt.show()

    print(f"Rotation vector: \n{rvecs}")
    print(f"Translation vector: \n{tvecs}")

    """
    Thoughts: for picking in-place get x,y, use 0.5z last joint rotation(ask kene) from the image, 
    inv ik to get angles, path plan to grab. Grasp servo (ask kene).
    For movement one move to be -0.2y, 0x

    """