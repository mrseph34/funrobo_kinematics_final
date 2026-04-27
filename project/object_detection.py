import time
import cv2
import numpy as np
import matplotlib.pyplot as plt


class ArucoCameraTracker:
    def __init__(
        self,
        video_id=1,
        save_path="/home/pi/funrobo_kinematics_final/project/img/img1.png",
        marker_length=0.0254
    ):
        """
        video_id: camera index
        save_path: where the captured frame is saved
        marker_length: physical marker size in meters
        """

        self.video_id = video_id
        self.save_path = save_path
        self.marker_length = marker_length

        # Camera calibration parameters
        self.K = np.array([
            [527.5, 0, 305.7],
            [0, 528.4, 251.6],
            [0, 0, 1]
        ])

        self.distortion = np.array([
            -0.531, 0.5044, -0.006695, -0.001467, -0.3957
        ])

        self.T = np.array([
            [0.9737, -0.1124, -0.1981, -0.0554],
            [0.2273, 0.5377, 0.8119, -0.05759],
            [0.01524, -0.8356, 0.5491, 0.268],
            [0, 0, 0, 1]
        ])

        # ArUco detector setup
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(
            cv2.aruco.DICT_APRILTAG_36H11
        )
        self.parameters = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(
            self.aruco_dict, self.parameters
        )

        self.cap = None
        self.frame = None

    def open_camera(self):
        prev_log = cv2.setLogLevel(0)
        for vid_id in range(32):
            c = cv2.VideoCapture(vid_id)
            if c.isOpened():
                ret, frame = c.read()
                if ret and frame is not None:
                    cv2.setLogLevel(prev_log)
                    self.cap = c
                    return
            c.release()
        cv2.setLogLevel(prev_log)
        raise RuntimeError("Camera failed to open")

    def capture_frame_after_delay(self, delay_sec=0.5, verbose=True):
        """
        Waits for delay_sec seconds, captures a frame, and saves it
        """
        if verbose:
            print(f"Waiting {delay_sec} seconds before capturing frame...")
        time.sleep(delay_sec)

        ret, self.frame = self.cap.read()
        if not ret:
            raise RuntimeError("Failed to capture frame")

        cv2.imwrite(self.save_path, self.frame)
        if verbose:
            print(f"Frame saved to {self.save_path}")

        return self.frame

    def undistort_image(self, img):
        return cv2.undistort(img, self.K, self.distortion)

    def detect_and_estimate_pose(self, img, verbose=True):
        """
        Detects ArUco markers and estimates pose
        Returns: ids, rvecs, tvecs
        """
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self.detector.detectMarkers(gray)

        if ids is None:
            if verbose:
                print("No markers detected")
            return None, None, None

        rvecs, tvecs = [], []
        for corner in corners:
            obj_points = np.array([
                [-self.marker_length / 2,  self.marker_length / 2, 0],
                [ self.marker_length / 2,  self.marker_length / 2, 0],
                [ self.marker_length / 2, -self.marker_length / 2, 0],
                [-self.marker_length / 2, -self.marker_length / 2, 0]
            ], dtype=np.float32)
            _, rvec, tvec = cv2.solvePnP(obj_points, corner[0], self.K, self.distortion)
            rvecs.append(rvec)
            tvecs.append(tvec)
        rvecs = np.array(rvecs)
        tvecs = np.array(tvecs)

        return ids, rvecs, tvecs

    def run(self, verbose=True):
        """
        Full pipeline:
        open camera → wait → capture → undistort → detect → estimate pose
        """
        self.open_camera()

        frame = self.capture_frame_after_delay(0.025, verbose=verbose)
        undistorted = self.undistort_image(frame)

        ids, rvecs, tvecs = self.detect_and_estimate_pose(undistorted, verbose=verbose)

        if ids is not None:
            annotated = undistorted.copy()
            cv2.aruco.drawDetectedMarkers(annotated, corners, ids)
            ts = int(time.time() * 1000)
            cv2.imwrite(f"/home/pi/funrobo_kinematics_final/project/img/detect_{ts}.png", annotated)
            if verbose:
                print("Detected markers:", ids.flatten())
                print("Rotation vectors:\n", rvecs)
                print("Translation vectors:\n", tvecs)

        self.cleanup()

        return ids, rvecs, tvecs, undistorted

    def cleanup(self):
        if self.cap is not None:
            self.cap.release()
        cv2.destroyAllWindows()