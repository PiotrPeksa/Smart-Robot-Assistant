import cv2
import numpy as np
import glob

global cameraMatrixL
global cameraMatrixR
global disparity_map
chessboard_size = (9, 6)
frameSize = (640,480)
objp = np.zeros((chessboard_size[0] * chessboard_size[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:chessboard_size[0], 0:chessboard_size[1]].T.reshape(-1, 2)

# Arrays to store object points and image points
objpoints = []  # 3d points in real world space
imgpoints_left = []  # 2d points in image plane for left camera
imgpoints_right = []  # 2d points in image plane for right camera

images_left = glob.glob('C:/Users/peksa/OneDrive/Pictures/Camera Roll/*.jpg') #
images_right = glob.glob('C:/Users/peksa/OneDrive/Pictures/Camera Roll/*.jpg')
left_image = "calibrate0.jpg"
right_image =  "calibrate1.jpg"


def find_image_points(images, imgpoints):
    for fname in images:
        img = cv2.imread(fname)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        ret, corners = cv2.findChessboardCorners(gray, chessboard_size, None)

        if ret:
            objpoints.append(objp)
            imgpoints.append(corners)

# Process both left and right images
find_image_points(images_left, imgpoints_left)
find_image_points(images_right, imgpoints_right)

# Stereo calibration
retL, cameraMatrixL, distL, rvecsL, tvecsL = cv2.calibrateCamera(objpoints, imgpoints_left, frameSize, None, None)
retR, cameraMatrixR, distR, rvecsR, tvecsR = cv2.calibrateCamera(objpoints, imgpoints_right, frameSize, None, None)


window_size = 5  # Adjust this parameter as needed
min_disparity = 0
num_disparities = 128  # Adjust this parameter as needed
sgbm = cv2.StereoSGBM_create(
    minDisparity=min_disparity,
    numDisparities=num_disparities,
    blockSize=window_size)

# Compute the disparity map
disparity_map = sgbm.compute(left_image, right_image)

# Normalize the disparity map for display
#disparity_map_normalized = cv2.normalize(disparity_map, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8U)
