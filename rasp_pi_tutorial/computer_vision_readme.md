# Computer Vision with Raspberry Pi
This tutorial is meant to assist if you want to implement vision into your fleet robotics. It includes a section on AprilTag detection, which is a common use case for vision in robotics. It also provides additional resource if you wish to use more advance techniques such as deep learning.

# AprilTag Detection and Pose Estimation Tutorial
## What are AprilTags?

AprilTags are a visual fiducial system (2D barcodes) used for robotics and computer vision applications. They:
- Can be printed on paper or displayed on screens
- Enable robust 6-DOF (6 degrees of freedom) pose estimation
- Are more reliable than arUco markers in challenging lighting conditions
- Provide both detection confidence and pose information

## Installation

### 1. Install Required Dependencies
For installing OpenCV and the AprilTag library on your Raspberry Pi by running these 1 by 1 in terminal:

```bash
sudo apt-get update
sudo apt-get install -y python3-pip cmake libgl1 libopenblas-dev
pip install opencv-python-headless
pip3 install numpy pupil-apriltags opencv-python-headless flask --break-system-packages
```
For installing OpenCV and the AprilTag library on a Windows device: 

```bash
pip install opencv-python
pip install pupil-apriltags
```

## Basic AprilTag Detection
The code below is a simple example of how to detect AprilTags using OpenCV and the pupil-apriltags library on **your local laptop.**

```python
import cv2
import math
import numpy as np
from pupil_apriltags import Detector

# 1. SETUP: Change these to match your specific setup
TAG_SIZE = 0.16  # The size of your tag in meters (e.g., 0.16 = 16cm)

# Camera Intrinsics (f_x, f_y, c_x, c_y) 
# Use these generic values for Astra Pro Plus (this camera is provided to you)
CAM_PARAMS = [1050.0, 1050.0, 640.0, 360.0] 

# Initialize detector
at_detector = Detector(families='tag36h11') #You can change the family if you are using a different type of AprilTag. to find an april tag that works with this setup, search up "tag36h11 apriltag" online.

cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()
    if not ret: break

    # AprilTag detection requires grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # 2. DETECTION & POSE ESTIMATION
    results = at_detector.detect(gray, 
                                 estimate_tag_pose=True, 
                                 camera_params=CAM_PARAMS, 
                                 tag_size=TAG_SIZE)

    for r in results:
        # Extract translation vector (x, y, z) in meters
        # Coordinate system (camera frame):
        #   X = horizontal (left/right), positive = RIGHT
        #   Y = vertical (up/down), positive = DOWN
        #   Z = depth (forward/backward), positive = AWAY FROM CAMERA (forward)
        x_offset = r.pose_t[0][0]  # horizontal offset (left/right)
        y_offset = r.pose_t[1][0]  # vertical offset (up/down)
        z_distance = r.pose_t[2][0]  # forward distance from camera

        # 3. CALCULATE ANGLE of camera to tag in the horizontal plane.
        # Using atan2(x, z) gives the horizontal angle in radians
        angle_rad = math.atan2(x_offset, z_distance)
        angle_deg = math.degrees(angle_rad)

        # Output the data
        print(f"ID: {r.tag_id} | X: {x_offset:.2f}m | Y: {y_offset:.2f}m | Angle: {angle_deg:.1f}°")

        # Visual feedback (optional)
        (ptA, ptB, ptC, ptD) = r.corners
        cv2.line(frame, tuple(ptA.astype(int)), tuple(ptB.astype(int)), (0, 255, 0), 2)
        cv2.putText(frame, f"ID: {r.tag_id}", (int(ptA[0]), int(ptA[1] - 10)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

    cv2.imshow("AprilTag Setup", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
```
## Addtional Computer Vision Resources
### Deep Learning-Based Object Detection
If you want to implement more advanced computer vision techniques to identify the environment, such as deep learning-based object detection, here are some places to get you started:

- **openCV documentation**: https://docs.opencv.org/4.x/d6/d00/tutorial_py_root.html
- **yolov11n**: excellent object-detection model that can run on edge devices like the Raspberry Pi. check out here for more info: https://github.com/ultralytics/yolov11
- **TensorFlow Lite**: TensorFlow's lightweight solution for running machine learning models on edge devices. You can find pre-trained models for object detection that can be optimized for the Raspberry Pi. Check out the TensorFlow Lite Model Zoo: https://www.tensorflow.org/lite/models
- **Mediapipe**: A cross-platform framework for building multimodal applied machine learning pipelines, including computer vision. It offers pre-built solutions for tasks like pose estimation, object detection, and hand tracking. Check it out here: https://mediapipe.dev/
