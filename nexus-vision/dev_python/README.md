# Stereo Vision base

- Hardware: ELP-1MP2CAM001
- Hardware information:
  - https://www.amazon.com/gp/product/B00VG32EC2
- Useful resources:
  - https://github.com/niconielsen32/ComputerVision 
  - https://stereopi.com/blog/projects
  - https://python.plainenglish.io/the-depth-i-stereo-calibration-and-rectification-24da7b0fb1e0
  - https://aliyasineser.medium.com/opencv-camera-calibration-e9a48bdd1844
  - https://github.com/aliyasineser/stereoDepth
  - http://wiki.ros.org/elp_stereo_camera 
  - https://www.youtube.com/watch?v=x6YIwoQBBxA 
  - https://www.youtube.com/watch?v=GpU1Vx-b3VA
  - https://courses.cs.washington.edu/courses/cse455/09wi/Lects/      
      (List of ComputerVision lecture)
## Test ELP stereo camera
- Connect the camera to the PC
- Run test_ELPCAM.py to check if the camera worked or not

## Stereo camera calibration procedure
- Step 1: Get images for calibration
  - Run get_calib_img.py to get the camera calibration data
- Step 2: Single camera calibration
  - Calibrate left camera:
    ```
    $ python3 single_calib.py --prefix left_cam_ --save_file data/left.yml
    ```
  - Calibrate right camera:
    ```
    $ python3 single_calib.py --prefix right_cam_ --save_file data/right.yml
    ```
- Step 3: Stereo camera calibration
  ```
  $ python3 stereo_calib.py
  ```
- Step 4: Show disparity map (Using recorded video)
  ```
  $ python3 disparity_map.py
  ```
## ERROR: module 'cv2.cv2' has no attribute 'ximgproc'
- Fix:
  ```
  $ pip install --force-reinstall opencv-contrib-python==4.1.2.30
  ```