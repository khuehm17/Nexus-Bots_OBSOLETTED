# Stereo Vision base

- Hardware: ELP-1MP2CAM001
- Hardware information:
  - https://www.amazon.com/gp/product/B00VG32EC2
- Useful resources:
  - https://github.com/niconielsen32/ComputerVision 
  - https://stereopi.com/blog/projects
  - https://python.plainenglish.io/the-depth-i-stereo-c`alibration-and-rectification-24da7b0fb1e0
  - https://aliyasineser.medium.com/opencv-camera-calibration-e9a48bdd1844
  - https://github.com/aliyasineser/stereoDepth

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

## Calibration

- Take a look: 
  - http://wiki.ros.org/elp_stereo_camera 
  - https://www.youtube.com/watch?v=x6YIwoQBBxA 
    " - prefix: Images should have the same name. This prefix represents that name. (If the list is: image1.jpg, image2.jpg … it shows that the prefix is “image”. Code is generalized but we need a prefix to iterate, otherwise, there can be any other file that we don’t care about.) "
- C++ Code: https://learnopencv.com/camera-calibration-using-opencv/