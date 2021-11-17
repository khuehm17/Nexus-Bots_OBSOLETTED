#designed by vtmkhoi

import cv2
import numpy as np

import glob
import os
import random
import sys


#init chess board
CHESSBOARD_SIZE = (7, 6)
CHESSBOARD_OPTIONS = (cv2.CALIB_CB_ADAPTIVE_THRESH |
        cv2.CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_FAST_CHECK)

OBJECT_POINT_ZERO = np.zeros((CHESSBOARD_SIZE[0] * CHESSBOARD_SIZE[1], 3),
        np.float32)
OBJECT_POINT_ZERO[:, :2] = np.mgrid[0:CHESSBOARD_SIZE[0],
        0:CHESSBOARD_SIZE[1]].T.reshape(-1, 2)

OPTIMIZE_ALPHA = 0.25

TERMINATION_CRITERIA = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_MAX_ITER, 30,
        0.001)

MAX_IMAGES = 64

if len(sys.argv) != 4:
    print("Syntax: {0} LEFT_IMAGE_DIR RIGHT_IMAGE_DIR OUTPUT_FILENAME"
            .format(sys.argv[0]))
    sys.exit(1)
    
F_WIDTH = 1280
F_HEIGHT = 480
F_FPS = 30

stereo = cv2.VideoCapture(1)

stereo.set(cv2.CAP_PROP_FPS, F_FPS)
stereo.set(cv2.CAP_PROP_FRAME_WIDTH, F_WIDTH)
stereo.set(cv2.CAP_PROP_FRAME_HEIGHT, F_HEIGHT)

while(True):
    stereo.grab()
    _, orginal = stereo.retrieve(flag = 0)

    rigthCam = orginal[0:F_HEIGHT, 0:int(F_WIDTH/2)]
    leftCam = orginal[0:F_HEIGHT, int(F_WIDTH/2):F_WIDTH]

    cv2.imshow('Left CAM', leftCam)
    cv2.imshow('Right CAM', rigthCam)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

stereo.release()
cv2.destroyAllWindows()
