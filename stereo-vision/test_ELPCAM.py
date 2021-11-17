###TEST
##test 02
import cv2

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

cv2.destroyAllWindows()
