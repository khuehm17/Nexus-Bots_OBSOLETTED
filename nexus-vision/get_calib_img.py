"""
This script used for capture image in manual. 
Press 'c' for capture and 'q' for release camera
"""
import cv2

F_WIDTH = 640 * 2
F_HEIGHT = 480
F_FPS = 30

output_dir = "./data/calib_img"
prefix = "img"
left_prefix = 'left_cam_'
right_prefix = 'right_cam_'
image_format = "jpg"
count = 0

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

    # TODO: use os.path.join() method here
    right_img_path = output_dir + '/' + right_prefix + str(count) + '.' + image_format
    left_img_path = output_dir + '/' + left_prefix + str(count) + '.' + image_format

    if cv2.waitKey(1) & 0xFF == ord("c"):
        cv2.imwrite(right_img_path, rigthCam)
        cv2.imwrite(left_img_path, leftCam)
        count += 1      
        print("{} calib image captured".format(count))  

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

stereo.release()
cv2.destroyAllWindows()
