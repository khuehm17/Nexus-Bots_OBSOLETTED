# 19.03.2022
# Process: Script for auto capture image (On going...)

#30.03.2022
# Process: Script for auto capture image after 10sec(Finished)

"""
This script used for auto capture image after 10sec. 
Data will stored in ./nexus-vision/data/calib_img
"""
import cv2
import time
from datetime import datetime

# Camera configuaration
F_WIDTH = 640 * 2
F_HEIGHT = 480
F_FPS = 30

# Photo session settings
total_photos = 30             # Number of images to take
countdown = 5                 # Interval for count-down timer, seconds
font=cv2.FONT_HERSHEY_SIMPLEX # Cowntdown timer font
count_img = 0                 # Count number of images   

output_dir = "./nexus-vision/data/calib_img"
prefix = "img"
left_prefix = 'left_cam_'
right_prefix = 'right_cam_'
image_format = "jpg"   

t2 = datetime.now()

stereo = cv2.VideoCapture(1)
stereo.set(cv2.CAP_PROP_FPS, F_FPS)
stereo.set(cv2.CAP_PROP_FRAME_WIDTH, F_WIDTH)
stereo.set(cv2.CAP_PROP_FRAME_HEIGHT, F_HEIGHT)

while(True):
    stereo.grab()
    _, orginal = stereo.retrieve(flag = 0)

    t1 = datetime.now()
    cntdwn_timer = countdown - int ((t1-t2).total_seconds())

    rigthCam = orginal[0:F_HEIGHT, 0:int(F_WIDTH/2)]
    leftCam = orginal[0:F_HEIGHT, int(F_WIDTH/2):F_WIDTH]

    #cv2.imshow('Left CAM', leftCam)
    #cv2.imshow('Right CAM', rigthCam)
    
    # TODO: use os.path.join() method here
    right_img_path = output_dir + '/' + right_prefix + str(count_img) + '.' + image_format
    left_img_path = output_dir + '/' + left_prefix + str(count_img) + '.' + image_format
    print(cntdwn_timer)
    if cntdwn_timer == -4:
        count_img += 1

        cv2.imwrite(right_img_path, rigthCam)
        cv2.imwrite(left_img_path, leftCam)

        print("{} calib image captured".format(count_img))
        t2 = datetime.now()
        time.sleep(1)
        cntdwn_timer = 0      # To avoid "-1" timer display 
        next
    #Draw countdown counter
    cv2.putText(leftCam, str(cntdwn_timer), (50,50), font, 2.0, (0,0,255),4, cv2.LINE_AA)
    cv2.imshow('Left CAM', leftCam)
    cv2.imshow('Right CAM', rigthCam)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

print("Photo captured finished")
stereo.release()
cv2.destroyAllWindows()
