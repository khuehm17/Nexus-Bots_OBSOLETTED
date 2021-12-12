import cv2

CAM_IDX = 1
F_WIDTH = 640 * 2
F_HEIGHT = 480
# F_WIDTH = 640
# F_HEIGHT = 480
F_FPS = 30

stereo_left_dir = './data/stereo_left.avi'
stereo_right_dir = './data/stereo_right.avi'

# Define the codec and create VideoWriter object
out_left = cv2.VideoWriter(stereo_left_dir, cv2.VideoWriter_fourcc('M','J','P','G'), F_FPS, (int(F_WIDTH/2), int(F_HEIGHT)))
out_right = cv2.VideoWriter(stereo_right_dir, cv2.VideoWriter_fourcc('M','J','P','G'), F_FPS, (int(F_WIDTH/2), int(F_HEIGHT)))

stereo = cv2.VideoCapture(CAM_IDX)
stereo.set(cv2.CAP_PROP_FPS, F_FPS)
stereo.set(cv2.CAP_PROP_FRAME_WIDTH, F_WIDTH)
stereo.set(cv2.CAP_PROP_FRAME_HEIGHT, F_HEIGHT)

print("record started...")

while(True):
    stereo.grab()
    _, orginal = stereo.retrieve(flag = 0)

    leftFrame = orginal[0:F_HEIGHT, int(F_WIDTH/2):F_WIDTH]
    rightFrame = orginal[0:F_HEIGHT, 0:int(F_WIDTH/2)]

    # write the frame
    out_left.write(leftFrame)
    out_right.write(rightFrame)

    cv2.imshow('Left CAM', leftFrame)
    cv2.imshow('Right CAM', rightFrame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release everything if job is finished
stereo.release()
out_left.release()
out_right.release()
cv2.destroyAllWindows()
