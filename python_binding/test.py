import picam360
import time
import cv2


cam = picam360.Camera()
for i in range(3):
    if cam.value is not None:
        print("show image!")
        cv2.imshow('image', cam.value)
        cv2.waitKey(0)
    time.sleep(1)

cv2.destroyAllWindows()
cam.stop()