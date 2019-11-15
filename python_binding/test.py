import picam360
import cv2
from pyquaternion import Quaternion

CV_WAITKEY_CURSORKEY_LEFT   = 81
CV_WAITKEY_CURSORKEY_TOP    = 82
CV_WAITKEY_CURSORKEY_RIGHT  = 83 
CV_WAITKEY_CURSORKEY_BOTTOM = 84
  
M_PI = 3.1415926535

ang_x = 90
ang_y = 0
ang_z = 0
cam = picam360.Camera()
cam.set_fov(90)
while True:
    qx = Quaternion(axis=[1, 0, 0], angle=M_PI*ang_x/180)
    qy = Quaternion(axis=[0, 1, 0], angle=M_PI*ang_y/180)
    qz = Quaternion(axis=[0, 0, 1], angle=M_PI*ang_z/180)
    cam.set_view_quat(qx*qy*qz)
    if cam.value is not None:
        cv2.imshow('image', cam.value)
    key = cv2.waitKey(100)
    if key == ord('q'):
        break
    elif key == CV_WAITKEY_CURSORKEY_TOP:
        ang_x += 1
    elif key == CV_WAITKEY_CURSORKEY_BOTTOM:
        ang_x -= 1
    elif key == CV_WAITKEY_CURSORKEY_RIGHT:
        ang_z += 1
    elif key == CV_WAITKEY_CURSORKEY_LEFT:
        ang_z -= 1
    elif key == -1:
        pass
    else:
        print("key=%d" % key)

cv2.destroyAllWindows()
cam.stop()