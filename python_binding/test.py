import picam360
import cv2
from pyquaternion import Quaternion

CV_WAITKEY_CURSORKEY_LEFT   = 81
CV_WAITKEY_CURSORKEY_TOP    = 82
CV_WAITKEY_CURSORKEY_RIGHT  = 83 
CV_WAITKEY_CURSORKEY_BOTTOM = 84
CV_WAITKEY_CURSORKEY_PLUS   = 171
CV_WAITKEY_CURSORKEY_MINUS  = 141
  
M_PI = 3.1415926535

step = 3
fov = 90
ang_x = 90
ang_y = 0
ang_z = 0
cam = picam360.Camera()
while True:

    qx = Quaternion(axis=[1, 0, 0], angle=M_PI*ang_x/180)
    qy = Quaternion(axis=[0, 1, 0], angle=M_PI*ang_y/180)
    qz = Quaternion(axis=[0, 0, 1], angle=M_PI*ang_z/180)
    quat = qy*qx*qz
    cmd = "view_quat=%f,%f,%f,%f fov=%f" % (quat.x, quat.y, quat.z, quat.w, fov)
    cam.set_vostream_param(cmd)
    
    if cam.value is not None:
        cv2.imshow('image', cam.value)
        
    key = cv2.waitKey(100)
    if key == ord('q'):
        break
    elif key == CV_WAITKEY_CURSORKEY_TOP:
        ang_x += step
    elif key == CV_WAITKEY_CURSORKEY_BOTTOM:
        ang_x -= step
    elif key == CV_WAITKEY_CURSORKEY_LEFT:
        ang_y += step
    elif key == CV_WAITKEY_CURSORKEY_RIGHT:
        ang_y -= step
    elif key == CV_WAITKEY_CURSORKEY_MINUS:
        fov += step
    elif key == CV_WAITKEY_CURSORKEY_PLUS:
        fov -= step
    elif key == -1:
        pass
    else:
        print("key=%d" % key)

cv2.destroyAllWindows()
cam.stop()