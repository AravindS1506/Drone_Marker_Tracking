import numpy as np
import cv2
from djitellopy import Tello

# ARUCO_DICT = {
#   "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
#   "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
#   "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
#   "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
#   "DICT_5X5_50": cv2.aruco.DICT_5X5_50,
#   "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
#   "DICT_5X5_250": cv2.aruco.DICT_5X5_250,
#   "DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
#   "DICT_6X6_50": cv2.aruco.DICT_6X6_50,
#   "DICT_6X6_100": cv2.aruco.DICT_6X6_100,
#   "DICT_6X6_250": cv2.aruco.DICT_6X6_250,
#   "DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
#   "DICT_7X7_50": cv2.aruco.DICT_7X7_50,
#   "DICT_7X7_100": cv2.aruco.DICT_7X7_100,
#   "DICT_7X7_250": cv2.aruco.DICT_7X7_250,
#   "DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
#   "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL,
#   "DICT_APRILTAG_16h5": cv2.aruco.DICT_APRILTAG_16h5,
#   "DICT_APRILTAG_25h9": cv2.aruco.DICT_APRILTAG_25h9,
#   "DICT_APRILTAG_36h10": cv2.aruco.DICT_APRILTAG_36h10,
#   "DICT_APRILTAG_36h11": cv2.aruco.DICT_APRILTAG_36h11
# }
prev_rl=0
prev_ud=0
prev_fb=0
def pid_control(distance,prev):
            return int(0.7*distance)+int(0.7*(distance-prev))

def aruco_display(corners, ids, rejected, image):
    
    if len(corners) > 0:
        
        ids = ids.flatten()
        
        for (markerCorner, markerID) in zip(corners, ids):
            
            corners = markerCorner.reshape((4, 2))
            (topLeft, topRight, bottomRight, bottomLeft) = corners
            
            topRight = (int(topRight[0]), int(topRight[1]))
            bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
            bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
            topLeft = (int(topLeft[0]), int(topLeft[1]))

            cv2.line(image, topLeft, topRight, (0, 255, 0), 2)
            cv2.line(image, topRight, bottomRight, (0, 255, 0), 2)
            cv2.line(image, bottomRight, bottomLeft, (0, 255, 0), 2)
            cv2.line(image, bottomLeft, topLeft, (0, 255, 0), 2)
            
            cX = int((topLeft[0] + bottomRight[0]) / 2.0)
            cY = int((topLeft[1] + bottomRight[1]) / 2.0)
            cv2.circle(image, (cX, cY), 4, (0, 0, 255), -1)
            
            cv2.putText(image, str(markerID),(topLeft[0], topLeft[1] - 10), cv2.FONT_HERSHEY_SIMPLEX,
                0.5, (0, 255, 0), 2)
            print("[Inference] ArUco marker ID: {}".format(markerID))
            
    return image


def pose_estimation(frame, aruco_dict_type, matrix_coefficients, distortion_coefficients):

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    cv2.aruco_dict = cv2.aruco.getPredefinedDictionary(aruco_dict_type)
    parameters = cv2.aruco.DetectorParameters()
    detector=cv2.aruco.ArucoDetector(cv2.aruco_dict,arucoParams)
    corners, ids, rejected_img_points=detector.detectMarkers(gray)
    global prev_rl
    global prev_fb
    global prev_ud
        
    if len(corners) > 0:
        for i in range(0, len(ids)):
           
            rvec, tvec, markerPoints = cv2.aruco.estimatePoseSingleMarkers(corners[i], 10, matrix_coefficients,
                                                                       distortion_coefficients)
            
            cv2.aruco.drawDetectedMarkers(frame, corners) 

            cv2.drawFrameAxes(frame, matrix_coefficients, distortion_coefficients, rvec, tvec, 5)
            print(tvec)
            # speedrl=0
            # speedfb=0
            # speedud=0
            # yaw=0
            # speedrl=pid_control(tvec[0][0][0],prev_rl)
            # speedud=-pid_control(tvec[0][0][1],prev_ud)
            # speedfb=pid_control(tvec[0][0][2]-100,prev_fb-100)
            # prev_rl=tvec[0][0][0]
            # prev_ud=tvec[0][0][1]
            # prev_fb=tvec[0][0][2]
            # #yaw=pid_control(rvec[0][0][2]*15)
            # #if(rvec[0][0][2]>0):
            # #    yaw=12
            # #else:
            # #    yaw=-12

            # tello.send_rc_control(speedrl,speedfb,speedud,yaw)

    # else:
    #     #   tello.send_rc_control(0,0,0,0)
    #     #   prev_fb=0
    #     #   prev_rl=0
    #     #   prev_ud=0
            

            

    return frame

    
tello=Tello()
tello.connect()
tello.streamoff()
arucoDict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_250)
arucoParams=cv2.aruco.DetectorParameters()

intrinsic_camera = np.array(((625.26780559,0,347.20776765),(0,831.58598639,294.81516888),(0,0,1)))
distortion = np.array((0.08787649,-1.12494803,-0.02581286,-0.01175951,0.61957025))

tello.streamon()
# tello.takeoff()
while True:
    
    frame_read=tello.get_frame_read()
    myFrame=frame_read.frame
    img=cv2.resize(myFrame,(720,720))
    
    output = pose_estimation(img, cv2.aruco.DICT_5X5_250, intrinsic_camera, distortion)

    cv2.imshow('Estimated Pose', output)

    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        break
# tello.send_rc_control(0,0,0,0)
print(f"Battery Life Pecentage: {tello.get_battery()}")
# tello.land()
tello.streamoff()
cv2.destroyAllWindows()