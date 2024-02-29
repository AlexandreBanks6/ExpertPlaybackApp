import cv2
import cv2.aruco as aruco
import yaml
import numpy as np


DEFAULT_CAMCALIB_DIR='../resources/Calib/'


class ArucoTracker:

    def __init__(self,app):
        #Init Aruco Marker things
        self.aruco_dict=aruco.getPredefinedDictionary(aruco.DICT_4X4_1000) #using the 4x4 dictionary to find markers
        self.aruco_params=aruco.DetectorParameters()
        self.aruco_params.adaptiveThreshConstant=10

        self.app=app

        #Reading in camera calibration parameters
        #Right
        with open(DEFAULT_CAMCALIB_DIR+'calibration_params_right/calibration_matrix.yaml','r') as file:
            right_cam_info=yaml.load(file)

        self.mtx_right=right_cam_info['camera_matrix'] #Camera Matrix
        self.dist_right=right_cam_info['dist_coeff']    #Distortion Coefficients

        #Left

        with open(DEFAULT_CAMCALIB_DIR+'calibration_params_left/calibration_matrix.yaml','r') as file:
            left_cam_info=yaml.load(file)

        self.mtx_left=left_cam_info['camera_matrix'] #Camera Matrix
        self.dist_left=left_cam_info['dist_coeff']    #Distortion Coefficients

        self.font=cv2.FONT_HERSHEY_SIMPLEX


    
    def arucoTracking(self,left_right):
        if left_right=='left':
            print('left tracked')
            #Converts frame to grayscale
            frame_gray=cv2.cvtColor(self.app.frame_left_converted,cv2.COLOR_RGB2GRAY)

            corners,ids,rejected=aruco.detectMarkers(frame_gray,dictionary=self.aruco_dict,parameters=self.aruco_params)

            if np.all(ids != None) > 0:

                frame_gray=aruco.drawDetectedMarkers(frame_gray,corners,ids)
            cv2.imshow("Frame Left",frame_gray)
            cv2.waitKey(1)

        if left_right=='right':
            frame_gray=cv2.cvtColor(self.app.frame_right_converted,cv2.COLOR_RGB2GRAY)
            corners,ids,rejected=aruco.detectMarkers(frame_gray,dictionary=self.aruco_dict,parameters=self.aruco_params)

            if np.all(ids != None):
                frame_gray=aruco.drawDetectedMarkers(frame_gray,corners,ids)
            
            cv2.imshow("Frame Right",frame_gray)
            cv2.waitKey(1)


