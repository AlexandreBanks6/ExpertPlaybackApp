import cv2
import cv2.aruco as aruco
import yaml
import numpy as np

#Note: We only track aruco markers for one camera for the scene "calibration" but need to track for both camera for the hand-eye calibration

DEFAULT_CAMCALIB_DIR='../resources/Calib/'
ARUCO_IDs=[4,5,6,7] #List containing the IDs of the aruco markers that we are tracking
NUM_FRAME_DETECTIONS=1 #How many sets of aruco "frames" need to be detected, ToDo for later

#Rigid Body of Frame with ArUcos
OBJECT_POINTS=np.array([])

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

        self.corners_scene=[]
        self.ids_scene=[]

        self.corner_scene_list=[] #List of list containing corners for each detection (each frame)
        self.ids_scene_list=[]

    
    def arucoTrackingScene(self):
        #We only do aruco tracking of the scene for one video stream (only need to know scene w.r.t. one camera)

        #Use Left Camera
        #print('left tracked')
        #Converts frame to grayscale
        frame_gray=cv2.cvtColor(self.app.frame_left_converted,cv2.COLOR_RGB2GRAY)

        corners,ids,rejected=aruco.detectMarkers(frame_gray,dictionary=self.aruco_dict,parameters=self.aruco_params)

        #Check if any of the IDs are valid
        if bool(set(ARUCO_IDs)&set(ids)): #Enters if we have a match with our IDs
            for id,corner in zip(ids,corners):
                if id in ARUCO_IDs:
                    frame_gray=aruco.drawDetectedMarkers(frame_gray,corners=corner,ids=id)
                    self.corners_scene.append(corner)
                    self.ids_scene.append(id)
        cv2.imshow("Frame Left",frame_gray)
        cv2.waitKey(1)


    def calibrateScene(self):
        base_frame=None
        self.corner_scene_list.append(self.corners_scene)
        self.ids_scene_list.append(self.ids_scene)

        if len(self.ids_scene_list)>NUM_FRAME_DETECTIONS:           

            cv2.solvePnPRansac(flags=cv2.USAC_MAGSAC)
            #Clearing the buffer
            self.corner_scene_list=[]
            self.ids_scene_list=[]

        
    


