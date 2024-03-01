import cv2
import cv2.aruco as aruco
import yaml
import numpy as np
import glm
from include import Renderer

#Note: We only track aruco markers for one camera for the scene "calibration" but need to track for both camera for the hand-eye calibration

DEFAULT_CAMCALIB_DIR='../resources/Calib/'
ARUCO_IDs=[4,5,6,7] #List containing the IDs of the aruco markers that we are tracking
NUM_FRAME_DETECTIONS=1 #How many sets of aruco "frames" need to be detected, ToDo for later

RANSAC_SCENE_REPROJECTION_ERROR=0.005 #Reprojection error for scene localization RANSAC (in meters)
RANSAC_SCENE_ITERATIONS=40 #Number of iterations for scene localization RANSAC

#Rigid Body Definition of Ring Over Wire Aruco Holder, each four coordinates define an ArUco marker with corresponding ID:
#Marker corners are defined clockwise from top left
#3D model origin is the top left of AruCO ID 6
ARUCO_SIDELENGTH=0.025527 #in meters
ARUCO_SEPERATION=0.1022858 #From closest edges, in meters
RINGOWIRE_MODELPOINTS={
    "6":np.array([
    (0.0,0.0,0.0),
    (ARUCO_SIDELENGTH,0.0,0.0),
    (ARUCO_SIDELENGTH,-ARUCO_SIDELENGTH,0.0),
    (0.0,-ARUCO_SIDELENGTH,0.0)],dtype='float32'),

    "4":np.array([
    (0.0,ARUCO_SEPERATION+ARUCO_SIDELENGTH,0.0),
    (ARUCO_SIDELENGTH,ARUCO_SEPERATION+ARUCO_SIDELENGTH,0.0),
    (ARUCO_SIDELENGTH,ARUCO_SEPERATION,0.0),
    (0.0,ARUCO_SEPERATION,0.0)],dtype='float32'),

    "5":np.array([
    (ARUCO_SIDELENGTH+ARUCO_SEPERATION,ARUCO_SEPERATION+ARUCO_SIDELENGTH,0.0),
    (2*ARUCO_SIDELENGTH+ARUCO_SEPERATION,ARUCO_SEPERATION+ARUCO_SIDELENGTH,0.0),
    (2*ARUCO_SIDELENGTH+ARUCO_SEPERATION,ARUCO_SEPERATION,0.0),
    (ARUCO_SIDELENGTH+ARUCO_SEPERATION,ARUCO_SEPERATION,0.0)],dtype='float32'),

    "7":np.array([
    (ARUCO_SIDELENGTH+ARUCO_SEPERATION,0.0,0.0),
    (2*ARUCO_SIDELENGTH+ARUCO_SEPERATION,0.0,0.0),
    (2*ARUCO_SIDELENGTH+ARUCO_SEPERATION,-ARUCO_SEPERATION,0.0),
    (ARUCO_SIDELENGTH+ARUCO_SEPERATION,-ARUCO_SEPERATION,0.0)],dtype='float32')
}


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
        self.mtx_right=np.array(self.mtx_right,dtype='float32')
        self.dist_right=right_cam_info['dist_coeff']    #Distortion Coefficients
        self.dist_right=np.array(self.dist_right,dtype='float32')

        #Left

        with open(DEFAULT_CAMCALIB_DIR+'calibration_params_left/calibration_matrix.yaml','r') as file:
            left_cam_info=yaml.load(file)

        self.mtx_left=left_cam_info['camera_matrix'] #Camera Matrix
        self.mtx_left=np.array(self.mtx_left,dtype='float32')

        self.dist_left=left_cam_info['dist_coeff']    #Distortion Coefficients
        self.dist_left=np.array(self.dist_left,dtype='float32')

        self.corners_scene=None
        self.ids_scene=None

        self.corner_scene_list=[] #List of list containing corners for each detection (each frame)
        self.ids_scene_list=[]

    
    def arucoTrackingScene(self):
        #We only do aruco tracking of the scene for one video stream (only need to know scene w.r.t. one camera)

        #Use Left Camera

        frame=self.app.frame_left
        frame_gray=cv2.cvtColor(frame.copy(),cv2.COLOR_BGR2GRAY)

        corners,ids,rejected=aruco.detectMarkers(frame_gray,dictionary=self.aruco_dict,parameters=self.aruco_params)
        #print("ids original:"+str(ids))
        #print("corners original:"+str(corners))

        corners_filtered=[]
        ids_filtered=[]
        if ids is not None:
            for id,corner in zip(ids,corners):
                if id[0] in ARUCO_IDs:
                    corners_filtered.append(corner)
                    ids_filtered.append([id[0]])

        ids_print=np.array(ids_filtered)
        corners_print=np.array(corners_filtered,dtype='float32')

        if len(ids_filtered)>0:
            self.app.frame_left_converted=aruco.drawDetectedMarkers(frame,corners=corners_print,ids=ids_print)
            self.corners_scene=corners_filtered
            self.ids_scene=ids_filtered
            #print("Corners:"+str(self.corners_scene))
            #print("IDs:"+str(self.ids_scene))
        else:
            self.app.frame_left_converted=frame



    def calibrateScene(self):
        #Maybe average/some time of filtering to include multiple frames, or we can accumulate multiple frames, create larger array 
        #with corresponding model points to image points, and fit the points to the array that way

        #Note that the rotation_vector and tranlation_vector returned by PnPRansac are the object w.r.t camera
        #But we want the camera w.r.t. object so we invert
        self.corner_scene_list.append(self.corners_scene)
        self.ids_scene_list.append(self.ids_scene)

        if len(self.ids_scene_list)>=NUM_FRAME_DETECTIONS: 
            model_points=None
            image_points=None
            for ids,corners in zip(self.ids_scene_list,self.corner_scene_list): #Looping over corners/ids for each frame
                for id,corner, in zip(ids,corners): #Looping over each id/corner in a given frame
                    #print("id: "+str(id))
                    if image_points is None:
                        image_points=corner[0]
                        model_points=RINGOWIRE_MODELPOINTS[str(id[0])]
                    else:
                        image_points=np.vstack((image_points,corner[0]))
                        model_points=np.vstack((model_points,RINGOWIRE_MODELPOINTS[str(id[0])]))

            print("image points:"+str(image_points))
            print("model points"+str(model_points))
            (success,rotation_vector,translation_vector,_)=cv2.solvePnPRansac(objectPoints=model_points,imagePoints=image_points,\
                                                                    cameraMatrix=self.mtx_left,distCoeffs=self.dist_left,\
                                                                        iterationsCount=RANSAC_SCENE_ITERATIONS,reprojectionError=RANSAC_SCENE_REPROJECTION_ERROR,flags=cv2.USAC_MAGSAC)

            if success: #We successfully found rotation/translation to the scene with ransac
                self.app.lci_T_si=self.convertRvecTvectoHomo(rotation_vector,translation_vector)
                print("Base Frame: "+str(self.app.lci_T_si))
                self.app.si_T_lci=Renderer.invHomogenous(self.app.lci_T_si)       
                print("Inverted Base Frame: "+str(self.app.si_T_lci))     
                self.app.rvec_scene=rotation_vector
                self.app.tvec_scene=translation_vector    
            #Clearing the buffer
            self.corner_scene_list=[]
            self.ids_scene_list=[]
    
    def flattenList(self,xss):
        return [x for xs in xss for x in xs]
    
    def convertRvecTvectoHomo(self,rvec,tvec):
        Rot,_=cv2.Rodrigues(rvec)
        transform=glm.mat4(glm.vec4(Rot[0,0],Rot[1,0],Rot[2,0],0),
                           glm.vec4(Rot[0,1],Rot[1,1],Rot[2,1],0),
                           glm.vec4(Rot[0,2],Rot[1,2],Rot[2,2],0),
                           glm.vec4(tvec[0],tvec[1],tvec[2],1))
        return transform

        
    


