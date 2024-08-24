import cv2
#from cv2 import aruco
import yaml
import numpy as np
import glm

#Note: We only track aruco markers for one camera for the scene "calibration" but need to track for both camera for the hand-eye calibration

DEFAULT_CAMCALIB_DIR='../resources/Calib/Calib_Best/'
ARUCO_IDs=[4,5,6,7] #List containing the IDs of the aruco markers that we are tracking
NUM_FRAME_DETECTIONS=10 #How many sets of aruco "frames" need to be detected, ToDo for later

#Params that work with right:
# RANSAC_SCENE_REPROJECTION_ERROR=0.001 #Reprojection error for scene localization RANSAC (in meters)
# RANSAC_SCENE_ITERATIONS=275 #Number of iterations for scene localization RANSAC

#Params that work with left:
RANSAC_SCENE_REPROJECTION_ERROR=0.005 #Reprojection error for scene localization RANSAC (in meters)
RANSAC_SCENE_ITERATIONS=400 #Number of iterations for scene localization RANSAC

#Rigid Body Definition of Ring Over Wire Aruco Holder, each four coordinates define an ArUco marker with corresponding ID:
#Marker corners are defined clockwise from top left
#3D model origin is the top left of AruCO ID 6
ARUCO_SIDELENGTH=0.0254508  #0.025527
ARUCO_SEPERATION=0.10226 #0.1022477
ARUCO_HEIGHT_OFFSET=0.005

RINGOWIRE_MODELPOINTS={
    "6":np.array([
    [0.0,0.0,0.0],
    [ARUCO_SIDELENGTH,0.0,0.0],
    [ARUCO_SIDELENGTH,-ARUCO_SIDELENGTH,0.0],
    [0.0,-ARUCO_SIDELENGTH,0.0]],dtype='float32'),

    "4":np.array([
    [0.0,ARUCO_SEPERATION+ARUCO_SIDELENGTH,ARUCO_HEIGHT_OFFSET],
    [ARUCO_SIDELENGTH,ARUCO_SEPERATION+ARUCO_SIDELENGTH,ARUCO_HEIGHT_OFFSET],
    [ARUCO_SIDELENGTH,ARUCO_SEPERATION,ARUCO_HEIGHT_OFFSET],
    [0.0,ARUCO_SEPERATION,ARUCO_HEIGHT_OFFSET]],dtype='float32'),

    "5":np.array([
    [ARUCO_SIDELENGTH+ARUCO_SEPERATION,ARUCO_SEPERATION+ARUCO_SIDELENGTH,2*ARUCO_HEIGHT_OFFSET],
    [2*ARUCO_SIDELENGTH+ARUCO_SEPERATION,ARUCO_SEPERATION+ARUCO_SIDELENGTH,2*ARUCO_HEIGHT_OFFSET],
    [2*ARUCO_SIDELENGTH+ARUCO_SEPERATION,ARUCO_SEPERATION,2*ARUCO_HEIGHT_OFFSET],
    [ARUCO_SIDELENGTH+ARUCO_SEPERATION,ARUCO_SEPERATION,2*ARUCO_HEIGHT_OFFSET]],dtype='float32'),

    "7":np.array([
    [ARUCO_SIDELENGTH+ARUCO_SEPERATION,0.0,3*ARUCO_HEIGHT_OFFSET],
    [2*ARUCO_SIDELENGTH+ARUCO_SEPERATION,0.0,3*ARUCO_HEIGHT_OFFSET],
    [2*ARUCO_SIDELENGTH+ARUCO_SEPERATION,-ARUCO_SIDELENGTH,3*ARUCO_HEIGHT_OFFSET],
    [ARUCO_SIDELENGTH+ARUCO_SEPERATION,-ARUCO_SIDELENGTH,3*ARUCO_HEIGHT_OFFSET]],dtype='float32')
}

#Index of numbering-scheme for hand-eye calibration corners w.r.t. above structure
CORNER_NUMBERS=[0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15] #We have 16 corners



#Test Marker Points for SolvePnP
marker_points = np.array([[0.0, 0.0, 0.0],
                            [ARUCO_SIDELENGTH, 0.0, 0.0],
                            [ARUCO_SIDELENGTH,-ARUCO_SIDELENGTH,0.0],
                            [0.0, -ARUCO_SIDELENGTH, 0.0]], dtype=np.float32)


class ArucoTracker:

    def __init__(self,app,left_right):
        #Init Aruco Marker things
        self.aruco_dict=cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_1000) #using the 4x4 dictionary to find markers
        self.aruco_params=cv2.aruco.DetectorParameters()
        self.aruco_detector=cv2.aruco.ArucoDetector(self.aruco_dict,self.aruco_params)
        
        self.aruco_params.adaptiveThreshConstant=10

        self.app=app

        #Reading in camera calibration parameters
        #Right
        if left_right=='right':
            with open(DEFAULT_CAMCALIB_DIR+'calibration_params_right/calibration_matrix.yaml','r') as file:
                cam_info=yaml.load(file)
        elif left_right=='left':
            with open(DEFAULT_CAMCALIB_DIR+'calibration_params_left/calibration_matrix.yaml','r') as file:
                cam_info=yaml.load(file)

        self.mtx=cam_info['camera_matrix'] #Camera Matrix
        self.mtx=np.array(self.mtx,dtype='float32')
        self.dist=cam_info['dist_coeff']    #Distortion Coefficients
        self.dist=np.array(self.dist,dtype='float32')

        self.corners_scene=None
        self.ids_scene=None
        self.calibrate_done=False #Returns true when the calibration is done

        #Transformation matrices to return:
        self.ci_T_si=None #Transformation from scene to camera coordinate system (initial)
        self.rvec_scene=None
        self.tvec_scene=None

        self.corner_scene_list=[] #List of list containing corners for each detection (each frame)
        self.ids_scene_list=[]


    def calibrateSceneDirectNumpy(self,frame,show_pose_tracking,disp_name):
        #Takes in a frame, and finds cam_T_scene (c_T_s) based on one frame
        frame_gray=cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
        corners,ids,_=self.aruco_detector.detectMarkers(frame_gray)
        cam_T_scene=None
        success=False
        rotation_vector=None
        translation_vector=None
        
        if ids is not None:

            corners_filtered=[]
            ids_filtered=[]
            for id,corner in zip(ids,corners):
                if id[0] in ARUCO_IDs:
                    corners_filtered.append(corner)
                    ids_filtered.append([id[0]])
            
            image_points=None
            model_points=None
            if len(ids_filtered)>0: #We found IDs after filtering 
                try:    #If we find corner with ID 6 only use that corner
                    flattened_list=self.flattenList(ids_filtered)
                    index_of_six=flattened_list.index(6)
                    image_points=corners_filtered[index_of_six]
                    model_points=RINGOWIRE_MODELPOINTS[str(ids_filtered[index_of_six][0])]
                except ValueError: #No 6 in the IDs list
                    
                    for id,corner in zip(ids_filtered,corners_filtered):
                        if image_points is None:
                            image_points=corner[0]
                            model_points=RINGOWIRE_MODELPOINTS[str(id[0])]
                        else:
                            image_points=np.vstack((image_points,corner[0]))
                            model_points=np.vstack((model_points,RINGOWIRE_MODELPOINTS[str(id[0])]))
                

                mtx=self.mtx               
                success,rotation_vector,translation_vector,_=cv2.solvePnPRansac(model_points,image_points,mtx,self.dist,\
                                                                                        iterationsCount=RANSAC_SCENE_ITERATIONS,reprojectionError=RANSAC_SCENE_REPROJECTION_ERROR,flags=cv2.USAC_MAGSAC) #SOLVEPNP_AP3P
                if success:
                    cam_T_scene=self.convertRvecTvectoHomo(rotation_vector,translation_vector)

                    if show_pose_tracking:
                        ids_print=np.array(ids_filtered)
                        corners_print=np.array(corners_filtered,dtype='float32')
                        frame_converted=cv2.aruco.drawDetectedMarkers(frame,corners=corners_print,ids=ids_print) 
                        cv2.drawFrameAxes(frame_converted,self.mtx,self.dist,rotation_vector,translation_vector,0.05)
                        cv2.imshow(disp_name,frame_converted)
                        cv2.waitKey(1)   

                    

        return cam_T_scene


    def flattenList(self,xss):
        return [x for xs in xss for x in xs]
    

    def convertRvecTvectoHomo(self,rvec,tvec):
        tvec=tvec.flatten()
        #print("tvec new: "+str(tvec))
        #Input: OpenCV rvec (rotation) and tvec (translation)
        #Output: Homogeneous Transform
        Rot,_=cv2.Rodrigues(rvec)
        #print("Rotation Matrix: "+str(Rot))
        transform=np.identity(4)
        transform[0:3,0:3]=Rot
        transform[0:3,3]=tvec
        transform=self.EnforceOrthogonalityNumpy_FullTransform(transform)
        return transform

    def EnforceOrthogonalityNumpy_FullTransform(self,transform):
        transform[0:3,0:3]=self.EnforceOrthogonalityNumpy(transform[0:3,0:3])

        return transform

    def EnforceOrthogonalityNumpy(self,R):
        #Function which enforces a rotation matrix to be orthogonal
        #Input: R is a 4x numpy rotation

        #Extracting columns of rotation matrix
        x=R[:,0] 
        y=R[:,1]
        z=R[:,2]
        diff_err=np.dot(x,y)
        x_orth=x-(0.5*diff_err*y)
        y_orth=y-(0.5*diff_err*x)
        z_orth=np.cross(x_orth,y_orth)
        x_norm=0.5*(3-np.dot(x_orth,x_orth))*x_orth
        y_norm=0.5*(3-np.dot(y_orth,y_orth))*y_orth
        z_norm=0.5*(3-np.dot(z_orth,z_orth))*z_orth
        R_new=np.column_stack((x_norm,y_norm,z_norm))

        return R_new
    


        
    