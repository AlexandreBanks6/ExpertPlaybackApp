import cv2
import cv2.aruco as aruco
import yaml
import numpy as np
import glm
from include import utils

#Note: We only track aruco markers for one camera for the scene "calibration" but need to track for both camera for the hand-eye calibration

DEFAULT_CAMCALIB_DIR='../resources/Calib/Calib_Best/'
ARUCO_IDs=[4,5,6,7] #List containing the IDs of the aruco markers that we are tracking
NUM_FRAME_DETECTIONS=8 #How many sets of aruco "frames" need to be detected, ToDo for later

RANSAC_SCENE_REPROJECTION_ERROR=0.0005 #Reprojection error for scene localization RANSAC (in meters)
RANSAC_SCENE_ITERATIONS=100 #Number of iterations for scene localization RANSAC

#Rigid Body Definition of Ring Over Wire Aruco Holder, each four coordinates define an ArUco marker with corresponding ID:
#Marker corners are defined clockwise from top left
#3D model origin is the top left of AruCO ID 6
ARUCO_SIDELENGTH=0.025527 #in meters
ARUCO_SEPERATION=0.1022477 #From closest edges, in meters
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
        self.aruco_dict=aruco.getPredefinedDictionary(aruco.DICT_4X4_1000) #using the 4x4 dictionary to find markers
        self.aruco_params=aruco.DetectorParameters()
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

        #self.corner_for_disp=None

        self.corner_scene_list=[] #List of list containing corners for each detection (each frame)
        self.ids_scene_list=[]

    
    def arucoTrackingScene(self,frame):
        #We only do aruco tracking of the scene for one video stream (only need to know scene w.r.t. one camera)

        #Use Left Camera

        frame_gray=cv2.cvtColor(frame.copy(),cv2.COLOR_BGR2GRAY)

        corners,ids,rejected=aruco.detectMarkers(frame_gray,dictionary=self.aruco_dict,parameters=self.aruco_params)

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
            frame_converted=aruco.drawDetectedMarkers(frame,corners=corners_print,ids=ids_print)
            self.corners_scene=corners_filtered
            self.ids_scene=ids_filtered
            

            #Test Aruco Pose Estimation
            #rvec,tvec,_=aruco.estimatePoseSingleMarkers(corners,ARUCO_SIDELENGTH,self.mtx_left,self.dist_left)
            
            #for c in corners_filtered:
                #print('corner inner: '+str(c))
                #print('Marker Points: '+str(marker_points))
             #   _,rvec,tvec=cv2.solvePnP(marker_points,c,self.mtx_left,self.dist_left,False,cv2.SOLVEPNP_IPPE_SQUARE)
                #cv2.drawFrameAxes(self.app.frame_left_converted,self.mtx_left,self.dist_left,rvec[i],tvec[i],0.05)
              #  cv2.drawFrameAxes(self.app.frame_left_converted,self.mtx_left,self.dist_left,rvec,tvec,0.05)
            #print("Corners:"+str(self.corners_scene))
            #print("IDs:"+str(self.ids_scene))
        else:
            frame_converted=frame
        
        return frame_converted

    def returnObjectPointForHandeye(self,corner_number):
        ids=[elem for entry in self.ids_scene for elem in entry]
        corner_point=False
        print("ids: "+str(ids))
        if corner_number<4 and (6 in ids): #Corner numbering corresponding to ID 6
            corners=RINGOWIRE_MODELPOINTS['6']
            corners=corners.tolist()
            corner_point=corners[corner_number]
        elif corner_number<8 and corner_number>=4 and (4 in ids): #Corner numbering corresponding to ID 4
            corners=RINGOWIRE_MODELPOINTS['4']
            corners=corners.tolist()
            corner_point=corners[corner_number-4]
        elif corner_number<12 and corner_number>=8 and (5 in ids): #Corner numbering corresponding to ID 5
            corners=RINGOWIRE_MODELPOINTS['5']
            corners=corners.tolist()
            corner_point=corners[corner_number-8]
        elif corner_number<16 and corner_number>=12 and (7 in ids): #Corner numbering corresponding to ID 7
            corners=RINGOWIRE_MODELPOINTS['7']
            corners=corners.tolist()
            corner_point=corners[corner_number-12]

        return corner_point





    def calibrateScene(self):
        #Maybe average/some time of filtering to include multiple frames, or we can accumulate multiple frames, create larger array 
        #with corresponding model points to image points, and fit the points to the array that way

        #Note that the rotation_vector and tranlation_vector returned by PnPRansac are the camera w.r.t. the object
        self.corner_scene_list.append(self.corners_scene)
        self.ids_scene_list.append(self.ids_scene)

        if len(self.ids_scene_list)>=NUM_FRAME_DETECTIONS: 
            #self.app.calibrate_on=False
            #print("Corners Scene"+str(self.corner_scene_list))
            
            model_points=None
            image_points=None
            for ids,corners in zip(self.ids_scene_list,self.corner_scene_list): #Looping over corners/ids for each frame
                #corner_found=False
                for id,corner in zip(ids,corners): #Looping over each id/corner in a given frame
                    
                    if image_points is None:
                        image_points=corner[0]
                        model_points=RINGOWIRE_MODELPOINTS[str(id[0])]
                    else:
                        image_points=np.vstack((image_points,corner[0]))
                        model_points=np.vstack((model_points,RINGOWIRE_MODELPOINTS[str(id[0])]))

                    

            print("image points:"+str(image_points))
            print("model points"+str(model_points))
            success,rotation_vector,translation_vector,_=cv2.solvePnPRansac(model_points,image_points,self.mtx,self.dist,\
                                                                          iterationsCount=RANSAC_SCENE_ITERATIONS,reprojectionError=RANSAC_SCENE_REPROJECTION_ERROR,flags=cv2.USAC_MAGSAC)
            #success,rotation_vector,translation_vector=cv2.solvePnP(model_points,image_points,self.mtx_left,self.dist_left,False,cv2.SOLVEPNP_IPPE_SQUARE)
            if success: #We successfully found rotation/translation to the scene with ransac
                #cv2.drawFrameAxes(self.app.frame_left_converted,self.mtx_left,self.dist_left,rotation_vector,translation_vector,0.05)
                #print("Calib Success")
                self.calibrate_done=True
                ci_T_si=utils.convertRvecTvectoHomo(rotation_vector,translation_vector) #Returns as numpy array
                #ci_T_si=utils.EnforceOrthogonalityNumpy_FullTransform(ci_T_si)
                ###So rvec and tvec are object wiriti camera (c_T_o) what we need is (o_T_c) so we invert
                #ci_T_si=utils.invHomogeneousNumpy(ci_T_si)
                self.ci_T_si=glm.mat4(*ci_T_si.T.flatten())
                
                

                ######What we get above is rotation from object to camera (camera w.r.t. scene), if we need opposite the rotation
                ######from camera to object (scene w.r.t. camera) we invert, uncomment this section

                #inverted_mat=Renderer.invHomogenous(self.app.si_T_lci)
                #Rot=np.array([(inverted_mat[0,0],inverted_mat[1,0],inverted_mat[2,0]),
                             #(inverted_mat[0,1],inverted_mat[1,1],inverted_mat[2,1]),
                             #(inverted_mat[0,1],inverted_mat[1,2],inverted_mat[2,2])],dtype='float32')
                #rvec_new,_=cv2.Rodrigues(Rot)                
                #self.app.rvec_scene=rvec_new
                #self.app.tvec_scene=trans_new

                self.rvec_scene=rotation_vector
                self.tvec_scene=translation_vector
                #print("tvec: "+str(self.app.tvec_scene))    
            #Clearing the buffer
            self.corner_scene_list=[]
            self.ids_scene_list=[]
    
    def flattenList(self,xss):
        return [x for xs in xss for x in xs]

        
    