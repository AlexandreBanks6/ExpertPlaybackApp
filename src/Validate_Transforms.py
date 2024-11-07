'''
Description: Script to validate the camera-to-scene transform and the PSM1/PSM3 tool pose estimation


'''
import rospy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float32MultiArray
import cv2
from cv_bridge import CvBridge
import time
import moderngl as mgl

from include import Model as mdl
from include import Camera
from include import Light
from include import mesh
from include import scene
from include import ArucoTracker
from include import utils
import glm
import pyglet


#NDI Tracker Stuff
from sksurgerynditracker.nditracker import NDITracker
from datetime import datetime
import csv

from pyglet.gl import Config, Context
from pyglet.window import key

#TKinter importer
import tkinter as tk

#dvrk tool stuff
import dvrk

import tf_conversions.posemath as pm
import tf_conversions
import yaml
import pandas as pd


import numpy as np
import os

#################File Names#####################
PATH_TO_NDI_APPLE3='../resources/NDI_DRF_Models/APPLE03.rom' 
#PATH_TO_NDI_APPLE3='../resources/NDI_DRF_Models/APPLE09.rom' 
PATH_TO_VALIDATION_CSV='../resources/validation/camToScene/'
PATH_TO_POSE_VALIDATION_CSV='../resources/validation/PSMPose/'



##################ROS Topics####################
#Image Topic Subscriptions
RightFrame_Topic='ubc_dVRK_ECM/right/decklink/camera/image_raw/compressed'
LeftFrame_Topic='ubc_dVRK_ECM/left/decklink/camera/image_raw/compressed'
lc_T_s_Topic='ExpertPlayback/lc_T_s' #Published from PC2 (subscribed to by PC1)
rc_T_s_Topic='ExpertPlayback/rc_T_s' #Published from PC2 (subscribed to by PC1)



###############Header for Cam-to-Scene Validation CSV#############
repeat_string=["Tx","Ty","Tz","R00","R01","R02","R10","R11","R12","R20","R21","R22"] #First number is row of R, second number is column
#Camera to scene transform validation
CAM_TO_SCENE_VALIDATION_HEADER=["Timestamp","Trial #",'Compute Time',"NDI_Tracker",\
                                            "Tx","Ty","Tz","R00","R01","R02","R10","R11","R12","R20","R21","R22","Tracking Quality",\
                                            "visual_frame","Tx","Ty","Tz","R00","R01","R02","R10","R11","R12","R20","R21","R22"]

#Scene to PSM validation

S_T_PSM_VALIDATION_HEADER=["Timestamp","Trial #",'Compute Time',"NDI_Tracker",\
                                            "Tx","Ty","Tz","R00","R01","R02","R10","R11","R12","R20","R21","R22","Tracking Quality",\
                                            "s_T_psm_estimate","Tx","Ty","Tz","R00","R01","R02","R10","R11","R12","R20","R21","R22"]
#Camera to PSM validation
LC_T_PSM_VALIDATION_HEADER=["Timestamp","Trial #",'Compute Time',"lc_T_aruco",\
                                            "Tx","Ty","Tz","R00","R01","R02","R10","R11","R12","R20","R21","R22",\
                                            "lc_T_psm_estimate","Tx","Ty","Tz","R00","R01","R02","R10","R11","R12","R20","R21","R22"]




ECM_FRAME_WIDTH_DESIRED=1024
ECM_FRAME_HEIGHT_DESIRED=722



#Console Viewport Size
CONSOLE_VIEWPORT_WIDTH=1024
CONSOLE_VIEWPORT_HEIGHT=722


#tool_tip_offset=0.0102  #PSM1 tooltip offset (large needle driver)
tool_tip_offset=0.021082 #For Fenestrated Bipolar Forceps
#tool_tip_offset_psm3=0.0102 #PSM3 tooltip offset
tool_tip_offset_psm3=0.021082


tool_tip_point=np.array([0,0,tool_tip_offset+0.001,1],dtype=np.float32) #Point of tooltip for API correction
METERS_TO_RENDER_SCALE=1000 #OpenGL in mm, real word in m

#########Constant Transforms
mul_mat=glm.mat4x3()  #Selection matrix to take [Xc,Yc,Zc]
input_pose=glm.mat4()

test_point_psm=glm.vec4(0.0,0.0,0.0,1.0) #Used to project last joint of psm1

#Converts opencv camera to opengl
opengl_T_opencv=glm.mat4(glm.vec4(1,0,0,0),
                     glm.vec4(0,-1,0,0),
                     glm.vec4(0,0,-1,0),
                     glm.vec4(0,0,0,1))

class Renderer:
    def __init__(self,win_size=(CONSOLE_VIEWPORT_WIDTH,CONSOLE_VIEWPORT_HEIGHT)):

        self.WIN_SIZE=win_size

        #Creates the pyglet window
        config=Config(major_version=3,minor_version=3,depth_size=3,double_buffer=True)
        self.window_left = pyglet.window.Window(width=win_size[0], height=win_size[1], config=config,caption='Left Eye')
        self.window_right = pyglet.window.Window(width=win_size[0], height=win_size[1], config=config,caption='Right Eye')

        vertex_source,fragment_source=self.get_program_background('background')

        #ArUco tracking objects
        self.aruco_tracker_left=ArucoTracker.ArucoTracker(self,'left')
        cam_mat=self.aruco_tracker_left.mtx 
        self.cam_mat_left=glm.mat3(*cam_mat.T.flatten())
        
        self.aruco_tracker_right=ArucoTracker.ArucoTracker(self,'right')
        cam_mat=self.aruco_tracker_right.mtx 
        self.cam_mat_right=glm.mat3(*cam_mat.T.flatten())



        ############Left Window Initialization
        self.window_left.switch_to()
        self.window_left.on_mouse_motion=self.on_mouse_motion_left
        self.window_left.on_key_press=self.on_key_press
        self.window_left.on_key_release=self.on_key_release
        self.ctx_left=mgl.create_context(require=330,standalone=False)
        self.ctx_left.enable(flags=mgl.DEPTH_TEST|mgl.CULL_FACE) #CULL_FACE does not render invisible faces
        self.ctx_left.enable(mgl.BLEND)
        self.camera_left=Camera.Camera(self,self.aruco_tracker_left.mtx,CONSOLE_VIEWPORT_WIDTH,CONSOLE_VIEWPORT_HEIGHT)

        #Background Image
        self.background_program_left=self.ctx_left.program(vertex_shader=vertex_source,fragment_shader=fragment_source)
        self.background_program_left['texture0']=0
        self.vertex_array_left=self.init_vertex_array(self.ctx_left,self.background_program_left)

        #self.texture_left=self.ctx_left.texture(self.test_image.size,3,data=self.test_image.tobytes())
        self.texture_left=self.ctx_left.texture(size=(CONSOLE_VIEWPORT_WIDTH,CONSOLE_VIEWPORT_HEIGHT),components=3)
        self.texture_left.use()


        ############Right Window Initialization
        self.window_right.switch_to()
        self.window_right.on_mouse_motion=self.on_mouse_motion_right
        self.window_right.on_key_press=self.on_key_press
        self.window_right.on_key_release=self.on_key_release
        self.ctx_right=mgl.create_context(require=330,standalone=False)
        self.ctx_right.enable(flags=mgl.DEPTH_TEST|mgl.CULL_FACE)
        self.ctx_right.enable(mgl.BLEND)
        self.camera_right=Camera.Camera(self,self.aruco_tracker_right.mtx,CONSOLE_VIEWPORT_WIDTH,CONSOLE_VIEWPORT_HEIGHT)

        #Initializing right background
        self.window_right.switch_to()
        self.background_program_right=self.ctx_right.program(vertex_shader=vertex_source,fragment_shader=fragment_source)
        self.background_program_right['texture0']=0
        self.vertex_array_right=self.init_vertex_array(self.ctx_right,self.background_program_right)

        self.texture_right=self.ctx_right.texture(size=(CONSOLE_VIEWPORT_WIDTH,CONSOLE_VIEWPORT_HEIGHT),components=3)
        self.texture_right.use()


        self.key_dict={
            'W': False,
            'S': False,
            'A': False,
            'D': False,
            'Q': False,
            'E': False
        }


        #Lighting instance
        self.light=Light.Light()
        print("Done Light Constructor")


        #Mesh instance:
        self.mesh=mesh.Mesh(self)
        print("Done Mesh Constructor")


        #Scene
        #self.scene=scene.Scene(self)
        self.scene_PSM1=scene.Scene(self) #Scene for PSM1
        self.scene_PSM3=scene.Scene(self) #Scene for PSM2
        print("Done Scene Constructor")

        ########Variables for frames passed by endoscope
        self.frame_right=None #Updated by the callback in rospy
        self.frame_left=None
        self.frame_left_converted=None
        self.frame_right_converted=None

        self.bridge=CvBridge()



        ##############GUI SETUP################
        self.gui_window=tk.Tk()
        self.gui_window.title("dVRK Playback App Validation")

        self.gui_window.rowconfigure([0,1,2,3,4,5,6],weight=1)
        self.gui_window.columnconfigure([0,1,2,3,5],weight=1)
        self.gui_window.minsize(300,100)

        #Title at top
        self.welcome_text=tk.Label(self.gui_window,text="Welcome to the dVRK Playback App Transform Validation")
        self.welcome_text.grid(row=0,column=1,sticky='n')     


        ###Buttons to calibrate PSM API error
        self.start_psmerror_calibration=tk.Button(self.gui_window,text="Start PSM Error Calib",command=self.startPSMErrorCalib)
        self.start_psmerror_calibration.grid(row=2,column=0,sticky="nsew")

        self.grab_psm1_pointbutton=tk.Button(self.gui_window,text="Grab PSM1 Point",command=self.grabPSM1PointCallback)
        self.grab_psm1_pointbutton.grid(row=2,column=1,sticky="nsew")

        self.grab_psm3_pointbutton=tk.Button(self.gui_window,text="Grab PSM3 Point",command=self.grabPSM3PointCallback)
        self.grab_psm3_pointbutton.grid(row=2,column=2,sticky="nsew")

        self.calibrate_psmerror_button=tk.Button(self.gui_window,text="Calibrate PSM Error",command=self.calibratePSMErrorCallback)
        self.calibrate_psmerror_button.grid(row=2,column=3,sticky="nsew")


        ###Buttons to Validate Camera-To-Scene
        self.start_camToSceneValid_Button=tk.Button(self.gui_window,text="Start Cam-To-Scene Validation",command=self.startCamToSceneCallback)
        self.start_camToSceneValid_Button.grid(row=3,column=2,sticky="nsew")

        #Select Either Single Cam-to-Scene Estimation, or Multiple
        self.checkbox_camToSceneSingle=tk.Checkbutton(self.gui_window,text="Single Cam-to-Scene",onvalue=1,offvalue=0,command=self.camToSceneSingleCallback)
        self.checkbox_camToSceneSingle.grid(row=3,column=0,sticky='nsew')

        self.checkbox_camToSceneMultiple=tk.Checkbutton(self.gui_window,text="Multiple Cam-to-Scene",onvalue=1,offvalue=0,command=self.camToSceneMultipleCallback)
        self.checkbox_camToSceneMultiple.grid(row=3,column=1,sticky='nsew')
        
        self.calibrate_Scene_Button=tk.Button(self.gui_window,text="Get Cam-To-Scene",command=self.calibrateSceneCallback)
        self.calibrate_Scene_Button.grid(row=3,column=3,sticky="nsew")

        ###Buttons to Validate Pose Tracking
        

        #Select Either Single Cam-to-Scene Estimation, or Multiple
        self.checkbox_PSM1=tk.Checkbutton(self.gui_window,text="PSM1",onvalue=1,offvalue=0,command=self.psm1Checkbox)
        self.checkbox_PSM1.grid(row=4,column=0,sticky='nsew')

        self.checkbox_PSM3=tk.Checkbutton(self.gui_window,text="PSM3",onvalue=1,offvalue=0,command=self.psm3Checkbox)
        self.checkbox_PSM3.grid(row=4,column=1,sticky='nsew')


        self.start_poseValid_Button=tk.Button(self.gui_window,text="Start PSM Pose Validation",command=self.startPSMPoseCallback)
        self.start_poseValid_Button.grid(row=4,column=2,sticky="nsew")
        
        self.grabPoseButton=tk.Button(self.gui_window,text="Grab PSM Pose",command=self.grabPSMPoseCallback)
        self.grabPoseButton.grid(row=4,column=3,sticky="nsew")




        ##################Booleans and Vars################

        #Variables for PSM error correction
        self.psm1_points_count=0
        self.psm3_points_count=0
        self.is_psmerror_started=False
        self.is_psmerror_calib=False

        self.project_point_psm1_left=None #Point to touch with psm
        self.project_point_psm1_right=None #Point to touch with psm

        self.project_point_psm3_left=None #Point to touch with psm
        self.project_point_psm3_right=None #Point to touch with psm


        self.p_ecm_ac_list_psm1=None   #True points locations in ecm coord 
        self.p_ecm_rep_list_psm1=None  #Reported point locations in ecm coord

        self.p_ecm_ac_list_psm3=None
        self.p_ecm_rep_list_psm3=None

        self.p_ecm_ac_list_psm1_right=None   #True points locations in ecm coord 
        self.p_ecm_rep_list_psm1_right=None  #Reported point locations in ecm coord

        self.p_ecm_ac_list_psm3_right=None
        self.p_ecm_rep_list_psm3_right=None

        self.is_new_psm1_points=False
        self.is_new_psm3_points=False



        
        ##Time Variables
        self.delta_time=None
        self.record_time=0 #For recording
        self.playback_time=0 #For playback (playback is a continuous loop)
        self.pc2_time=None

        self.start_time_calib=time.time()

        ##Camera to Scene Valid Vars
        self.is_camToScene_started=False
        self.is_CamToScene_Single=False
        self.is_CamToScene_Multiple=False
        self.calibrate_on_right=False
        self.calibrate_on_left=False
        self.num_frames_captured=0
        self.csv_name_camtoscene_left=None
        self.csv_name_camtoscene_right=None

        ##Pose Validation VARS
        self.PSM1_on=False
        self.PSM3_on=False
        self.is_PoseValidStarted=False
        self.is_GrabPose=False
        self.csv_name_s_T_PSM_kinematics_withErrCorr=None
        self.csv_name_s_T_PSM_visual_withErrCorr=None
        self.csv_name_lc_T_psm_withErrCorr=None
        self.csv_name_lc_T_psm_withoutErrCorr=None
        self.delta_Time_PoseValid=0




        ###########Converts the dictionary of model (scene) points to a list of list of points
        ARUCO_IDs=[6,4,5,7]
        self.model_scene_points=None
        for id in ARUCO_IDs:
            curr_points=ArucoTracker.RINGOWIRE_MODELPOINTS[str(id)]
            if self.model_scene_points is None:
                self.model_scene_points=curr_points
            else:
                self.model_scene_points=np.vstack((self.model_scene_points,curr_points))


        ####################Init Transforms#######################

        #Camera to scene registration Initial
        self.lci_T_si=None
        self.rci_T_si=None
        self.lc_T_s=None
        self.rc_T_s=None

        self.inv_lci_T_si=None
        self.inv_rci_T_si=None

        #Camera to scene registration continuous (published from PC2)
        self.lc_T_s=None
        self.rc_T_s=None

        self.inv_lc_T_s=None
        self.inv_rc_T_s=None

        #Initial ECM Pose
        self.cart_T_ecmi=None
        self.inv_cart_T_ecmi=None
        ecmi_T_ecm=None
        


        ###Getting hand-eye calibration matrices
        with open(ArucoTracker.DEFAULT_CAMCALIB_DIR+'calibration_params_right/hand_eye_calibration_right.yaml','r') as file:
            right_handeye=yaml.load(file)
            file.close()

        with open(ArucoTracker.DEFAULT_CAMCALIB_DIR+'calibration_params_left/hand_eye_calibration_left.yaml','r') as file:
            left_handeye=yaml.load(file)
            file.close()

        self.ecm_T_lc_np=left_handeye['ecm_T_leftcam']
        self.ecm_T_lc_np=np.array(self.ecm_T_lc_np,dtype='float32')
        
        self.ecm_T_lc_np=utils.EnforceOrthogonalityNumpy_FullTransform(self.ecm_T_lc_np)
        self.ecm_T_lc=glm.mat4(*self.ecm_T_lc_np.T.flatten())
        print("ecm_T_lc: "+str(self.ecm_T_lc))
        
        self.ecm_T_rc_np=right_handeye['ecm_T_rightcam']
        self.ecm_T_rc_np=np.array(self.ecm_T_rc_np,dtype='float32')
        
        self.ecm_T_rc_np=utils.EnforceOrthogonalityNumpy_FullTransform(self.ecm_T_rc_np)
        self.ecm_T_rc=glm.mat4(*self.ecm_T_rc_np.T.flatten())
        print("ecm_T_rc: "+str(self.ecm_T_rc))

        self.inv_ecm_T_lc=utils.invHomogeneousGLM(self.ecm_T_lc)
        self.inv_ecm_T_rc=utils.invHomogeneousGLM(self.ecm_T_rc)



        ###API Error Correction:

        #PSM1 Left Error Correction
        self.ecmac_T_ecmrep_psm1=None #The API error correction factor for psm1
        self.ecmac_T_ecmrep_psm1_np=None

        if os.path.isfile(ArucoTracker.DEFAULT_CAMCALIB_DIR+'API_Error_Offset/ecmac_T_ecmrep_psm1.yaml'): 
            with open(ArucoTracker.DEFAULT_CAMCALIB_DIR+'API_Error_Offset/ecmac_T_ecmrep_psm1.yaml','r') as file:
                self.ecmac_T_ecmrep_psm1_np=yaml.load(file) 

                self.ecmac_T_ecmrep_psm1_np=self.ecmac_T_ecmrep_psm1_np['ecmac_T_ecmrep_psm1']
                self.ecmac_T_ecmrep_psm1_np=np.array(self.ecmac_T_ecmrep_psm1_np,dtype=np.float32)

                self.ecmac_T_ecmrep_psm1=glm.mat4(*self.ecmac_T_ecmrep_psm1_np.T.flatten())    
                print('ecmac_T_ecmrep_psm1: '+str(self.ecmac_T_ecmrep_psm1)) 

                self.inv_ecmac_T_ecmrep_psm1=utils.invHomogeneousGLM(self.ecmac_T_ecmrep_psm1)    
                self.is_psmerror_calib=True 
                file.close()

        #PSM3 Left Error Correction
        self.ecmac_T_ecmrep_psm3=None #The API error correction factor for psm1
        self.ecmac_T_ecmrep_psm3_np=None

        if os.path.isfile(ArucoTracker.DEFAULT_CAMCALIB_DIR+'API_Error_Offset/ecmac_T_ecmrep_psm3.yaml'): 
            with open(ArucoTracker.DEFAULT_CAMCALIB_DIR+'API_Error_Offset/ecmac_T_ecmrep_psm3.yaml','r') as file:
                self.ecmac_T_ecmrep_psm3_np=yaml.load(file) 

                self.ecmac_T_ecmrep_psm3_np=self.ecmac_T_ecmrep_psm3_np['ecmac_T_ecmrep_psm3']
                self.ecmac_T_ecmrep_psm3_np=np.array(self.ecmac_T_ecmrep_psm3_np,dtype=np.float32)

                self.ecmac_T_ecmrep_psm3=glm.mat4(*self.ecmac_T_ecmrep_psm3_np.T.flatten())    
                print('ecmac_T_ecmrep_psm3: '+str(self.ecmac_T_ecmrep_psm3))     
                self.inv_ecmac_T_ecmrep_psm3=utils.invHomogeneousGLM(self.ecmac_T_ecmrep_psm3)
                self.is_psmerror_calib=True 
                file.close()


        #PSM1 Right Error Correction
        self.ecmac_T_ecmrep_psm1_right=None #The API error correction factor for psm1
        self.ecmac_T_ecmrep_psm1_right_np=None

        if os.path.isfile(ArucoTracker.DEFAULT_CAMCALIB_DIR+'API_Error_Offset/ecmac_T_ecmrep_psm1_right.yaml'): 
            with open(ArucoTracker.DEFAULT_CAMCALIB_DIR+'API_Error_Offset/ecmac_T_ecmrep_psm1_right.yaml','r') as file:
                self.ecmac_T_ecmrep_psm1_right_np=yaml.load(file) 

                self.ecmac_T_ecmrep_psm1_right_np=self.ecmac_T_ecmrep_psm1_right_np['ecmac_T_ecmrep_psm1_right']
                self.ecmac_T_ecmrep_psm1_right_np=np.array(self.ecmac_T_ecmrep_psm1_right_np,dtype=np.float32)

                self.ecmac_T_ecmrep_psm1_right=glm.mat4(*self.ecmac_T_ecmrep_psm1_right_np.T.flatten())    
                print('ecmac_T_ecmrep_psm1_right: '+str(self.ecmac_T_ecmrep_psm1_right)) 

                self.inv_ecmac_T_ecmrep_psm1_right=utils.invHomogeneousGLM(self.ecmac_T_ecmrep_psm1_right)    
                self.is_psmerror_calib=True 
                file.close()

        #PSM3 Right Error Correction
        self.ecmac_T_ecmrep_psm3_right=None #The API error correction factor for psm1
        self.ecmac_T_ecmrep_psm3_right_np=None

        if os.path.isfile(ArucoTracker.DEFAULT_CAMCALIB_DIR+'API_Error_Offset/ecmac_T_ecmrep_psm3_right.yaml'): 
            with open(ArucoTracker.DEFAULT_CAMCALIB_DIR+'API_Error_Offset/ecmac_T_ecmrep_psm3_right.yaml','r') as file:
                self.ecmac_T_ecmrep_psm3_right_np=yaml.load(file) 

                self.ecmac_T_ecmrep_psm3_right_np=self.ecmac_T_ecmrep_psm3_right_np['ecmac_T_ecmrep_psm3_right']
                self.ecmac_T_ecmrep_psm3_right_np=np.array(self.ecmac_T_ecmrep_psm3_right_np,dtype=np.float32)

                self.ecmac_T_ecmrep_psm3_right=glm.mat4(*self.ecmac_T_ecmrep_psm3_right_np.T.flatten())    
                print('ecmac_T_ecmrep_psm3_right: '+str(self.ecmac_T_ecmrep_psm3_right))     
                self.inv_ecmac_T_ecmrep_psm3_right=utils.invHomogeneousGLM(self.ecmac_T_ecmrep_psm3_right)
                self.is_psmerror_calib=True 
                file.close()


        
        #################dVRK API Config###################

        self.psm1=dvrk.psm("PSM1") #Mapped to left hand
        self.psm3=dvrk.psm("PSM3") #Mapped to right hand
        self.ecm=dvrk.ecm("ECM")

        #Enabling and Homing
        self.psm1.enable()
        self.psm1.home()

        self.psm3.enable()
        self.psm3.home()

        self.ecm.enable()
        self.ecm.home()


        rospy.sleep(1)


    def lcTs_Callback(self,data):
        #Extracts the list
        lc_T_s_list=data.data
        lc_T_s_numpy=np.array(lc_T_s_list,dtype='float32').reshape(4,4)

        #Computes average of filter buffer, returned as glm mat
        self.lc_T_s=glm.mat4(*lc_T_s_numpy.T.flatten())

        self.inv_lc_T_s=utils.invHomogeneousGLM(self.lc_T_s)

        # print("lc_T_s: "+str(self.lc_T_s))

    def rcTs_Callback(self,data):
        rc_T_s_list=data.data
        rc_T_s_numpy=np.array(rc_T_s_list,dtype='float32').reshape(4,4)

        #Computers filter mean
        self.rc_T_s=glm.mat4(*rc_T_s_numpy.T.flatten())

        self.inv_rc_T_s=utils.invHomogeneousGLM(self.rc_T_s)  
        #print("rc_T_s: "+str(self.rc_T_s))


    def frameCallbackRight(self,data):
        self.frame_right=self.bridge.compressed_imgmsg_to_cv2(data,'passthrough')
        #self.frame_right=cv2.resize(self.frame_right,(ECM_FRAME_WIDTH_DESIRED,ECM_FRAME_HEIGHT_DESIRED),interpolation=cv2.INTER_LINEAR)

    def frameCallbackLeft(self,data):        
        self.frame_left=self.bridge.compressed_imgmsg_to_cv2(data,'passthrough')
        #self.frame_left=cv2.resize(self.frame_left,(ECM_FRAME_WIDTH_DESIRED,ECM_FRAME_HEIGHT_DESIRED),interpolation=cv2.INTER_LINEAR)


    #Cam-to-scene Validation Callbacks

    def startCamToSceneCallback(self):
        if self.is_CamToScene_Single or self.is_CamToScene_Multiple:
            self.is_camToScene_started= not self.is_camToScene_started

            if self.is_camToScene_started:
                self.num_frames_captured=0
                ##########NDI Tracker Setup
                #print("Path To Tracker: "+str(PATH_TO_NDI_APPLE3))
                settings={
                        "tracker type": "polaris",
                        "romfiles": [PATH_TO_NDI_APPLE3]
                    }
                self.ndi_tracker=NDITracker(settings)
                self.ndi_tracker.use_quaternions=False
                self.ndi_tracker.start_tracking()
                print('Started NDI Tracker')

                #Gets if it is single or multiple:
                if self.is_CamToScene_Single:
                    string_text='_single_'

                else:
                    string_text='_multiple_'

                #Create csv to store data (left frame)
                now=datetime.now()
                dt_string=now.strftime("%d-%m-%Y_%H-%M-%S")
                self.csv_name_camtoscene_left=PATH_TO_VALIDATION_CSV+'cam_to_scene'+string_text+'left_'+dt_string+'.csv'

                #Writes the header
                with open(self.csv_name_camtoscene_left,'w',newline='') as file_object:
                    writer_object=csv.writer(file_object)
                    writer_object.writerow(CAM_TO_SCENE_VALIDATION_HEADER)
                    file_object.close()


                #Create csv to store data (right frame)
                now=datetime.now()
                dt_string=now.strftime("%d-%m-%Y_%H-%M-%S")
                self.csv_name_camtoscene_right=PATH_TO_VALIDATION_CSV+'cam_to_scene'+string_text+'right_'+dt_string+'.csv'

                #Writes the header
                with open(self.csv_name_camtoscene_right,'w',newline='') as file_object:
                    writer_object=csv.writer(file_object)
                    writer_object.writerow(CAM_TO_SCENE_VALIDATION_HEADER)
                    file_object.close()

            else:
                self.ndi_tracker.stop_tracking()
                self.ndi_tracker.close()
                print('Stopped NDI Tracker')
                self.num_frames_captured=0
                self.csv_name_camtoscene_left=None
                self.csv_name_camtoscene_right=None


    def camToSceneSingleCallback(self):
        self.is_CamToScene_Single=not self.is_CamToScene_Single

    def camToSceneMultipleCallback(self):
        self.is_CamToScene_Multiple=not self.is_CamToScene_Multiple  

    def calibrateSceneCallback(self):
        self.calibrate_on_left=not self.calibrate_on_left
        self.calibrate_on_right=not self.calibrate_on_right
        
        if self.calibrate_on_left:
            
            self.num_frames_captured+=1

            self.aruco_tracker_left.calibrate_done=False
            self.aruco_tracker_left.ci_T_si=None
            self.aruco_tracker_left.corner_scene_list=[]
            self.aruco_tracker_left.ids_scene_list=[]

            self.aruco_tracker_right.calibrate_done=False
            self.aruco_tracker_right.ci_T_si=None
            self.aruco_tracker_right.corner_scene_list=[]
            self.aruco_tracker_right.ids_scene_list=[]
            self.start_time_calib=time.time()


    #####PSM Pose Validation Callbacks
    def psm1Checkbox(self):
        self.PSM1_on=not self.PSM1_on

    def psm3Checkbox(self):
        self.PSM3_on=not self.PSM3_on 
    
    def grabPSMPoseCallback(self):
        self.is_GrabPose = not self.is_GrabPose
        self.num_frames_captured+=1
        self.delta_Time_PoseValid=0

    
    def startPSMPoseCallback(self):
        self.is_PoseValidStarted=not self.is_PoseValidStarted

        if self.is_PoseValidStarted:
            self.num_frames_captured=0
            ##########NDI Tracker Setup
            #print("Path To Tracker: "+str(PATH_TO_NDI_APPLE3))
            settings={
                    "tracker type": "polaris",
                    "romfiles": [PATH_TO_NDI_APPLE3]
                }
            self.ndi_tracker=NDITracker(settings)
            self.ndi_tracker.use_quaternions=False
            self.ndi_tracker.start_tracking()
            print('Started NDI Tracker')

            #Gets if it is single or multiple:
            if self.PSM1_on:
                string_text='_PSM1_'
            elif self.PSM3_on:
                string_text='_PSM3_'

            #Create csv to store data (Kinematics based)
            now=datetime.now()
            dt_string=now.strftime("%d-%m-%Y_%H-%M-%S")
            self.csv_name_s_T_PSM_kinematics_withErrCorr=PATH_TO_POSE_VALIDATION_CSV+'s_T_PSM'+string_text+'kinematics_withErrCorr_'+dt_string+'.csv'
            self.csv_name_s_T_PSM_visual_withErrCorr=PATH_TO_POSE_VALIDATION_CSV+'s_T_PSM'+string_text+'visual_withErrCorr_'+dt_string+'.csv'
            
            self.csv_name_lc_T_psm_withErrCorr=PATH_TO_POSE_VALIDATION_CSV+'lc_T_psm'+string_text+'_withErrCorr_'+dt_string+'.csv'
            self.csv_name_lc_T_psm_withoutErrCorr=PATH_TO_POSE_VALIDATION_CSV+'lc_T_psm'+string_text+'_withoutErrCorr_'+dt_string+'.csv'

            #Writes the header
            with open(self.csv_name_s_T_PSM_kinematics_withErrCorr,'w',newline='') as file_object:
                writer_object=csv.writer(file_object)
                writer_object.writerow(S_T_PSM_VALIDATION_HEADER)
                file_object.close()

            #Writes the header
            with open(self.csv_name_s_T_PSM_visual_withErrCorr,'w',newline='') as file_object:
                writer_object=csv.writer(file_object)
                writer_object.writerow(S_T_PSM_VALIDATION_HEADER)
                file_object.close()

            #Writes the header
            with open(self.csv_name_lc_T_psm_withErrCorr,'w',newline='') as file_object:
                writer_object=csv.writer(file_object)
                writer_object.writerow(LC_T_PSM_VALIDATION_HEADER)
                file_object.close()
            
            with open(self.csv_name_lc_T_psm_withoutErrCorr,'w',newline='') as file_object:
                writer_object=csv.writer(file_object)
                writer_object.writerow(LC_T_PSM_VALIDATION_HEADER)
                file_object.close()

        else:
            self.ndi_tracker.stop_tracking()
            self.ndi_tracker.close()
            print('Stopped NDI Tracker')
            self.num_frames_captured=0
            self.csv_name_s_T_PSM_kinematics_withErrCorr=None
            self.csv_name_s_T_PSM_visual_withErrCorr=None
            self.csv_name_lc_T_psm_withErrCorr=None
            self.csv_name_lc_T_psm_withoutErrCorr=None
            self.num_frames_captured=0


#########API Error Correction Methods

    def startPSMErrorCalib(self):
        
        #PSM1
        self.psm1_points_count=0
        
        point_si=self.model_scene_points[self.psm1_points_count]
        point_si_list=point_si.tolist()        
        point_si_list.append(1)
        point_si=glm.vec4(point_si_list)

        #Projecting point on left ecm frame to show 
        proj_point=self.projectPointOnImagePlane(self.lci_T_si,point_si,self.aruco_tracker_left.mtx)
        self.project_point_psm1_left=proj_point
        
        #Projecting point on right ecm frame to show
        proj_point=self.projectPointOnImagePlane(self.rci_T_si,point_si,self.aruco_tracker_right.mtx)
        self.project_point_psm1_right=proj_point


        #PSM3
        self.psm3_points_count=0
        
        point_si=self.model_scene_points[self.psm3_points_count]
        point_si_list=point_si.tolist()        
        point_si_list.append(1)
        point_si=glm.vec4(point_si_list)

        #Projecting point on left ecm frame to show 
        proj_point=self.projectPointOnImagePlane(self.lci_T_si,point_si,self.aruco_tracker_left.mtx)
        self.project_point_psm3_left=proj_point
        #Projecting point on right ecm frame to show
        proj_point=self.projectPointOnImagePlane(self.rci_T_si,point_si,self.aruco_tracker_right.mtx)
        self.project_point_psm3_right=proj_point
        
        
        self.is_psmerror_started=True

    def grabPSM1PointCallback(self):
        if self.is_psmerror_started:
            self.is_new_psm1_points=True
            #############Left Camera
            #############Gets actual point location
            point_si=self.model_scene_points[self.psm1_points_count]
            point_si_list=point_si.tolist()        
            point_si_list.append(1)
            point_si=glm.vec4(point_si_list)

            #point_ecm_ac=self.ecm_T_lc*self.lci_T_si*point_si
            point_ecm_ac=self.lci_T_si*point_si

            point_ecm_ac_list=[point_ecm_ac[0],point_ecm_ac[1],point_ecm_ac[2]]

            point_ecm_ac=np.array(point_ecm_ac_list,dtype=np.float32)

            if self.p_ecm_ac_list_psm1 is None:
                self.p_ecm_ac_list_psm1=point_ecm_ac
            else:
                self.p_ecm_ac_list_psm1=np.vstack((self.p_ecm_ac_list_psm1,point_ecm_ac))

            

            #############Gets Reported Point
            #rospy.sleep(0.5)
            try_true=False
            try:
                ecm_T_psm_rep=self.psm1.setpoint_cp()
                try_true=True
            except Exception as e:
                print("Unable to read psm1: "+str(e))
                return
            
            if try_true:
                ecm_T_psm_rep=utils.enforceOrthogonalPyKDL(ecm_T_psm_rep)
                ecm_T_psm_rep=pm.toMatrix(ecm_T_psm_rep) #Numpy array

                

                point_ecm_rep=utils.invHomogeneousNumpy(self.ecm_T_lc_np)@ecm_T_psm_rep@tool_tip_point
                #point_ecm_rep=ecm_T_psm_rep@tool_tip_point
                #point_ecm_rep=np.matmul(ecm_T_psm_rep,tool_tip_point)
                point_ecm_rep=point_ecm_rep[0:3]

                if self.p_ecm_rep_list_psm1 is None:
                    self.p_ecm_rep_list_psm1=point_ecm_rep
                else:
                    self.p_ecm_rep_list_psm1=np.vstack((self.p_ecm_rep_list_psm1,point_ecm_rep))

                print("point_lc_ac: "+str(point_ecm_rep))
                print("point_lc_rep: "+str(point_ecm_rep))




                ###########Right Camera Next
                point_ecm_ac=self.rci_T_si*point_si

                point_ecm_ac_list=[point_ecm_ac[0],point_ecm_ac[1],point_ecm_ac[2]]

                point_ecm_ac=np.array(point_ecm_ac_list,dtype=np.float32)

                if self.p_ecm_ac_list_psm1_right is None:
                    self.p_ecm_ac_list_psm1_right=point_ecm_ac
                else:
                    self.p_ecm_ac_list_psm1_right=np.vstack((self.p_ecm_ac_list_psm1_right,point_ecm_ac))


                point_ecm_rep=utils.invHomogeneousNumpy(self.ecm_T_rc_np)@ecm_T_psm_rep@tool_tip_point
                #point_ecm_rep=ecm_T_psm_rep@tool_tip_point
                #point_ecm_rep=np.matmul(ecm_T_psm_rep,tool_tip_point)
                point_ecm_rep=point_ecm_rep[0:3]

                if self.p_ecm_rep_list_psm1_right is None:
                    self.p_ecm_rep_list_psm1_right=point_ecm_rep
                else:
                    self.p_ecm_rep_list_psm1_right=np.vstack((self.p_ecm_rep_list_psm1_right,point_ecm_rep))

                print("point_lc_ac: "+str(point_ecm_rep))
                print("point_lc_rep: "+str(point_ecm_rep))


                ###########Updates Visual Point for next iteration 
                self.psm1_points_count+=1
                point_si=self.model_scene_points[self.psm1_points_count]
                point_si_list=point_si.tolist()        
                point_si_list.append(1)
                point_si=glm.vec4(point_si_list)
                #Projecting point on left ecm frame to show 
                proj_point=self.projectPointOnImagePlane(self.lci_T_si,point_si,self.aruco_tracker_left.mtx)
                self.project_point_psm1_left=proj_point
                #Projecting point on right ecm frame to show
                proj_point=self.projectPointOnImagePlane(self.rci_T_si,point_si,self.aruco_tracker_right.mtx)
                self.project_point_psm1_right=proj_point
    
    def grabPSM3PointCallback(self):
        if self.is_psmerror_started:
            self.is_new_psm3_points=True

            #############Gets actual point location
            point_si=self.model_scene_points[self.psm3_points_count]
            point_si_list=point_si.tolist()        
            point_si_list.append(1)
            point_si=glm.vec4(point_si_list)
            

            ##############Left Camera First############
            #point_ecm_ac=self.ecm_T_lc*self.lci_T_si*point_si
            point_ecm_ac=self.lci_T_si*point_si

            point_ecm_ac_list=[point_ecm_ac[0],point_ecm_ac[1],point_ecm_ac[2]]

            point_ecm_ac=np.array(point_ecm_ac_list,dtype=np.float32)

            if self.p_ecm_ac_list_psm3 is None:
                self.p_ecm_ac_list_psm3=point_ecm_ac
            else:
                self.p_ecm_ac_list_psm3=np.vstack((self.p_ecm_ac_list_psm3,point_ecm_ac))

            

            #############Gets Reported Point for left camera
            #rospy.sleep(0.5)
            try_true=False
            try:
                ecm_T_psm_rep=self.psm3.setpoint_cp()
                try_true=True
            except Exception as e:
                print("Unable to read psm1: "+str(e))
                return
            
            if try_true:
                ecm_T_psm_rep=utils.enforceOrthogonalPyKDL(ecm_T_psm_rep)
                ecm_T_psm_rep=pm.toMatrix(ecm_T_psm_rep) #Numpy array


                # point_lc_rep=utils.invHomogeneousNumpy(self.ecm_T_lc_np)@ecm_T_psm_rep@tool_tip_point
                point_ecm_rep=utils.invHomogeneousNumpy(self.ecm_T_lc_np)@ecm_T_psm_rep@tool_tip_point
                #point_ecm_rep=ecm_T_psm_rep@tool_tip_point
                #point_ecm_rep=np.matmul(ecm_T_psm_rep,tool_tip_point)
                point_ecm_rep=point_ecm_rep[0:3]

                if self.p_ecm_rep_list_psm3 is None:
                    self.p_ecm_rep_list_psm3=point_ecm_rep
                else:
                    self.p_ecm_rep_list_psm3=np.vstack((self.p_ecm_rep_list_psm3,point_ecm_rep))

                print("point_lc_ac: "+str(point_ecm_rep))
                print("point_lc_rep: "+str(point_ecm_rep))


                ##############Right Camera Next
                #point_ecm_ac=self.ecm_T_lc*self.lci_T_si*point_si
                point_ecm_ac=self.rci_T_si*point_si

                point_ecm_ac_list=[point_ecm_ac[0],point_ecm_ac[1],point_ecm_ac[2]]

                point_ecm_ac=np.array(point_ecm_ac_list,dtype=np.float32)

                if self.p_ecm_ac_list_psm3_right is None:
                    self.p_ecm_ac_list_psm3_right=point_ecm_ac
                else:
                    self.p_ecm_ac_list_psm3_right=np.vstack((self.p_ecm_ac_list_psm3_right,point_ecm_ac))

                

                #############Gets Reported Point for right camera
                #rospy.sleep(0.5)

                # point_lc_rep=utils.invHomogeneousNumpy(self.ecm_T_lc_np)@ecm_T_psm_rep@tool_tip_point
                point_ecm_rep=utils.invHomogeneousNumpy(self.ecm_T_rc_np)@ecm_T_psm_rep@tool_tip_point
                #point_ecm_rep=ecm_T_psm_rep@tool_tip_point
                #point_ecm_rep=np.matmul(ecm_T_psm_rep,tool_tip_point)
                point_ecm_rep=point_ecm_rep[0:3]

                if self.p_ecm_rep_list_psm3_right is None:
                    self.p_ecm_rep_list_psm3_right=point_ecm_rep
                else:
                    self.p_ecm_rep_list_psm3_right=np.vstack((self.p_ecm_rep_list_psm3_right,point_ecm_rep))

                print("point_rc_ac: "+str(point_ecm_rep))
                print("point_rc_rep: "+str(point_ecm_rep))


            ###########Updates Visual Point for next iteration 
            self.psm3_points_count+=1
            point_si=self.model_scene_points[self.psm3_points_count]
            point_si_list=point_si.tolist()        
            point_si_list.append(1)
            point_si=glm.vec4(point_si_list)
            #Projecting point on left ecm frame to show 
            proj_point=self.projectPointOnImagePlane(self.lci_T_si,point_si,self.aruco_tracker_left.mtx)
            self.project_point_psm3_left=proj_point
            #Projecting point on right ecm frame to show
            proj_point=self.projectPointOnImagePlane(self.rci_T_si,point_si,self.aruco_tracker_right.mtx)
            self.project_point_psm3_right=proj_point

    def calibratePSMErrorCallback(self):
        if not os.path.isdir(ArucoTracker.DEFAULT_CAMCALIB_DIR+'API_Error_Offset/'): #Creates store directory
            os.mkdir(ArucoTracker.DEFAULT_CAMCALIB_DIR+'API_Error_Offset/')

        #PSM1 Left:
        #Store the psm1 points first (both lc and rc)
        if self.is_new_psm1_points:
            np.save(ArucoTracker.DEFAULT_CAMCALIB_DIR+'API_Error_Offset/p_ecm_ac_list_psm1.npy',self.p_ecm_ac_list_psm1)
            np.save(ArucoTracker.DEFAULT_CAMCALIB_DIR+'API_Error_Offset/p_ecm_rep_list_psm1.npy',self.p_ecm_rep_list_psm1)
        else:
            self.p_ecm_ac_list_psm1=np.load(ArucoTracker.DEFAULT_CAMCALIB_DIR+'API_Error_Offset/p_ecm_ac_list_psm1.npy')
            self.p_ecm_rep_list_psm1=np.load(ArucoTracker.DEFAULT_CAMCALIB_DIR+'API_Error_Offset/p_ecm_rep_list_psm1.npy')


        #Finding the error compensation transform, psm1 lc (psm1 left camera)

        psm1_lc_points_shape=self.p_ecm_rep_list_psm1.shape


        if psm1_lc_points_shape is not None:
            if psm1_lc_points_shape[0]>3:
                


                print("p_ecm_ac_list_psm1: "+str(self.p_ecm_ac_list_psm1))
                print("p_ecm_rep_list_psm1: "+str(self.p_ecm_rep_list_psm1))
                self.ecmac_T_ecmrep_psm1_np,_=utils.ransacRigidRransformation(self.p_ecm_ac_list_psm1,self.p_ecm_rep_list_psm1)


                #Saving Results
                ecmac_T_ecmrep_psm1_list=self.ecmac_T_ecmrep_psm1_np.tolist()
                data_psm1={'ecmac_T_ecmrep_psm1':ecmac_T_ecmrep_psm1_list}
                with open(ArucoTracker.DEFAULT_CAMCALIB_DIR+'API_Error_Offset/ecmac_T_ecmrep_psm1.yaml','w') as f:
                    yaml.dump(data_psm1,f)
                    f.close()

                self.ecmac_T_ecmrep_psm1=glm.mat4(*self.ecmac_T_ecmrep_psm1_np.T.flatten())
                self.inv_ecmac_T_ecmrep_psm1=utils.invHomogeneousGLM(self.ecmac_T_ecmrep_psm1)

                #Finding translation error:
                translation_diff_list=[]
                for i in range(psm1_lc_points_shape[0]): #Loops for the number of points captured
                    p_lc_rep_point=self.p_ecm_rep_list_psm1[i].tolist()
                    p_lc_rep_point.append(1)
                    p_lc_rep_point=np.array(p_lc_rep_point,dtype=np.float32)

                    est_point=self.ecmac_T_ecmrep_psm1_np@p_lc_rep_point
                    #est_point=np.matmul(self.ecmac_T_ecmrep_psm1_np,p_lc_rep_point)
                    est_point=est_point[0:3]


                    trans_diff=est_point-self.p_ecm_ac_list_psm1[i]
                    trans_diff=np.linalg.norm(trans_diff)
                    translation_diff_list.append(trans_diff)
                translation_diff_list=np.array(translation_diff_list,dtype=np.float32)
                trans_diff=np.mean(translation_diff_list)
                print('registration error psm1 lc: '+str(trans_diff))

                self.is_psmerror_calib=True

        #PSM1 Right:
        #Store the psm1 points first (both lc and rc)
        if self.is_new_psm1_points:
            np.save(ArucoTracker.DEFAULT_CAMCALIB_DIR+'API_Error_Offset/p_ecm_ac_list_psm1_right.npy',self.p_ecm_ac_list_psm1_right)
            np.save(ArucoTracker.DEFAULT_CAMCALIB_DIR+'API_Error_Offset/p_ecm_rep_list_psm1_right.npy',self.p_ecm_rep_list_psm1_right)
        else:
            self.p_ecm_ac_list_psm1_right=np.load(ArucoTracker.DEFAULT_CAMCALIB_DIR+'API_Error_Offset/p_ecm_ac_list_psm1_right.npy')
            self.p_ecm_rep_list_psm1_right=np.load(ArucoTracker.DEFAULT_CAMCALIB_DIR+'API_Error_Offset/p_ecm_rep_list_psm1_right.npy')


        #Finding the error compensation transform, psm1 lc (psm1 left camera)

        psm1_rc_points_shape=self.p_ecm_rep_list_psm1_right.shape


        if psm1_rc_points_shape is not None:
            if psm1_rc_points_shape[0]>3:
                


                print("p_ecm_ac_list_psm1_right: "+str(self.p_ecm_ac_list_psm1_right))
                print("p_ecm_rep_list_psm1_right: "+str(self.p_ecm_rep_list_psm1_right))
                self.ecmac_T_ecmrep_psm1_right_np,_=utils.ransacRigidRransformation(self.p_ecm_ac_list_psm1_right,self.p_ecm_rep_list_psm1_right)


                #Saving Results
                ecmac_T_ecmrep_psm1_right_list=self.ecmac_T_ecmrep_psm1_right_np.tolist()
                data_psm1={'ecmac_T_ecmrep_psm1_right':ecmac_T_ecmrep_psm1_right_list}
                with open(ArucoTracker.DEFAULT_CAMCALIB_DIR+'API_Error_Offset/ecmac_T_ecmrep_psm1_right.yaml','w') as f:
                    yaml.dump(data_psm1,f)
                    f.close()

                self.ecmac_T_ecmrep_psm1_right=glm.mat4(*self.ecmac_T_ecmrep_psm1_right_np.T.flatten())
                self.inv_ecmac_T_ecmrep_psm1_right=utils.invHomogeneousGLM(self.ecmac_T_ecmrep_psm1_right)

                #Finding translation error:
                translation_diff_list=[]
                for i in range(psm1_rc_points_shape[0]): #Loops for the number of points captured
                    p_lc_rep_point=self.p_ecm_rep_list_psm1_right[i].tolist()
                    p_lc_rep_point.append(1)
                    p_lc_rep_point=np.array(p_lc_rep_point,dtype=np.float32)

                    est_point=self.ecmac_T_ecmrep_psm1_right_np@p_lc_rep_point
                    #est_point=np.matmul(self.ecmac_T_ecmrep_psm1_np,p_lc_rep_point)
                    est_point=est_point[0:3]


                    trans_diff=est_point-self.p_ecm_ac_list_psm1_right[i]
                    trans_diff=np.linalg.norm(trans_diff)
                    translation_diff_list.append(trans_diff)
                translation_diff_list=np.array(translation_diff_list,dtype=np.float32)
                trans_diff=np.mean(translation_diff_list)
                print('registration error psm1 rc: '+str(trans_diff))

                self.is_psmerror_calib=True


        #PSM3 Left:
        #Store the psm1 points first (both lc and rc)
        if self.is_new_psm3_points:
            np.save(ArucoTracker.DEFAULT_CAMCALIB_DIR+'API_Error_Offset/p_ecm_ac_list_psm3.npy',self.p_ecm_ac_list_psm3)
            np.save(ArucoTracker.DEFAULT_CAMCALIB_DIR+'API_Error_Offset/p_ecm_rep_list_psm3.npy',self.p_ecm_rep_list_psm3)
        else:
            self.p_ecm_ac_list_psm3=np.load(ArucoTracker.DEFAULT_CAMCALIB_DIR+'API_Error_Offset/p_ecm_ac_list_psm3.npy')
            self.p_ecm_rep_list_psm3=np.load(ArucoTracker.DEFAULT_CAMCALIB_DIR+'API_Error_Offset/p_ecm_rep_list_psm3.npy')


        #Finding the error compensation transform, psm1 lc (psm1 left camera)

        psm3_lc_points_shape=self.p_ecm_rep_list_psm3.shape


        if psm3_lc_points_shape is not None:
            if psm3_lc_points_shape[0]>3:
                


                print("p_ecm_ac_list_psm3: "+str(self.p_ecm_ac_list_psm3))
                print("p_ecm_rep_list_psm3: "+str(self.p_ecm_rep_list_psm3))
                self.ecmac_T_ecmrep_psm3_np,_=utils.ransacRigidRransformation(self.p_ecm_ac_list_psm3,self.p_ecm_rep_list_psm3)

                #Saving Results
                ecmac_T_ecmrep_psm3_list=self.ecmac_T_ecmrep_psm3_np.tolist()
                data_psm3={'ecmac_T_ecmrep_psm3':ecmac_T_ecmrep_psm3_list}
                with open(ArucoTracker.DEFAULT_CAMCALIB_DIR+'API_Error_Offset/ecmac_T_ecmrep_psm3.yaml','w') as f:
                    yaml.dump(data_psm3,f)
                    f.close()

                self.ecmac_T_ecmrep_psm3=glm.mat4(*self.ecmac_T_ecmrep_psm3_np.T.flatten())
                self.inv_ecmac_T_ecmrep_psm3=utils.invHomogeneousGLM(self.ecmac_T_ecmrep_psm3)

                #Finding translation error:
                translation_diff_list=[]
                for i in range(psm3_lc_points_shape[0]): #Loops for the number of points captured
                    p_lc_rep_point=self.p_ecm_rep_list_psm3[i].tolist()
                    p_lc_rep_point.append(1)
                    p_lc_rep_point=np.array(p_lc_rep_point,dtype=np.float32)

                    est_point=self.ecmac_T_ecmrep_psm3_np@p_lc_rep_point
                    #est_point=np.matmul(self.ecmac_T_ecmrep_psm3_np,p_lc_rep_point)
                    est_point=est_point[0:3]


                    trans_diff=est_point-self.p_ecm_ac_list_psm3[i]
                    trans_diff=np.linalg.norm(trans_diff)
                    translation_diff_list.append(trans_diff)
                translation_diff_list=np.array(translation_diff_list,dtype=np.float32)
                trans_diff=np.mean(translation_diff_list)
                print('registration error psm3 lc: '+str(trans_diff))
                self.is_psmerror_calib=True
  
        #PSM3 Right:
        #Store the psm3 points first (both lc and rc)
        if self.is_new_psm3_points:
            np.save(ArucoTracker.DEFAULT_CAMCALIB_DIR+'API_Error_Offset/p_ecm_ac_list_psm3_right.npy',self.p_ecm_ac_list_psm3_right)
            np.save(ArucoTracker.DEFAULT_CAMCALIB_DIR+'API_Error_Offset/p_ecm_rep_list_psm3_right.npy',self.p_ecm_rep_list_psm3_right)
        else:
            self.p_ecm_ac_list_psm3_right=np.load(ArucoTracker.DEFAULT_CAMCALIB_DIR+'API_Error_Offset/p_ecm_ac_list_psm3_right.npy')
            self.p_ecm_rep_list_psm3_right=np.load(ArucoTracker.DEFAULT_CAMCALIB_DIR+'API_Error_Offset/p_ecm_rep_list_psm3_right.npy')


        #Finding the error compensation transform, psm1 lc (psm1 left camera)

        psm3_rc_points_shape=self.p_ecm_rep_list_psm3_right.shape


        if psm3_rc_points_shape is not None:
            if psm3_rc_points_shape[0]>3:
                


                print("p_ecm_ac_list_psm3_right: "+str(self.p_ecm_ac_list_psm3_right))
                print("p_ecm_rep_list_psm3_right: "+str(self.p_ecm_rep_list_psm3_right))
                self.ecmac_T_ecmrep_psm3_right_np,_=utils.ransacRigidRransformation(self.p_ecm_ac_list_psm3_right,self.p_ecm_rep_list_psm3_right)


                #Saving Results
                ecmac_T_ecmrep_psm3_right_list=self.ecmac_T_ecmrep_psm3_right_np.tolist()
                data_psm3={'ecmac_T_ecmrep_psm3_right':ecmac_T_ecmrep_psm3_right_list}
                with open(ArucoTracker.DEFAULT_CAMCALIB_DIR+'API_Error_Offset/ecmac_T_ecmrep_psm3_right.yaml','w') as f:
                    yaml.dump(data_psm3,f)
                    f.close()

                self.ecmac_T_ecmrep_psm3_right=glm.mat4(*self.ecmac_T_ecmrep_psm3_right_np.T.flatten())
                self.inv_ecmac_T_ecmrep_psm3_right=utils.invHomogeneousGLM(self.ecmac_T_ecmrep_psm3_right)

                #Finding translation error:
                translation_diff_list=[]
                for i in range(psm3_rc_points_shape[0]): #Loops for the number of points captured
                    p_lc_rep_point=self.p_ecm_rep_list_psm3_right[i].tolist()
                    p_lc_rep_point.append(1)
                    p_lc_rep_point=np.array(p_lc_rep_point,dtype=np.float32)

                    est_point=self.ecmac_T_ecmrep_psm3_right_np@p_lc_rep_point
                    #est_point=np.matmul(self.ecmac_T_ecmrep_psm1_np,p_lc_rep_point)
                    est_point=est_point[0:3]


                    trans_diff=est_point-self.p_ecm_ac_list_psm3_right[i]
                    trans_diff=np.linalg.norm(trans_diff)
                    translation_diff_list.append(trans_diff)
                translation_diff_list=np.array(translation_diff_list,dtype=np.float32)
                trans_diff=np.mean(translation_diff_list)
                print('registration error psm3 rc: '+str(trans_diff))

                self.is_psmerror_calib=True


 ######################Support Methods#####################
    
    #Method to project points on camera frame:
    def projectPointOnImagePlane(self,ci_T_si,point_si,mtx):
        #Input: ci_T_si (transform from camera to scene), point_si (point in scene frame), mtx (camera intrinsics)
        #Output: point projected on image plane
        cam_mat=mtx 
        cam_mat=glm.mat3(*cam_mat.T.flatten()) #3x3 camera intrinsics, numpy=>glm transpose the matrix

        test_point_camera=ci_T_si*point_si #Gets point in camera coordinate system

        test_point_camera=mul_mat*test_point_camera #Extracts top three entries [Xc,Yc,Zc]
        proj_point=cam_mat*test_point_camera #Projects to image plane

        proj_point[0]=proj_point[0]/proj_point[2] #Normalize by Zc
        proj_point[1]=proj_point[1]/proj_point[2]

        return proj_point


    def on_key_press(self,symbol,modifiers):

        print("Key Pressed")
        if symbol==key.ESCAPE: #Close app when escape key is pressed
            print("Escape Pressed")
            self.mesh.destroy()
            self.window_left.close()
            self.window_right.close()
            self.ctx_right.release()
            self.ctx_left.release()
            self.vertex_array_left.release()
            self.texture_left.release()
            self.gui_window.destroy()


        #Update dict  that key is down
        if symbol==key.W:
            self.key_dict['W']=True
        if symbol==key.S:
            self.key_dict['S']=True
        if symbol==key.A:
            self.key_dict['A']=True
        if symbol==key.D:
            self.key_dict['D']=True
        if symbol==key.Q:
            self.key_dict['Q']=True
        if symbol==key.E:
            self.key_dict['E']=True


    def on_key_release(self,symbol,modifiers):
        print("Key Released")
        #Update dict  that key is released
        if symbol==key.W:
            self.key_dict['W']=False
        if symbol==key.S:
            self.key_dict['S']=False
        if symbol==key.A:
            self.key_dict['A']=False
        if symbol==key.D:
            self.key_dict['D']=False
        if symbol==key.Q:
            self.key_dict['Q']=False
        if symbol==key.E:
            self.key_dict['E']=False

    def writeFramesToCSV(self,compute_time,NDI_dat,frame,csv_name,count):

        bool_check=False
        print("NDI Data: ")
        print(NDI_dat)
        timestamp=NDI_dat[1]    #Timestamp
        print("Timestamp: "+str(timestamp))
        transform_list=NDI_dat[3] #Transform
        quality=NDI_dat[4]  #Tracking Quality
        print("quality: "+str(quality))

        data_list=[" "]*29 #Initializes the list of data as empty spaces

        data_list[0]=timestamp[0]
        data_list[1]=count
        data_list[2]=compute_time

        if len(transform_list)>0: #Quaternion is found                            
            transform_list=transform_list[0]
            data_list[4:16]=self.convertHomogeneousToCSVROW(transform_list)
            #print("Saved NDI Data: ")
            #print(data_list[4:16])
            data_list[16]=quality[0]
            if frame is not None:
                
                frame_numpy=np.array(glm.transpose(frame).to_list(),dtype='float32')
               #print("Frame Data: ")
                #print(frame_numpy)
                frame_list=self.convertHomogeneousToCSVROW(frame_numpy)

                data_list[18:-1]=frame_list
                #print("Saved Frame Data: ")
                #print(data_list[18:-1])
                bool_check=True
                #Write the row:
                with open(csv_name,'a',newline='') as file_object:
                    writer_object=csv.writer(file_object)
                    writer_object.writerow(data_list)
                    file_object.close()
        print("Total Data: ")
        print(data_list)
        return bool_check
    
    def writeFramesToCSV_Aruco(self,compute_time,aruco_frame,frame,csv_name,count):

        bool_check=False

        timestamp=0    #Timestamp

        data_list=[" "]*29 #Initializes the list of data as empty spaces

        data_list[0]=timestamp
        data_list[1]=count
        data_list[2]=compute_time

        if aruco_frame is not None:                        
            aruco_frame_numpy=np.array(glm.transpose(aruco_frame).to_list(),dtype='float32')
            aruco_frame_list=self.convertHomogeneousToCSVROW(aruco_frame_numpy)
            data_list[4:14]=aruco_frame_list

            if frame is not None:
                
                frame_numpy=np.array(glm.transpose(frame).to_list(),dtype='float32')
                frame_list=self.convertHomogeneousToCSVROW(frame_numpy)

                data_list[17:-1]=frame_list
                bool_check=True
                #Write the row:
                with open(csv_name,'a',newline='') as file_object:
                    writer_object=csv.writer(file_object)
                    writer_object.writerow(data_list)
                    file_object.close()
        
        return bool_check

    def convertHomogeneousToCSVROW(self,transform):
        #Input: 4x4 numpy array for homogeneous transform
        #Output: 12x1 string list with: "Tx","Ty","Tz","R00","R01","R02","R10","R11","R12","R20","R21","R22"

        string_list=[transform[0,3],transform[1,3],transform[2,3],\
                    transform[0,0],transform[0,1],transform[0,2],\
                    transform[1,0],transform[1,1],transform[1,2],\
                    transform[2,0],transform[2,1],transform[2,2]]
        
        
        return string_list

    def on_mouse_motion_right(self,x,y,dx,dy):
        #print("Test")
        #print("Mouse Motion, x: "+str(x)+"y: "+str(y))
        self.camera_right.mouse_x=x
        self.camera_right.mouse_y=y
    
    def on_mouse_motion_left(self,x,y,dx,dy):
        #print("Left Mouse")
        #print("Mouse Motion, x: "+str(x)+"y: "+str(y))
        self.camera_left.mouse_x=x
        self.camera_left.mouse_y=y

    
    def get_program_background(self,shader_program_name):
        try:
            print("Shader Entered")
            with open(f'shaders/{shader_program_name}.vert') as file:
                vertex_shader_source = file.read()
                file.close()

            with open(f'shaders/{shader_program_name}.frag') as file:
                fragment_shader_source = file.read()
                file.close()

            return vertex_shader_source,fragment_shader_source

        except Exception as e:
            print(f"Error creating shader program '{shader_program_name}': {e}")
            return None


    def init_vertex_array(self, context: mgl.Context, program: mgl.Program) -> mgl.VertexArray:
        vertices_xy = self.get_vertices_for_quad_2d(size=(2.0, 2.0), bottom_left_corner=(-1.0, -1.0))
        vertex_buffer_xy = context.buffer(vertices_xy.tobytes())

        vertices_uv = self.get_vertices_for_quad_2d(size=(1.0, 1.0), bottom_left_corner=(0.0, 0.0))
        vertex_buffer_uv = context.buffer(vertices_uv.tobytes())

        vertex_array = context.vertex_array(program, [(vertex_buffer_xy, "2f", "vertex_xy"),
                                                      (vertex_buffer_uv, "2f", "vertex_uv")])
        return vertex_array


    def get_vertices_for_quad_2d(self, size=(2.0, 2.0), bottom_left_corner=(-1.0, -1.0)) -> np.array:
        # A quad is composed of 2 triangles: https://en.wikipedia.org/wiki/Polygon_mesh
        w, h = size
        x_bl, y_bl = bottom_left_corner
        vertices = np.array([x_bl,     y_bl + h,
                             x_bl,     y_bl,
                             x_bl + w, y_bl,

                             x_bl,     y_bl + h,
                             x_bl + w, y_bl,
                             x_bl + w, y_bl + h], dtype=np.float32)
        return vertices
    

    def cvFrame2Gl(self,frame):
        #print('frame conversion')
        #Flips the frame vertically
        frame_new=cv2.flip(frame,0)
        #Converts the frame to RGB
        frame_new=cv2.cvtColor(frame_new,cv2.COLOR_BGR2RGB)

        if frame_new.shape!=(ECM_FRAME_HEIGHT_DESIRED,ECM_FRAME_WIDTH_DESIRED,3):
            print("Entered Check")
            frame_new=cv2.resize(frame_new,(ECM_FRAME_WIDTH_DESIRED,ECM_FRAME_HEIGHT_DESIRED),interpolation=cv2.INTER_LINEAR)

        #Create frame of correct size:
        # frame_new=cv2.resize(frame_new,(ECM_FRAME_WIDTH_DESIRED,ECM_FRAME_HEIGHT_DESIRED),interpolation=cv2.INTER_LINEAR)

        # frame_sized=np.zeros((CONSOLE_VIEWPORT_HEIGHT,CONSOLE_VIEWPORT_WIDTH,3),np.uint8)
        # frame_sized[int(round((CONSOLE_VIEWPORT_HEIGHT-ECM_FRAME_HEIGHT_DESIRED)/2)):int(round((CONSOLE_VIEWPORT_HEIGHT-ECM_FRAME_HEIGHT_DESIRED)/2))+ECM_FRAME_HEIGHT_DESIRED,:]=frame_new



        return frame_new
    

    def run(self,frame_rate=100):
        pyglet.clock.schedule_interval(self.render,1/frame_rate)
        pyglet.app.run()


    def render(self,dt):
        self.delta_time=dt

        ################Render to left window###################
        self.window_left.switch_to()       #Switch to left window
        self.ctx_left.clear()              #Switch to left context

        if self.frame_left is not None:     #We have a left frame from the ECM
            frame_left=cv2.resize(self.frame_left,(ECM_FRAME_WIDTH_DESIRED,ECM_FRAME_HEIGHT_DESIRED),interpolation=cv2.INTER_LINEAR)
            self.ctx_left.disable(mgl.DEPTH_TEST)
            self.frame_left_converted=frame_left            
            
            if self.calibrate_on_left: #Calibration was pressed, and we either do multiple or direct calibration
                

                if self.is_CamToScene_Multiple: #We are doing calibration over multiple frames
                    self.frame_left_converted=self.aruco_tracker_left.arucoTrackingScene(frame_left)

                    if not self.aruco_tracker_left.calibrate_done:
                        self.lci_T_si=None
                        self.aruco_tracker_left.calibrateScene()
                        self.lci_T_si=self.aruco_tracker_left.ci_T_si                        
                        if self.lci_T_si is not None:
                            self.inv_lci_T_si=utils.invHomogeneousGLM(self.lci_T_si)
                            #Gets time delay
                            end_time=time.time()
                            time_delay=end_time-self.start_time_calib
                            #Calibration worked
                            #Draw on frame
                            cv2.drawFrameAxes(self.frame_left_converted,self.aruco_tracker_left.mtx,\
                                      self.aruco_tracker_left.dist,self.aruco_tracker_left.rvec_scene,self.aruco_tracker_left.tvec_scene,0.05)
                            
                            #Only run next bit if we are doing validation
                            if self.is_camToScene_started:
                                self.calibrate_on_left=False #Turns off grabbing current cam-to-scene

                                #Getting NDI
                                NDI_dat=self.ndi_tracker.get_frame()

                                #Writing data to csv
                                success=self.writeFramesToCSV(time_delay,NDI_dat,self.lci_T_si,self.csv_name_camtoscene_left,self.num_frames_captured)
                                if success:
                                    print("Grabbed Left Frame")
                                else:
                                    print("Left Frame Not Grabbed")
                if self.is_CamToScene_Single:
                    #Doing calibration directly on single frame
                    self.lci_T_si=None
                    start_time=time.time()
                    self.lci_T_si=self.aruco_tracker_left.calibrateSceneDirect(frame_left)
                    end_time=time.time()
                    time_delay=end_time-start_time
                    if self.lci_T_si is not None:
                        #Calibration worked
                        #Draw on frame
                        # cv2.drawFrameAxes(self.frame_left_converted,self.aruco_tracker_left.mtx,\
                        #             self.aruco_tracker_left.dist,self.aruco_tracker_left.rvec_scene,self.aruco_tracker_left.tvec_scene,0.05)
                        
                        #Only run next bit if we are doing validation
                        if self.is_camToScene_started:
                            self.calibrate_on_left=False #Turns off grabbing current cam-to-scene

                            #Getting NDI
                            NDI_dat=self.ndi_tracker.get_frame()

                            #Writing data to csv
                            success=self.writeFramesToCSV(time_delay,NDI_dat,self.lci_T_si,self.csv_name_camtoscene_left,self.num_frames_captured)
                            if success:
                                print("Grabbed Left Frame")
                            else:
                                print("Left Frame Not Grabbed")

                self.frame_left_converted=self.cvFrame2Gl(self.frame_left_converted)
            
            elif self.is_psmerror_started: #We are calibrating the PSM error
                self.frame_left_converted=frame_left

                if self.project_point_psm1_left is not None: 
                        #We are doing error correction and need to project the point to guide person
                    point_2d=tuple((int(self.project_point_psm1_left[0]),int(self.project_point_psm1_left[1])))
                    self.frame_left_converted=cv2.circle(self.frame_left_converted,point_2d,radius=6,color=(255,0,0),thickness=2)
                if self.project_point_psm3_left is not None: 
                    #We are doing error correction and need to project the point to guide person
                    point_2d=tuple((int(self.project_point_psm3_left[0]),int(self.project_point_psm3_left[1])))
                    self.frame_left_converted=cv2.circle(self.frame_left_converted,point_2d,radius=6,color=(0,255,0),thickness=2)
                
                self.frame_left_converted=self.cvFrame2Gl(self.frame_left_converted)



            elif self.is_PoseValidStarted: #We are showing/capturing the Pose Estimation
                self.frame_left_converted=frame_left
                
                #Getting cart_T_ecm
                try_true=False
                start_time=time.time()
                try:
                    cart_T_ecm=self.ecm.setpoint_cp()
                    try_true=True
                except Exception as e:
                    print("Unable to read ECM: "+str(e))
                    return
                if try_true:
                    cart_T_ecm=utils.enforceOrthogonalPyKDL(cart_T_ecm)
                    cart_T_ecm=utils.convertPyDK_To_GLM(cart_T_ecm)
                    ecmi_T_ecm=self.inv_cart_T_ecmi*cart_T_ecm

                    #Getting ecm_T_psm transform
                    ecm_T_psm=None
                    if self.PSM1_on: # We are validating PSM1
                        T_correct=self.inv_ecmac_T_ecmrep_psm1
                        try_true=False
                        try:
                            ecm_T_psm1=self.psm1.setpoint_cp() #Gets ecm_T_psm1 from API
                            try_true=True
                        except Exception as e:
                            print("Unable to read psm1: "+str(e))
                            return
                        if try_true:
                            ecm_T_psm1=utils.enforceOrthogonalPyKDL(ecm_T_psm1)
                            ecm_T_psm1=utils.convertPyDK_To_GLM(ecm_T_psm1)
                            ecm_T_psm=ecm_T_psm1

                    if self.PSM3_on: # We are validating PSM3
                        T_correct=self.inv_ecmac_T_ecmrep_psm3
                        try_true=False
                        try:
                            ecm_T_psm3=self.psm3.setpoint_cp() #Gets ecm_T_psm3 from API
                            try_true=True
                        except Exception as e:
                            print("Unable to read psm1: "+str(e))
                            return
                        if try_true:
                            ecm_T_psm3=utils.enforceOrthogonalPyKDL(ecm_T_psm3)
                            ecm_T_psm3=utils.convertPyDK_To_GLM(ecm_T_psm3)
                            ecm_T_psm=ecm_T_psm3
                    
                    
                    end_time=time.time()
                    delta_time=end_time-start_time
                    self.delta_Time_PoseValid+=delta_time

                    #Gets lc_T_psm for display
                    lc_T_psm=T_correct*self.inv_ecm_T_lc*ecm_T_psm

                    rvec,tvec=utils.convertHomoToRvecTvec_GLM(lc_T_psm)
                    cv2.drawFrameAxes(self.frame_left_converted,self.aruco_tracker_left.mtx,\
                                        self.aruco_tracker_left.dist,rvec,tvec,0.008,thickness=2)
                    
                    #Gets Aruco on PSM Tip
                    lc_T_aruco=self.aruco_tracker_left.calibrateSceneDirectValidation(frame_left)
                    if lc_T_aruco is not None:
                        rvec,tvec=utils.convertHomoToRvecTvec_GLM(lc_T_aruco)
                        cv2.drawFrameAxes(self.frame_left_converted,self.aruco_tracker_left.mtx,\
                                            self.aruco_tracker_left.dist,rvec,tvec,0.008,thickness=2)
                        
                        if self.is_GrabPose: #Need to grab the pose

                            ######Gets lc_T_psm
                            lc_T_psm_withErrCorr=lc_T_psm
                            lc_T_psm_withoutErrCorr=self.inv_ecm_T_lc*ecm_T_psm

                            ######Gets s_T_psm
                            #Kinematics Based
                            start_time=time.time()
                            s_T_psm_kinematics=self.inv_lci_T_si*T_correct*self.inv_ecm_T_lc*ecmi_T_ecm*ecm_T_psm
                            end_time=time.time()
                            delta_time=end_time-start_time
                            self.delta_Time_PoseValid+=delta_time

                            #Vision Based
                            s_T_psm_vision=self.inv_lc_T_s*T_correct*self.inv_ecm_T_lc*ecm_T_psm

                            #Getting NDI
                            NDI_dat=self.ndi_tracker.get_frame()

                            #Writing data to csv
                            success=self.writeFramesToCSV(self.delta_Time_PoseValid,NDI_dat,s_T_psm_kinematics,self.csv_name_s_T_PSM_kinematics_withErrCorr,self.num_frames_captured)
                            success=(self.writeFramesToCSV(self.delta_Time_PoseValid,NDI_dat,s_T_psm_vision,self.csv_name_s_T_PSM_visual_withErrCorr,self.num_frames_captured)) and success
                            
                            success=(self.writeFramesToCSV_Aruco(self.delta_Time_PoseValid,lc_T_aruco,lc_T_psm_withErrCorr,self.csv_name_lc_T_psm_withErrCorr,self.num_frames_captured)) and success
                            success=(self.writeFramesToCSV_Aruco(self.delta_Time_PoseValid,lc_T_aruco,lc_T_psm_withoutErrCorr,self.csv_name_lc_T_psm_withoutErrCorr,self.num_frames_captured)) and success
                            if success:
                                print("Grabbed Left Frame")
                            else:
                                self.num_frames_captured-=1
                                print("Left Frame Not Grabbed")

                            self.is_GrabPose=False

                    self.delta_Time_PoseValid=0


                
                self.frame_left_converted=self.cvFrame2Gl(self.frame_left_converted)      


            else:
                self.frame_left_converted=self.cvFrame2Gl(frame_left)
            


            self.texture_left.write(self.frame_left_converted)
            self.texture_left.use()
            self.vertex_array_left.render()
            self.ctx_left.enable(mgl.DEPTH_TEST)

        
        ################Render to right window###################
        self.window_right.switch_to()       #Switch to right window
        self.ctx_right.clear()              #Switch to right context

        if self.frame_right is not None:     #We have a right frame from the ECM
            frame_right=cv2.resize(self.frame_right,(ECM_FRAME_WIDTH_DESIRED,ECM_FRAME_HEIGHT_DESIRED),interpolation=cv2.INTER_LINEAR)
            self.ctx_right.disable(mgl.DEPTH_TEST)

            if self.calibrate_on_right: #Calibration was pressed, and we either do multiple or direct calibration
                self.frame_right_converted=frame_right

                if self.is_CamToScene_Multiple: #We are doing calibration over multiple frames
                    self.frame_right_converted=self.aruco_tracker_right.arucoTrackingScene(frame_right)

                    if not self.aruco_tracker_right.calibrate_done:
                        self.rci_T_si=None
                        self.aruco_tracker_right.calibrateScene()
                        self.rci_T_si=self.aruco_tracker_right.ci_T_si
                        if self.rci_T_si is not None:
                            #Gets time delay
                            end_time=time.time()
                            time_delay=end_time-self.start_time_calib
                            #Calibration worked
                            #Draw on frame
                            cv2.drawFrameAxes(self.frame_right_converted,self.aruco_tracker_right.mtx,\
                                      self.aruco_tracker_right.dist,self.aruco_tracker_right.rvec_scene,self.aruco_tracker_right.tvec_scene,0.05)
                            
                            #Only run next bit if we are doing validation
                            if self.is_camToScene_started:
                                self.calibrate_on_right=False #Turns off grabbing current cam-to-scene

                                #Getting NDI
                                NDI_dat=self.ndi_tracker.get_frame()

                                #Writing data to csv
                                success=self.writeFramesToCSV(time_delay,NDI_dat,self.rci_T_si,self.csv_name_camtoscene_right,self.num_frames_captured)
                                if success:
                                    print("Grabbed Right Frame")
                                else:
                                    print("Right Frame Not Grabbed")
                if self.is_CamToScene_Single:
                    #Doing calibration directly on single frame
                    self.rci_T_si=None
                    start_time=time.time()
                    self.rci_T_si=self.aruco_tracker_right.calibrateSceneDirect(frame_right)
                    end_time=time.time()
                    time_delay=end_time-start_time
                    if self.rci_T_si is not None:
                        #Calibration worked
                        #Draw on frame
                        # cv2.drawFrameAxes(self.frame_right_converted,self.aruco_tracker_right.mtx,\
                        #               self.aruco_tracker_right.dist,self.aruco_tracker_right.rvec_scene,self.aruco_tracker_right.tvec_scene,0.05)
                        
                        #Only run next bit if we are doing validation
                        if self.is_camToScene_started:
                            self.calibrate_on_right=False #Turns off grabbing current cam-to-scene

                            #Getting NDI
                            NDI_dat=self.ndi_tracker.get_frame()

                            #Writing data to csv
                            success=self.writeFramesToCSV(time_delay,NDI_dat,self.rci_T_si,self.csv_name_camtoscene_right,self.num_frames_captured)
                            if success:
                                print("Grabbed Right Frame")
                            else:
                                self.num_frames_captured-=1
                                print("Right Frame Not Grabbed")

                self.frame_right_converted=self.cvFrame2Gl(self.frame_right_converted)
            elif self.is_psmerror_started: #We are calibrating the PSM error
                self.frame_right_converted=frame_right

                if self.project_point_psm1_right is not None: 
                        #We are doing error correction and need to project the point to guide person
                    point_2d=tuple((int(self.project_point_psm1_right[0]),int(self.project_point_psm1_right[1])))
                    self.frame_right_converted=cv2.circle(self.frame_right_converted,point_2d,radius=6,color=(255,0,0),thickness=2)
                if self.project_point_psm3_right is not None: 
                    #We are doing error correction and need to project the point to guide person
                    point_2d=tuple((int(self.project_point_psm3_right[0]),int(self.project_point_psm3_right[1])))
                    self.frame_right_converted=cv2.circle(self.frame_right_converted,point_2d,radius=6,color=(0,255,0),thickness=2)
                
                self.frame_right_converted=self.cvFrame2Gl(self.frame_right_converted)
            
            else:
                self.frame_right_converted=self.cvFrame2Gl(frame_right)
            

            self.texture_right.write(self.frame_right_converted)
            self.texture_right.use()
            self.vertex_array_right.render()
            self.ctx_right.enable(mgl.DEPTH_TEST)

        
        #Makes sure the PSM's jaw is closed
        if self.PSM1_on:
            # jawAng=self.psm1.jaw.measured_js()
            # jawAng[0]=-20.0
            self.psm1.jaw.move_jp(np.array([-20.0]))
        if self.PSM3_on:
            self.psm3.jaw.move_jp(np.array([-20.0]))



        ########Update GUI
        self.gui_window.update()
        







if __name__ == '__main__':
    
    #Initializing rospy

    rospy.init_node('ExpertPlaybackValidation')
    rospy.Rate(10000)

    #Initialize the Renderer Class for both right and left eyes
    tool_renderer=Renderer()

    rospy.Subscriber(name = RightFrame_Topic, data_class=CompressedImage, callback=tool_renderer.frameCallbackRight,queue_size=1,buff_size=2**18)
    rospy.Subscriber(name = LeftFrame_Topic, data_class=CompressedImage, callback=tool_renderer.frameCallbackLeft,queue_size=1,buff_size=2**18)
    rospy.Subscriber(name = lc_T_s_Topic,data_class=Float32MultiArray, callback=tool_renderer.lcTs_Callback,queue_size=1,buff_size=2**10)
    rospy.Subscriber(name = rc_T_s_Topic,data_class=Float32MultiArray, callback=tool_renderer.rcTs_Callback,queue_size=1,buff_size=2**10)

    tool_renderer.run(frame_rate=1000)