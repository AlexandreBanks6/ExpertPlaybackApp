import rospy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String 
from std_msgs.msg import Int32
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Joy


import moderngl as mgl
import sys

import skimage.transform

from include import Model as mdl
from include import Camera
from include import Light
from include import mesh
from include import scene
from include import ArucoTracker
from include import HandEye
from include import utils
from include import DataLogger
import glm
import pyglet


from pyglet.gl import Config, Context
from pyglet.window import key
from PIL import Image

import numpy as np
import os

#TKinter importer
import tkinter as tk

#CSV Stuff
from datetime import datetime
import csv

#dvrk tool stuff
import dvrk
import PyKDL

import skimage
import tf_conversions.posemath as pm
import tf_conversions
import yaml
import pandas as pd


#################File Names#####################
MOTIONS_ROOT='../resources/Motions/'


##################ROS Topics####################
#Image Topic Subscriptions
RightFrame_Topic='ubc_dVRK_ECM/right/decklink/camera/image_raw/compressed'
LeftFrame_Topic='ubc_dVRK_ECM/left/decklink/camera/image_raw/compressed'

#PC 2 Time Subscription
PC2_Time_Topic='ExpertPlayback/pc2Time' #Published from PC2 (subscribed to by PC1)
filecount_Topic='ExpertPlayback/fileCount' #Published from PC1 (subscribed to by PC2)
lc_T_s_Topic='ExpertPlayback/lc_T_s' #Published from PC2 (subscribed to by PC1)
rc_T_s_Topic='ExpertPlayback/rc_T_s' #Published from PC2 (subscribed to by PC1)


##################Constants#####################
#Tool and Console Constants
CONSOLE_VIEWPORT_WIDTH=1400
CONSOLE_VIEWPORT_HEIGHT=986

tool_tip_offset=0.0102  #PSM1 tooltip offset (large needle driver)
tool_tip_offset_psm3=0.0102 #PSM3 tooltip offset

tool_tip_point=np.array([0,0,tool_tip_offset+0.001,1],dtype=np.float32) #Point of tooltip for API correction


METERS_TO_RENDER_SCALE=1000 #OpenGL in mm, real word in m

obj_names=['shaft','body','jaw_right','jaw_left']

#########Constant Transforms
mul_mat=glm.mat4x3()  #Selection matrix to take [Xc,Yc,Zc]
input_pose=glm.mat4()

#Converts opencv camera to opengl
opengl_T_opencv=glm.mat4(glm.vec4(1,0,0,0),
                     glm.vec4(0,-1,0,0),
                     glm.vec4(0,0,-1,0),
                     glm.vec4(0,0,0,1))


#DH parameters for 4_C_5 (placing shaft)
T_5_alpha=glm.pi()/2
T_5_cos_alpha=glm.cos(T_5_alpha)
T_5_sin_alpha=glm.sin(T_5_alpha)

#DH parameters for 5_C_6 (placing body)
T_6_alpha=glm.pi()/2
T_6_cos_alpha=glm.cos(T_6_alpha)
T_6_sin_alpha=glm.sin(T_6_alpha)

T_6_a=0.0091 #Body length

#Goes from API reported last joint coord, to our kinematics coord system
T_7_psm=glm.mat4(glm.vec4(0,0,-1,0),
                     glm.vec4(0,1,0,0),
                     glm.vec4(1,0,0,0),
                     glm.vec4(0,0,0,1))


psm_T_7=utils.invHomogeneousGLM(T_7_psm)


#Transforms to convert between the standard DH frames and the frame system of the 3D models
jl_T_jl_local=glm.mat4(glm.vec4(1,0,0,0),
               glm.vec4(0,0,-1,0),
               glm.vec4(0,1,0,0),
               glm.vec4(0,0,0,1))  #Transform between Cjl and Cl (left jaw frame)

jr_T_jr_local=glm.mat4(glm.vec4(1,0,0,0),
               glm.vec4(0,0,-1,0),
               glm.vec4(0,1,0,0),
               glm.vec4(0,0,0,1))  #Transform between Cjr and Cr (right jaw frame)



T_6shift_b=glm.mat4(glm.vec4(1,0,0,0),
               glm.vec4(0,0,-1,0),
               glm.vec4(0,1,0,0),
               glm.vec4(0,0,0,1))  #Transform between C6 and Cb (body frame)

T_5_s=glm.mat4(glm.vec4(1,0,0,0),
               glm.vec4(0,0,1,0),
               glm.vec4(0,-1,0,0),
               glm.vec4(0,0,0,1))  #Transform between C5 and Cs (shaft frame)


#Just to start the instrument pose
start_pose=glm.mat4(glm.vec4(1,0,0,0),
                glm.vec4(0,1,0,0),
                glm.vec4(0,0,1,0),
                glm.vec4(0,0,-30,1))


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
        self.aruco_tracker_right=ArucoTracker.ArucoTracker(self,'right')

        #Data logger object
        self.dataLogger_pc1=DataLogger.DataLogger(self)

        #Object to publish the file count to syncrhonize the file numbers between PC1 and PC2
        self.filecount_pub=rospy.Publisher(name = filecount_Topic,data_class=Int32,queue_size=10)
        self.filecount_pub.publish(self.dataLogger_pc1.file_count)

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


        #Initialize the instrument frames
        self.instrument_dict_PSM1=None
        self.instrument_dict_PSM3=None
        self.instrument_kinematics([0,0,0,0],start_pose,'PSM1')
        self.instrument_kinematics([0,0,0,0],start_pose,'PSM3')


        ##############GUI SETUP################
        self.gui_window=tk.Tk()
        self.gui_window.title("dVRK Playback App PC1")

        self.gui_window.rowconfigure([0,1,2,3,4],weight=1)
        self.gui_window.columnconfigure([0,1,2,3],weight=1)
        self.gui_window.minsize(300,100)

        #Title at top
        self.welcome_text=tk.Label(self.gui_window,text="Welcome to the dVRK Playback App PC1")
        self.welcome_text.grid(row=0,column=1,sticky='n')

        #Show ArUco Button
        self.aruco_button=tk.Button(self.gui_window,text="Show ArUco",command=self.arucoToggleCallback)
        self.aruco_button.grid(row=1,column=0,sticky="nsew")
        #Button to calibrate the scene (register camera to rigid object)
        self.calibrate_scene_button=tk.Button(self.gui_window,text="Calibrate Scene",command=self.calibrateToggleCallback)
        self.calibrate_scene_button.grid(row=1,column=1,sticky="nsew")       


        #Buttons to calibrate PSM API error
        self.start_psmerror_calibration=tk.Button(self.gui_window,text="Start PSM Error Calib",command=self.startPSMErrorCalib)
        self.start_psmerror_calibration.grid(row=2,column=0,sticky="nsew")

        self.grab_psm1_pointbutton=tk.Button(self.gui_window,text="Grab PSM1 Point",command=self.grabPSM1PointCallback)
        self.grab_psm1_pointbutton.grid(row=2,column=1,sticky="nsew")

        self.grab_psm3_pointbutton=tk.Button(self.gui_window,text="Grab PSM3 Point",command=self.grabPSM3PointCallback)
        self.grab_psm3_pointbutton.grid(row=2,column=2,sticky="nsew")

        self.calibrate_psmerror_button=tk.Button(self.gui_window,text="Calibrate PSM Error",command=self.calibratePSMErrorCallback)
        self.calibrate_psmerror_button.grid(row=2,column=3,sticky="nsew")

        #Select PSM (to record/playback/overlay) Checkboxes
        self.checkbox_PSM1=tk.Checkbutton(self.gui_window,text="PSM1",onvalue=1,offvalue=0,command=self.psm1Checkbox)
        self.checkbox_PSM1.grid(row=3,column=0,sticky='nsew')

        self.checkbox_PSM3=tk.Checkbutton(self.gui_window,text="PSM3",onvalue=1,offvalue=0,command=self.psm3Checkbox)
        self.checkbox_PSM3.grid(row=3,column=1,sticky='nsew')
        
        ############Buttons for validating PSM1/PSM3, virtual overlay, recording motions, and playing back motions##########

        self.validate_error_correction_button=tk.Button(self.gui_window,text="Validate API Err Correction",command=self.validateErrorCorrectionCallback)
        self.validate_error_correction_button.grid(row=3,column=2,sticky="nsew")

        #Virtual Overlay Button
        self.virtual_overlay_button=tk.Button(self.gui_window,text="Virtual Overlay",command=self.virtualOverlayCallback)
        self.virtual_overlay_button.grid(row=4,column=0,sticky="nsew")
        
        #Record Expert Motions Button
        self.record_motions_button=tk.Button(self.gui_window,text="Record Motions (Start on PC1 before PC2)",command=self.rocordMotionsCallback)
        self.record_motions_button.grid(row=4,column=1,sticky="nsew")

        #Playback Tools Button
        self.calibrate_gaze_button=tk.Button(self.gui_window,text="Calibrate Gaze (Record Motions is Pressed First)",command=self.calibrateGazeCallback)
        self.calibrate_gaze_button.grid(row=4,column=2,sticky="nsew")

        #Playback Tools Button
        self.render_button=tk.Button(self.gui_window,text="Playback Tools",command=self.playbackMotionsCallback)
        self.render_button.grid(row=4,column=3,sticky="nsew")




        ####################Initializing Variables##################

        


        ##########Booleans and Counters
        self.aruco_on=False
        self.calibrate_on=False


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

        self.is_new_psm1_points=False
        self.is_new_psm3_points=False

        #Selecting which PSM to render
        self.PSM1_on=False
        self.PSM3_on=False

        #Validate err correction
        self.validate_error_correction_on=False

        #Virtual Overlay boolean
        self.virtual_overlay_on=False

        #Record motions
        self.record_motions_on=False

        #Calibrate Gaze Boolean
        self.is_gaze_calib=False

        #Playback motions
        self.playback_on=False

        ##Time Variables
        self.delta_time=None
        self.record_time=0 #For recording
        self.render_time=0 #For playback (playback is a continuous loop)
        self.pc2_time=None

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

        #Camera to scene registration
        self.lci_T_si=None
        self.rci_T_si=None

        self.inv_lci_T_si=None
        self.inv_rci_T_si=None

        #Initial ECM Pose
        self.cart_T_ecmi=None
        self.inv_cart_T_ecmi=None


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

        #PSM1 Error Correction
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

        #PSM3 Error Correction
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



    ######################Callbacks####################

    def arucoToggleCallback(self):
        self.aruco_on=not self.aruco_on

    def calibrateToggleCallback(self):
        self.calibrate_on=not self.calibrate_on
        if self.calibrate_on:
            self.aruco_tracker_left.calibrate_done=False
            self.aruco_tracker_left.corner_scene_list=[]
            self.aruco_tracker_left.ids_scene_list=[]

            self.aruco_tracker_right.calibrate_done=False
            self.aruco_tracker_right.corner_scene_list=[]
            self.aruco_tracker_right.ids_scene_list=[]

    def validateErrorCorrectionCallback(self):
        self.validate_error_correction_on=not self.validate_error_correction_on

    def psm1Checkbox(self):
        self.PSM1_on=not self.PSM1_on

    def psm3Checkbox(self):
        self.PSM3_on=not self.PSM3_on 

    def virtualOverlayCallback(self):
        self.virtual_overlay_on= not self.virtual_overlay_on

    def rocordMotionsCallback(self):
        self.record_motions_on=not self.record_motions_on
        #sets record time to zero:
        self.record_time=0
        self.pc2_time=None
        #Initializes the csv file
        self.dataLogger_pc1.initRecording_PC1(MOTIONS_ROOT)
        self.filecount_pub.publish(self.dataLogger_pc1.file_count)

    
    def calibrateGazeCallback(self):
        self.is_gaze_calib=not self.is_gaze_calib


    def playbackMotionsCallback(self):
        self.playback_on=not self.playback_on

    def frameCallbackRight(self,data):
        self.frame_right=self.bridge.compressed_imgmsg_to_cv2(data,'passthrough')

    def frameCallbackLeft(self,data):        
        self.frame_left=self.bridge.compressed_imgmsg_to_cv2(data,'passthrough')

    def pc2TimeCallback(self,data):
        self.pc2_time=data.data
    

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

            #############Gets actual point location
            point_si=self.model_scene_points[self.psm1_points_count]
            point_si_list=point_si.tolist()        
            point_si_list.append(1)
            point_si=glm.vec4(point_si_list)

            # point_lc_ac=self.lci_T_si*point_si
            point_ecm_ac=self.ecm_T_lc*self.lci_T_si*point_si

            point_ecm_ac_list=[point_ecm_ac[0],point_ecm_ac[1],point_ecm_ac[2]]

            point_ecm_ac=np.array(point_ecm_ac_list,dtype=np.float32)

            if self.p_ecm_ac_list_psm1 is None:
                self.p_ecm_ac_list_psm1=point_ecm_ac
            else:
                self.p_ecm_ac_list_psm1=np.vstack((self.p_ecm_ac_list_psm1,point_ecm_ac))

            

            #############Gets Reported Point for left camera
            #rospy.sleep(0.5)
            ecm_T_psm_rep=self.psm1.measured_cp()
            ecm_T_psm_rep=utils.enforceOrthogonalPyKDL(ecm_T_psm_rep)
            ecm_T_psm_rep=pm.toMatrix(ecm_T_psm_rep) #Numpy array

            

            point_ecm_rep=ecm_T_psm_rep@tool_tip_point
            point_ecm_rep=point_ecm_rep[0:3]

            if self.p_ecm_rep_list_psm1 is None:
                self.p_ecm_rep_list_psm1=point_ecm_rep
            else:
                self.p_ecm_rep_list_psm1=np.vstack((self.p_ecm_rep_list_psm1,point_ecm_rep))

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

            # point_lc_ac=self.lci_T_si*point_si
            point_ecm_ac=self.ecm_T_lc*self.lci_T_si*point_si

            point_ecm_ac_list=[point_ecm_ac[0],point_ecm_ac[1],point_ecm_ac[2]]

            point_ecm_ac=np.array(point_ecm_ac_list,dtype=np.float32)

            if self.p_ecm_ac_list_psm3 is None:
                self.p_ecm_ac_list_psm3=point_ecm_ac
            else:
                self.p_ecm_ac_list_psm3=np.vstack((self.p_ecm_ac_list_psm3,point_ecm_ac))

            

            #############Gets Reported Point for left camera
            #rospy.sleep(0.5)
            ecm_T_psm_rep=self.psm3.measured_cp()
            ecm_T_psm_rep=utils.enforceOrthogonalPyKDL(ecm_T_psm_rep)
            ecm_T_psm_rep=pm.toMatrix(ecm_T_psm_rep) #Numpy array


            # point_lc_rep=utils.invHomogeneousNumpy(self.ecm_T_lc_np)@ecm_T_psm_rep@tool_tip_point
            point_ecm_rep=ecm_T_psm_rep@tool_tip_point
            point_ecm_rep=point_ecm_rep[0:3]

            if self.p_ecm_rep_list_psm3 is None:
                self.p_ecm_rep_list_psm3=point_ecm_rep
            else:
                self.p_ecm_rep_list_psm3=np.vstack((self.p_ecm_rep_list_psm3,point_ecm_rep))

            print("point_lc_ac: "+str(point_ecm_rep))
            print("point_lc_rep: "+str(point_ecm_rep))


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

        #PSM1:
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
                    est_point=est_point[0:3]


                    trans_diff=est_point-self.p_ecm_ac_list_psm1[i]
                    trans_diff=np.linalg.norm(trans_diff)
                    translation_diff_list.append(trans_diff)
                translation_diff_list=np.array(translation_diff_list,dtype=np.float32)
                trans_diff=np.mean(translation_diff_list)
                print('registration error psm1 lc: '+str(trans_diff))

                self.is_psmerror_calib=True


        #PSM3:
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
                    est_point=est_point[0:3]


                    trans_diff=est_point-self.p_ecm_ac_list_psm3[i]
                    trans_diff=np.linalg.norm(trans_diff)
                    translation_diff_list.append(trans_diff)
                translation_diff_list=np.array(translation_diff_list,dtype=np.float32)
                trans_diff=np.mean(translation_diff_list)
                print('registration error psm3 lc: '+str(trans_diff))
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
        return frame_new
    

    #############Instrument Kinematics Methods############
    def instrument_kinematics(self,joint_angles,w_T_psm,PSM_Type):
        '''
        Input: 
        - list of angles of the intrument joints: body rotation (q6), jaw rotation (q7), jaw seperation (theta_j)
        - Tool first joint with respect to world (w_T_psm)
        Output: 
        - Frames for each instrument segment: shaft frame (Ts), body frame (Tb), left jaw (Tl), right jaw (Tr)  

        Also note, T4 is the base of the instrument, and T5 is the shaft rotated frame, T6 is the body rotated frame, T7 is the jaw base (no seperation)
        '''
        #Enforce >=0 jaw angle
        if joint_angles[2]<0:
            joint_angles[2]=0
        
        
        #Last joint (in our cood system)
        w_T_7=w_T_psm*psm_T_7

        #Left jaw
        w_T_jl_local=w_T_7*self.Rotz(-joint_angles[2]/2)*jl_T_jl_local    #Finds world to jaw left in object coords (local)
        w_T_jl_local=utils.scaleGLMTranform(w_T_jl_local,METERS_TO_RENDER_SCALE)

        #Right jaw
        w_T_jr_local=w_T_7*self.Rotz(joint_angles[2]/2)*jr_T_jr_local   #Finds world to right jaw in object coords
        w_T_jr_local=utils.scaleGLMTranform(w_T_jr_local,METERS_TO_RENDER_SCALE)


        #Body
        w_T_6=w_T_7*utils.invHomogeneousGLM(self.transform_6_T_7(joint_angles[1]))       
        w_T_bodylocal=glm.translate(w_T_6,glm.vec3(-T_6_a,0,0))*T_6shift_b
        w_T_bodylocal=utils.scaleGLMTranform(w_T_bodylocal,METERS_TO_RENDER_SCALE)

        #Shaft
        w_T_shaftlocal=w_T_7*utils.invHomogeneousGLM(self.transform_6_T_7(joint_angles[1]))*utils.invHomogeneousGLM(self.transform_5_T_6(-joint_angles[0]))*T_5_s
        w_T_shaftlocal=utils.scaleGLMTranform(w_T_shaftlocal,METERS_TO_RENDER_SCALE)

        


        if PSM_Type=='PSM1':
            self.instrument_dict_PSM1={
                'shaft': w_T_shaftlocal,
                'body': w_T_bodylocal,
                'jaw_right': w_T_jr_local,
                'jaw_left': w_T_jl_local
            }

        elif PSM_Type=='PSM3':
            self.instrument_dict_PSM3={
                'shaft': w_T_shaftlocal,
                'body': w_T_bodylocal,
                'jaw_right': w_T_jr_local,
                'jaw_left': w_T_jl_local
            }
    
    def transform_4_T_5(self,q5):
        #Homogeneous transform between the instrument base (C4) and shaft frame (C5) to place the shaft
        theta_5=q5

        trans_4_C_5=glm.mat4(glm.vec4(glm.cos(theta_5),glm.sin(theta_5),0,0),
                             glm.vec4(-glm.sin(theta_5)*T_5_cos_alpha,glm.cos(theta_5)*T_5_cos_alpha,T_5_sin_alpha,0),
                             glm.vec4(glm.sin(theta_5)*T_5_sin_alpha,-glm.cos(theta_5)*T_5_sin_alpha,T_5_cos_alpha,0),
                             glm.vec4(0,0,0,1))
        return trans_4_C_5
    

    def transform_5_T_6(self,q6):
        #Gets the homogenous transform between shaft frame (C5) and body frame (C6)
        theta_6=q6+(glm.pi()/2)
        trans_5_C_6=glm.mat4(glm.vec4(glm.cos(theta_6),glm.sin(theta_6),0,0),
                             glm.vec4(-glm.sin(theta_6)*T_6_cos_alpha,glm.cos(theta_6)*T_6_cos_alpha,T_6_sin_alpha,0),
                             glm.vec4(glm.sin(theta_6)*T_6_sin_alpha,-glm.cos(theta_6)*T_6_sin_alpha,T_6_cos_alpha,0),
                             glm.vec4(T_6_a*glm.cos(theta_6),T_6_a*glm.sin(theta_6),0,1))
        return trans_5_C_6
    
    def transform_6_T_7(self,q7):
        #Gets the homogenous transform between body frame (C6) and the jaw base (C7) => Jaw base has no rotation
        theta_7=q7
        trans_6_C_7=glm.mat4(glm.vec4(glm.cos(theta_7),glm.sin(theta_7),0,0),
                             glm.vec4(-glm.sin(theta_7),glm.cos(theta_7),0,0),
                             glm.vec4(0,0,1,0),
                             glm.vec4(0,0,0,1))
        return trans_6_C_7

    
    
    def Rotz(self,theta):
        #Rotation about z-axis of the base jaw frame to give the jaw seperation
        rot_mat=glm.mat4(glm.vec4(glm.cos(theta),glm.sin(theta),0,0),
                         glm.vec4(-glm.sin(theta),glm.cos(theta),0,0),
                         glm.vec4(0,0,1,0),
                         glm.vec4(0,0,0,1))
        return rot_mat    
    
    def move_objects(self):
        #This is the method where we pass the matrix to move_obj to move the object and also 
        #select which object to move: "shaft", "body","jaw_right","jaw_left"
        if self.PSM1_on:
            for obj_name in obj_names:
                #if obj_name=='jaw_right' or obj_name=='jaw_left':
                #   continue
                #print(obj_name)
                move_mat=self.instrument_dict_PSM1[obj_name]
                self.scene_PSM1.move_obj(obj_name,move_mat)

        if self.PSM3_on:
            for obj_name in obj_names:
                #if obj_name=='jaw_right' or obj_name=='jaw_left':
                #   continue
                #print(obj_name)
                move_mat=self.instrument_dict_PSM3[obj_name]
                self.scene_PSM3.move_obj(obj_name,move_mat)
    



    #######################Run Methods#####################

    def run(self,frame_rate=100):
        pyglet.clock.schedule_interval(self.render,1/frame_rate)
        pyglet.app.run()

    
    def render(self,dt):
        self.delta_time=dt
        print("delta_time"+str(dt))

        ############Real-Time Virtual Overlay##############
        #Get PSM Poses for virtual overlay and run instrument kinematics
        if self.virtual_overlay_on and self.aruco_tracker_left.calibrate_done and self.is_psmerror_calib:
            cart_T_ecm=self.ecm.measured_cp()
            cart_T_ecm=utils.enforceOrthogonalPyKDL(cart_T_ecm)
            cart_T_ecm=utils.convertPyDK_To_GLM(cart_T_ecm)
            ecmi_T_ecm=self.inv_cart_T_ecmi*cart_T_ecm
            #print("ecmi_T_ecm: "+str(ecmi_T_ecm))


            if self.PSM1_on:
                ecm_T_psm1=self.psm1.measured_cp() #Gets ecm_T_psm1 from API
                ecm_T_psm1=utils.enforceOrthogonalPyKDL(ecm_T_psm1)
                ecm_T_psm1=utils.convertPyDK_To_GLM(ecm_T_psm1)

                joint_vars_psm1=self.psm1.measured_js()[0]
                jaw_angle_psm1=self.psm1.jaw.measured_js()[0]
                joint_vars_psm1_new=[joint_vars_psm1[4],joint_vars_psm1[5],jaw_angle_psm1[0]]
                s_T_psm1=self.inv_lci_T_si*self.inv_ecm_T_lc*self.inv_ecmac_T_ecmrep_psm1*ecm_T_psm1

                self.instrument_kinematics(joint_vars_psm1_new,s_T_psm1,'PSM1')
            
            if self.PSM3_on:
                ecm_T_psm3=self.psm3.measured_cp() #Gets ecm_T_psm3 from API
                ecm_T_psm3=utils.enforceOrthogonalPyKDL(ecm_T_psm3)
                ecm_T_psm3=utils.convertPyDK_To_GLM(ecm_T_psm3)

                joint_vars_psm3=self.psm3.measured_js()[0]
                jaw_angle_psm3=self.psm3.jaw.measured_js()[0]
                joint_vars_psm3_new=[joint_vars_psm3[4],joint_vars_psm3[5],jaw_angle_psm3[0]]
                s_T_psm3=self.inv_lci_T_si*self.inv_ecm_T_lc*self.inv_ecmac_T_ecmrep_psm3*ecm_T_psm3

                self.instrument_kinematics(joint_vars_psm3_new,s_T_psm3,'PSM3')
            
            self.move_objects()

        ################Record Motions & Data###################
        if self.record_motions_on and self.aruco_tracker_left.calibrate_done and self.is_psmerror_calib:
            ####Time and Indexes to record
            #Task Time
            self.record_time+=self.delta_time

            #PC1 Time
            pc1_time=datetime.now().time()

            ####Transforms to Record
            if self.virtual_overlay_on: #Already computed some transforms transforms
                ecm_joints=self.ecm.measured_js()[0]

                if self.PSM1_on:
                    psm1_joints=joint_vars_psm1+jaw_angle_psm1
                else:
                    ecm_T_psm1=None
                    psm1_joints=None
                    s_T_psm1=None

                if self.PSM3_on:
                    psm3_joints=joint_vars_psm3+jaw_angle_psm3
                else:
                    ecm_T_psm3=None
                    psm3_joints=None
                    s_T_psm3=None
                
            else: #Did not computed needed transforms above in virtual overlay
                cart_T_ecm=self.ecm.measured_cp()
                cart_T_ecm=utils.enforceOrthogonalPyKDL(cart_T_ecm)
                cart_T_ecm=utils.convertPyDK_To_GLM(cart_T_ecm)

                ecm_joints=self.ecm.measured_js()[0]

                if self.PSM1_on:
                    ecm_T_psm1=self.psm1.measured_cp() #Gets ecm_T_psm1 from API
                    ecm_T_psm1=utils.enforceOrthogonalPyKDL(ecm_T_psm1)
                    ecm_T_psm1=utils.convertPyDK_To_GLM(ecm_T_psm1)
                    joint_vars_psm1=self.psm1.measured_js()[0]
                    jaw_angle_psm1=self.psm1.jaw.measured_js()[0]
                    psm1_joints=joint_vars_psm1+jaw_angle_psm1
                    s_T_psm1=self.inv_lci_T_si*self.inv_ecm_T_lc*self.inv_ecmac_T_ecmrep_psm1*ecm_T_psm1
                else:
                    ecm_T_psm1=None
                    psm1_joints=None
                    s_T_psm1=None


                if self.PSM3_on:
                    ecm_T_psm3=self.psm3.measured_cp() #Gets ecm_T_psm1 from API
                    ecm_T_psm3=utils.enforceOrthogonalPyKDL(ecm_T_psm3)
                    ecm_T_psm3=utils.convertPyDK_To_GLM(ecm_T_psm3)
                    joint_vars_psm3=self.psm3.measured_js()[0]
                    jaw_angle_psm3=self.psm3.jaw.measured_js()[0]
                    psm3_joints=joint_vars_psm3+jaw_angle_psm3
                    s_T_psm3=self.inv_lci_T_si*self.inv_ecm_T_lc*self.inv_ecmac_T_ecmrep_psm3*ecm_T_psm3
                else:
                    ecm_T_psm3=None
                    psm3_joints=None
                    s_T_psm3=None
            
            #Writing the row to the csv file
            self.dataLogger_pc1.writeRow_PC1(self.record_time,pc1_time,self.pc2_time,int(self.is_gaze_calib==True),s_T_psm1,s_T_psm3,\
                                             cart_T_ecm,ecm_T_psm1,ecm_T_psm3,psm1_joints,psm3_joints,ecm_joints)
                


        ################Render to left window###################
        self.window_left.switch_to()       #Switch to left window
        self.ctx_left.clear()              #Switch to left context

        if self.frame_left is not None:     #We have a left frame from the ECM
            self.ctx_left.disable(mgl.DEPTH_TEST)

            if self.aruco_on:   #User button press, we show aruco and maybe calibrate the scene
                self.frame_left_converted=self.aruco_tracker_left.arucoTrackingScene(self.frame_left) #Show ArUco

                if self.calibrate_on and (not self.aruco_tracker_left.calibrate_done): #Sets Scene Base Frame 
                    self.lci_T_si=None
                    self.aruco_tracker_left.calibrateScene()
                    self.lci_T_si=self.aruco_tracker_left.ci_T_si

                if self.lci_T_si is not None:   #Calibration worked
                    self.inv_lci_T_si=utils.invHomogeneousGLM(self.lci_T_si)
                    cv2.drawFrameAxes(self.frame_left_converted,self.aruco_tracker_left.mtx,\
                                      self.aruco_tracker_left.dist,self.aruco_tracker_left.rvec_scene,self.aruco_tracker_left.tvec_scene,0.05)
                
                self.frame_left_converted=self.cvFrame2Gl(self.frame_left_converted)

            elif self.is_psmerror_started: #We are calibrating the PSM error
                self.frame_left_converted=self.frame_left

                if self.project_point_psm1_left is not None: 
                        #We are doing error correction and need to project the point to guide person
                    point_2d=tuple((int(self.project_point_psm1_left[0]),int(self.project_point_psm1_left[1])))
                    self.frame_left_converted=cv2.circle(self.frame_left_converted,point_2d,radius=6,color=(255,0,0),thickness=2)
                if self.project_point_psm3_left is not None: 
                    #We are doing error correction and need to project the point to guide person
                    point_2d=tuple((int(self.project_point_psm3_left[0]),int(self.project_point_psm3_left[1])))
                    self.frame_left_converted=cv2.circle(self.frame_left_converted,point_2d,radius=6,color=(0,255,0),thickness=2)
                
                self.frame_left_converted=self.cvFrame2Gl(self.frame_left_converted)

            elif self.validate_error_correction_on: #We want to validate API error corr.
                self.frame_left_converted=self.frame_left
                ####PSM1
                ecm_T_psm1=self.psm1.measured_cp() #Gets ecm_T_psm1 from API
                ecm_T_psm1=utils.enforceOrthogonalPyKDL(ecm_T_psm1)
                ecm_T_psm1=utils.convertPyDK_To_GLM(ecm_T_psm1)

                test_pose_camera=utils.invHomogeneousGLM(self.ecm_T_lc)*utils.invHomogeneousGLM(self.ecmac_T_ecmrep_psm1)*ecm_T_psm1*input_pose

                rvec,tvec=utils.convertHomoToRvecTvec_GLM(test_pose_camera)
                cv2.drawFrameAxes(self.frame_left_converted,self.aruco_tracker_left.mtx,\
                                    self.aruco_tracker_left.dist,rvec,tvec,0.008,thickness=2)

                #####PSM 3
                ecm_T_psm3=self.psm3.measured_cp() #Gets ecm_T_psm3 from API
                ecm_T_psm3=utils.enforceOrthogonalPyKDL(ecm_T_psm3)
                ecm_T_psm3=utils.convertPyDK_To_GLM(ecm_T_psm3)

                test_pose_camera=utils.invHomogeneousGLM(self.ecm_T_lc)*utils.invHomogeneousGLM(self.ecmac_T_ecmrep_psm3)*ecm_T_psm3*input_pose
                    
                rvec,tvec=utils.convertHomoToRvecTvec_GLM(test_pose_camera)
                cv2.drawFrameAxes(self.frame_left_converted,self.aruco_tracker_left.mtx,\
                                    self.aruco_tracker_left.dist,rvec,tvec,0.008,thickness=2)
                
                self.frame_left_converted=self.cvFrame2Gl(self.frame_left_converted)
            
            else:
                self.frame_left_converted=self.cvFrame2Gl(self.frame_left)



            self.texture_left.write(self.frame_left_converted)
            self.texture_left.use()
            self.vertex_array_left.render()
            self.ctx_left.enable(mgl.DEPTH_TEST)

        ######Render Left Screen Instruments and Camera#######
        if (self.virtual_overlay_on or self.playback_on) and self.aruco_tracker_left.calibrate_done and self.is_psmerror_calib:
            
            #inv_ecmi_T_ecm=utils.invHomogeneousGLM(ecmi_T_ecm)
            #inv_ecmi_T_ecm=utils.scaleGLMTranform(inv_ecmi_T_ecm,3)
            #lc_T_s=opengl_T_opencv*self.inv_ecm_T_lc*inv_ecmi_T_ecm*self.ecm_T_lc*self.lci_T_si #For now don't include ECM motion
            lc_T_s=opengl_T_opencv*self.lci_T_si
            #Update the camera
            lc_T_s=utils.scaleGLMTranform(lc_T_s,METERS_TO_RENDER_SCALE)
            self.camera_left.update(lc_T_s)

            #Render the instruments:
            if self.PSM3_on:
                self.scene_PSM3.render(self.ctx_left)
            if self.PSM1_on:
                self.scene_PSM1.render(self.ctx_left)




        
        ################Render to right window###################
        self.window_right.switch_to()       #Switch to right window
        self.ctx_right.clear()              #Switch to right context

        if self.frame_right is not None:     #We have a right frame from the ECM
            self.ctx_right.disable(mgl.DEPTH_TEST)

            if self.aruco_on:   #User button press, we show aruco and maybe calibrate the scene
                self.frame_right_converted=self.aruco_tracker_right.arucoTrackingScene(self.frame_right) #Show ArUco

                if self.calibrate_on and (not self.aruco_tracker_right.calibrate_done): #Sets Scene Base Frame 
                    self.rci_T_si=None
                    self.aruco_tracker_right.calibrateScene()
                    self.rci_T_si=self.aruco_tracker_right.ci_T_si

                if self.rci_T_si is not None:   #Calibration worked
                    self.inv_rci_T_si=utils.invHomogeneousGLM(self.rci_T_si)
                    cv2.drawFrameAxes(self.frame_right_converted,self.aruco_tracker_right.mtx,\
                                      self.aruco_tracker_right.dist,self.aruco_tracker_right.rvec_scene,self.aruco_tracker_right.tvec_scene,0.05)
                
                self.frame_right_converted=self.cvFrame2Gl(self.frame_right_converted)

            elif self.is_psmerror_started: #We are calibrating the PSM error
                self.frame_right_converted=self.frame_right

                if self.project_point_psm1_right is not None: 
                        #We are doing error correction and need to project the point to guide person
                    point_2d=tuple((int(self.project_point_psm1_right[0]),int(self.project_point_psm1_right[1])))
                    self.frame_right_converted=cv2.circle(self.frame_right_converted,point_2d,radius=6,color=(255,0,0),thickness=2)
                if self.project_point_psm3_right is not None: 
                    #We are doing error correction and need to project the point to guide person
                    point_2d=tuple((int(self.project_point_psm3_right[0]),int(self.project_point_psm3_right[1])))
                    self.frame_right_converted=cv2.circle(self.frame_right_converted,point_2d,radius=6,color=(0,255,0),thickness=2)
                
                self.frame_right_converted=self.cvFrame2Gl(self.frame_right_converted)

            elif self.validate_error_correction_on: #We want to validate API error corr.
                self.frame_right_converted=self.frame_right
                
                test_pose_camera=self.inv_ecm_T_rc*self.inv_ecmac_T_ecmrep_psm1*ecm_T_psm1*input_pose

                rvec,tvec=utils.convertHomoToRvecTvec_GLM(test_pose_camera)
                cv2.drawFrameAxes(self.frame_right_converted,self.aruco_tracker_right.mtx,\
                                    self.aruco_tracker_right.dist,rvec,tvec,0.008,thickness=2)


                test_pose_camera=self.inv_ecm_T_rc*self.inv_ecmac_T_ecmrep_psm3*ecm_T_psm3*input_pose
                    
                rvec,tvec=utils.convertHomoToRvecTvec_GLM(test_pose_camera)
                cv2.drawFrameAxes(self.frame_right_converted,self.aruco_tracker_right.mtx,\
                                    self.aruco_tracker_right.dist,rvec,tvec,0.008,thickness=2)
                
                self.frame_right_converted=self.cvFrame2Gl(self.frame_right_converted)
            
            else:
                self.frame_right_converted=self.cvFrame2Gl(self.frame_right)



            self.texture_right.write(self.frame_right_converted)
            self.texture_right.use()
            self.vertex_array_right.render()
            self.ctx_right.enable(mgl.DEPTH_TEST)
        

        ######Render Right Screen Instruments and Camera#######
        if (self.virtual_overlay_on or self.playback_on) and self.aruco_tracker_right.calibrate_done and self.is_psmerror_calib:
            #rc_T_s=opengl_T_opencv*self.inv_ecm_T_rc*inv_ecmi_T_ecm*self.ecm_T_rc*self.rci_T_si #For now don't include ECM motion
            rc_T_s=opengl_T_opencv*self.rci_T_si

            rc_T_s=utils.scaleGLMTranform(rc_T_s,METERS_TO_RENDER_SCALE)
            #Update the camera
            self.camera_right.update(rc_T_s)

            #Render the instruments:
            if self.PSM3_on:
                self.scene_PSM3.render(self.ctx_right)
            if self.PSM1_on:
                self.scene_PSM1.render(self.ctx_right)


        ########Update GUI
        self.gui_window.update()
        



    

if __name__ == '__main__':
    
    #Initializing rospy

    rospy.init_node('ExpertPlayback')
    rospy.Rate(10000)

    #Initialize the Renderer Class for both right and left eyes
    tool_renderer=Renderer()

    rospy.Subscriber(name = RightFrame_Topic, data_class=CompressedImage, callback=tool_renderer.frameCallbackRight,queue_size=1,buff_size=2**18)
    rospy.Subscriber(name = LeftFrame_Topic, data_class=CompressedImage, callback=tool_renderer.frameCallbackLeft,queue_size=1,buff_size=2**18)
    rospy.Subscriber(name = PC2_Time_Topic,data_class=String,callback=tool_renderer.pc2TimeCallback,queue_size=1,buff_size=2**7)

    tool_renderer.run(frame_rate=1000)