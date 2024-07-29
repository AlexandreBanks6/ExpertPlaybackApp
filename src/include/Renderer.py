import rospy
from sensor_msgs.msg import CompressedImage
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


#Image Topic Subscriptions
RightFrame_Topic='ubc_dVRK_ECM/right/decklink/camera/image_raw/compressed'
LeftFrame_Topic='ubc_dVRK_ECM/left/decklink/camera/image_raw/compressed'

##################Constants#####################
#Tool and Console Constants
CONSOLE_VIEWPORT_WIDTH=1400
CONSOLE_VIEWPORT_HEIGHT=986

tool_tip_offset=0.0102  #PSM1 tooltip offset (large needle driver)
tool_tip_offset_psm3=0.0102 #PSM3 tooltip offset

tool_tip_point=np.array([0,0,tool_tip_offset+0.001,1],dtype=np.float32) #Point of tooltip for API correction

mul_mat=glm.mat4x3()  #Selection matrix to take [Xc,Yc,Zc]


class Renderer:
    def __init__(self,win_size=(CONSOLE_VIEWPORT_WIDTH,CONSOLE_VIEWPORT_HEIGHT)):

        self.WIN_SIZE=win_size

        #Creates the pyglet window
        config=Config(major_version=3,minor_version=3,depth_size=3,double_buffer=True)
        self.window_left = pyglet.window.Window(width=win_size[0], height=win_size[1], config=config,caption='Left Eye')
        self.window_right = pyglet.window.Window(width=win_size[0], height=win_size[1], config=config,caption='Right Eye')

        vertex_source,fragment_source=self.get_program_background('background')



        ############Left Window Initialization
        self.window_left.switch_to()
        self.window_left.on_mouse_motion=self.on_mouse_motion_left
        self.window_left.on_key_press=self.on_key_press
        self.window_left.on_key_release=self.on_key_release
        self.ctx_left=mgl.create_context(require=330,standalone=False)
        self.ctx_left.enable(flags=mgl.DEPTH_TEST|mgl.CULL_FACE) #CULL_FACE does not render invisible faces
        self.ctx_left.enable(mgl.BLEND)
        self.camera_left=Camera.Camera(self,'left')

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
        self.camera_right=Camera.Camera(self,'right')

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
        self.gui_window.title("dVRK Playback App")

        self.gui_window.rowconfigure([0,1,2,3,4],weight=1)
        self.gui_window.columnconfigure([0,1,2,3],weight=1)
        self.gui_window.minsize(300,100)

        #Title at top
        self.welcome_text=tk.Label(self.gui_window,text="Welcome to the dVRK Playback App")
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
        self.record_motions_button=tk.Button(self.gui_window,text="Record Motions",command=self.rocordMotionsCallback)
        self.record_motions_button.grid(row=4,column=1,sticky="nsew")

        #Playback Tools Button
        self.render_button=tk.Button(self.gui_window,text="Playback Tools",command=self.playbackMotionsCallback)
        self.render_button.grid(row=4,column=2,sticky="nsew")




        ####################Initializing Variables##################

        #ArUco tracking objects
        self.aruco_tracker_left=ArucoTracker.ArucoTracker(self,'left')
        self.aruco_tracker_right=ArucoTracker.ArucoTracker(self,'right')


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

        #Playback motions
        self.playback_on=False

        ##Time Variables
        self.delta_time=None
        self.record_time=0
        self.render_time=0

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

        #Initial ECM Pose
        self.cart_T_ecmi=None


        ###Getting hand-eye calibration matrices
        with open(ArucoTracker.DEFAULT_CAMCALIB_DIR+'calibration_params_right/hand_eye_calibration_right.yaml','r') as file:
            right_handeye=yaml.load(file)

        with open(ArucoTracker.DEFAULT_CAMCALIB_DIR+'calibration_params_left/hand_eye_calibration_left.yaml','r') as file:
            left_handeye=yaml.load(file)

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
                self.is_psmerror_calib=True 

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
                self.is_psmerror_calib=True 



        
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
        self.virtual_overlay_on=not self.virtual_overlay_on

    def rocordMotionsCallback(self):
        self.record_motions_on=not self.record_motions_on

    def playbackMotionsCallback(self):
        self.playback_on=not self.playback_on

    def frameCallbackRight(self,data):
        self.frame_right=self.bridge.compressed_imgmsg_to_cv2(data,'passthrough')

    def frameCallbackLeft(self,data):        
        self.frame_left=self.bridge.compressed_imgmsg_to_cv2(data,'passthrough')
    

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
        self.is_psmerror_started=True


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

                self.ecmac_T_ecmrep_psm1=glm.mat4(*self.ecmac_T_ecmrep_psm1_np.T.flatten())

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

                self.ecmac_T_ecmrep_psm3=glm.mat4(*self.ecmac_T_ecmrep_psm3_np.T.flatten())

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

            with open(f'shaders/{shader_program_name}.frag') as file:
                fragment_shader_source = file.read()

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
    



    #######################Run Methods#####################

    def run(self,frame_rate=100):
        pyglet.clock.schedule_interval(self.render,1/frame_rate)
        pyglet.app.run()

    
    def render(self,dt):

    



    

if __name__ == '__main__':
    
    #Initializing rospy

    rospy.init_node('ExpertPlayback')
    rospy.Rate(10000)

    #Initialize the Renderer Class for both right and left eyes
    tool_renderer=Renderer()

    rospy.Subscriber(name = RightFrame_Topic, data_class=CompressedImage, callback=tool_renderer.frameCallbackRight,queue_size=1,buff_size=2**18)
    rospy.Subscriber(name = LeftFrame_Topic, data_class=CompressedImage, callback=tool_renderer.frameCallbackLeft,queue_size=1,buff_size=2**18)

    tool_renderer.run(frame_rate=1000)