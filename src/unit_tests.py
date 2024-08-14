import rospy
from sensor_msgs.msg import CompressedImage
import cv2
from cv_bridge import CvBridge
import os
from sensor_msgs.msg import Joy


import moderngl as mgl
import sys

import skimage.measure
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

import math
import numpy as np

#TKinter importer
import tkinter as tk

#NDI Tracker Stuff
from sksurgerynditracker.nditracker import NDITracker
import time
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


ARUCO_SIDELENGTH=0.025527
ARUCO_SEPERATION=0.1022477 
ARUCO_HEIGHT_OFFSET=0.005

test_point_scene=glm.vec4(ARUCO_SIDELENGTH,0.0,0.0,1)

test_point_psm=glm.vec4(0.0,0.0,0.0,1.0) #Just the reported tool tip 

RightFrame_Topic='ubc_dVRK_ECM/right/decklink/camera/image_raw/compressed'
LeftFrame_Topic='ubc_dVRK_ECM/left/decklink/camera/image_raw/compressed'
ECM_Topic='ECM/measured_cp'

CONSOLE_VIEWPORT_WIDTH=1400
CONSOLE_VIEWPORT_HEIGHT=986

tool_tip_offset=0.0102
tool_tip_offset_psm3=0.0102  #0.0268


opengl_T_opencv=glm.mat4(glm.vec4(1,0,0,0),
                     glm.vec4(0,1,0,0),
                     glm.vec4(0,0,1,0),
                     glm.vec4(0,0,0,1))

class Renderer:
    def __init__(self,win_size=(CONSOLE_VIEWPORT_WIDTH,CONSOLE_VIEWPORT_HEIGHT)):

        self.WIN_SIZE=win_size

        #Creates the pyglet window
        config=Config(major_version=3,minor_version=3,depth_size=3,double_buffer=True)
        self.window_left = pyglet.window.Window(width=win_size[0], height=win_size[1], config=config,caption='Left Eye')
        self.window_right = pyglet.window.Window(width=win_size[0], height=win_size[1], config=config,caption='Right Eye')


        vertex_source,fragment_source=self.get_program_background('background')

        self.aruco_tracker_left=ArucoTracker.ArucoTracker(self,'left')
        self.aruco_tracker_right=ArucoTracker.ArucoTracker(self,'right')

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


        #############Right Window Initialization
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


        #Frames passed by endoscope
        self.frame_right=None #Updated by the callback in rospy
        self.frame_left=None
        self.frame_left_converted=None
        self.frame_right_converted=None

        self.bridge=CvBridge()


        ##############GUI SETUP################
        self.gui_window=tk.Tk()
        self.gui_window.title("dVRK Playback App")

        self.gui_window.rowconfigure([0,1,2],weight=1)
        self.gui_window.columnconfigure([0,1,2,3,4,5],weight=1)
        self.gui_window.minsize(300,100)

        
        
        self.calibrate_scene_button=tk.Button(self.gui_window,text="Calibrate Scene",command=self.calibrateToggleCallback)
        self.calibrate_scene_button.grid(row=0,column=0,sticky="nsew")

        self.validate_calibration_button=tk.Button(self.gui_window,text="Validate Calibration",command=self.validateCalibrationCallback)
        self.validate_calibration_button.grid(row=0,column=1,sticky="nsew")

        self.validate_handeye_button=tk.Button(self.gui_window,text="Validate Hand-Eye",command=self.validateHandEyeCallback)
        self.validate_handeye_button.grid(row=0,column=2,sticky="nsew")
        
        self.validate_error_correction_button=tk.Button(self.gui_window,text="Validate API Error Correction",command=self.validateErrorCorrectionCallback)
        self.validate_error_correction_button.grid(row=0,column=3,sticky="nsew")

        self.validate_handeyeerror_correction_button=tk.Button(self.gui_window,text="Validate HandEye Error Correction",command=self.validate_handeye_error_correction_callback)
        self.validate_handeyeerror_correction_button.grid(row=0,column=4,sticky="nsew")

        self.validate_ecmmotion_button=tk.Button(self.gui_window,text="Validate ECM Motion",command=self.validateECMMotionCallback)
        self.validate_ecmmotion_button.grid(row=0,column=5,sticky="nsew")




        #Buttons to grap PSM reported points in camera frame for error correction
        self.start_psmerror_calibration=tk.Button(self.gui_window,text="Start PSM Error Calib",command=self.startPSMErrorCalib)
        self.start_psmerror_calibration.grid(row=1,column=0,sticky="nsew")

        self.grab_psm1_pointbutton=tk.Button(self.gui_window,text="Grab PSM1 Point",command=self.grabPSM1PointCallback)
        self.grab_psm1_pointbutton.grid(row=1,column=1,sticky="nsew")

        self.grab_psm3_pointbutton=tk.Button(self.gui_window,text="Grab PSM3 Point",command=self.grabPSM3PointCallback)
        self.grab_psm3_pointbutton.grid(row=1,column=2,sticky="nsew")

        self.calibrate_psmerror_button=tk.Button(self.gui_window,text="Calibrate PSM Error",command=self.calibratePSMErrorCallback)
        self.calibrate_psmerror_button.grid(row=1,column=3,sticky="nsew")


        #Correct hand-eye error:
        self.start_handeyeerror_calibration=tk.Button(self.gui_window,text="Start Hand-Eye Error Calib",command=self.startHandEyeErrorCalib)
        self.start_handeyeerror_calibration.grid(row=2,column=0,sticky="nsew")

        self.grab_handeye_pointbutton=tk.Button(self.gui_window,text="Grab Hand-Eye Point (use PSM1)",command=self.grabHandEyePointCallback)
        self.grab_handeye_pointbutton.grid(row=2,column=1,sticky="nsew")

        self.calibrate_handeye_error_button=tk.Button(self.gui_window,text="Calibrate Hand-Eye Error",command=self.calibrateHandEyeErrorCallback)
        self.calibrate_handeye_error_button.grid(row=2,column=2,sticky="nsew")


        


        ##############Booleans and Counters
        self.calibrate_on=False
        self.validate_HandEye_on=False

        self.validate_calibration_on=False
        self.validate_calibration_on=False

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

        self.validate_error_correction_on=False
        self.validate_handeye_error_correction_on=False

        self.validate_ecmmotion_on=False


        #Hand-eye error correction
        self.handeye_points_count=0
        self.is_handeyeerror_started=False
        self.is_new_handeye_points=False

        self.p_lc_ac_list=None
        self.p_lc_rep_list=None
    


        ###############Transforms

        #Camera to scene registration
        self.lci_T_si=None
        self.rci_T_si=None

        #API Error Correction:

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

        #Hand-Eye Error Correction:
        self.lcac_T_lcrep=None #The Hand-eye correction factor for lc_T_s
        self.lcac_T_lcrep_np=None

        if os.path.isfile(ArucoTracker.DEFAULT_CAMCALIB_DIR+'API_Error_Offset/lcac_T_lcrep.yaml'): 
            with open(ArucoTracker.DEFAULT_CAMCALIB_DIR+'API_Error_Offset/lcac_T_lcrep.yaml','r') as file:
                self.lcac_T_lcrep_np=yaml.load(file) 

                self.lcac_T_lcrep_np=self.lcac_T_lcrep_np['lcac_T_lcrep']
                self.lcac_T_lcrep_np=np.array(self.lcac_T_lcrep_np,dtype=np.float32)

                self.lcac_T_lcrep=glm.mat4(*self.lcac_T_lcrep_np.T.flatten())    
                print('lcac_T_lcrep: '+str(self.lcac_T_lcrep))

        #s_T_psm errror correction
        self.T_psmcorr=None #The error correction for hand-eye for s_T_psm
        self.T_psmcorr_np=None

        if os.path.isfile(ArucoTracker.DEFAULT_CAMCALIB_DIR+'API_Error_Offset/T_psmcorr.yaml'): 
            with open(ArucoTracker.DEFAULT_CAMCALIB_DIR+'API_Error_Offset/T_psmcorr.yaml','r') as file:
                self.T_psmcorr_np=yaml.load(file) 

                self.T_psmcorr_np=self.T_psmcorr_np['T_psmcorr']
                self.T_psmcorr_np=np.array(self.T_psmcorr_np,dtype=np.float32)

                self.T_psmcorr=glm.mat4(*self.T_psmcorr_np.T.flatten())    
                print('lcac_T_lcrep: '+str(self.lcac_T_lcrep))   



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

        #Initial ECM Pose
        self.cart_T_ecmi=None



        ###########Converts the dictionary of model (scene) points to a list of list of points
        ARUCO_IDs=[6,4,5,7]
        self.model_scene_points=None
        for id in ARUCO_IDs:
            curr_points=ArucoTracker.RINGOWIRE_MODELPOINTS[str(id)]
            if self.model_scene_points is None:
                self.model_scene_points=curr_points
            else:
                self.model_scene_points=np.vstack((self.model_scene_points,curr_points))



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

    def validate_handeye_error_correction_callback(self):
        self.validate_handeye_error_correction_on=not self.validate_handeye_error_correction_on

    def validateHandEyeCallback(self):
        self.validate_HandEye_on=True

    def validateErrorCorrectionCallback(self):
        self.validate_error_correction_on=not self.validate_error_correction_on

    def calibrateToggleCallback(self):
        self.calibrate_on=not self.calibrate_on
        self.aruco_tracker_left.calibrate_done=False
        self.aruco_tracker_left.corner_scene_list=[]
        self.aruco_tracker_left.ids_scene_list=[]

        self.aruco_tracker_right.calibrate_done=False
        self.aruco_tracker_right.corner_scene_list=[]
        self.aruco_tracker_right.ids_scene_list=[]

    def validateCalibrationCallback(self):
        self.validate_calibration_on=not self.validate_calibration_on

    def validateECMMotionCallback(self):
        self.validate_ecmmotion_on=not self.validate_ecmmotion_on

    #Method to project points on camera frame:
    def projectPointOnImagePlane(self,ci_T_si,point_si,mtx):
        #Input: ci_T_si (transform from camera to scene), point_si (point in scene frame), mtx (camera intrinsics)
        #Output: point projected on image plane
        
        mul_mat=glm.mat4x3()        #Selection matrix to take [Xc,Yc,Zc]
        cam_mat=mtx 
        cam_mat=glm.mat3(*cam_mat.T.flatten()) #3x3 camera intrinsics, numpy=>glm transpose the matrix

        test_point_camera=ci_T_si*point_si #Gets point in camera coordinate system

        test_point_camera=mul_mat*test_point_camera #Extracts top three entries [Xc,Yc,Zc]
        proj_point=cam_mat*test_point_camera #Projects to image plane

        proj_point[0]=proj_point[0]/proj_point[2] #Normalize by Zc
        proj_point[1]=proj_point[1]/proj_point[2]

        return proj_point



    #########################Methods for Error Correction###########################



    #########API Error Correction Methods

    def startPSMErrorCalib(self):
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

            tool_tip_point=np.array([0,0,tool_tip_offset+0.001,1],dtype=np.float32)

            # point_lc_rep=utils.invHomogeneousNumpy(self.ecm_T_lc_np)@ecm_T_psm_rep@tool_tip_point
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
            #proj_point=self.projectPointOnImagePlane(self.rci_T_si,point_si,self.aruco_tracker_right.mtx)
            #self.project_point_psm1_right=proj_point
    
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

            tool_tip_point=np.array([0,0,tool_tip_offset_psm3+0.001,1],dtype=np.float32)

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
                


                #retval,psm1_rep_T_psm1_ac,_=cv2.estimateAffine3D(self.p_ecm_ac_list_psm1,self.p_ecm_rep_list_psm1,ransacThreshold=RANSAC_THRESHOLD,confidence=RANSAC_CONFIDENCE)
                print("p_ecm_ac_list_psm1: "+str(self.p_ecm_ac_list_psm1))
                print("p_ecm_rep_list_psm1: "+str(self.p_ecm_rep_list_psm1))
                self.ecmac_T_ecmrep_psm1_np,inliers=utils.ransacRigidRransformation(self.p_ecm_ac_list_psm1,self.p_ecm_rep_list_psm1)
                print("T_correct_lc_psm1_np: "+str(self.ecmac_T_ecmrep_psm1_np))
                #self.psm1_rep_T_psm1_ac=utils.convertAffineToHomogeneous(psm1_rep_T_psm1_ac)
                #self.psm1_rep_T_psm1_ac=np.vstack([psm1_rep_T_psm1_ac,[0,0,0,1]])

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
                


                #retval,psm1_rep_T_psm1_ac,_=cv2.estimateAffine3D(self.p_ecm_ac_list_psm1,self.p_ecm_rep_list_psm1,ransacThreshold=RANSAC_THRESHOLD,confidence=RANSAC_CONFIDENCE)
                print("p_ecm_ac_list_psm3: "+str(self.p_ecm_ac_list_psm3))
                print("p_ecm_rep_list_psm3: "+str(self.p_ecm_rep_list_psm3))
                self.ecmac_T_ecmrep_psm3_np,inliers=utils.ransacRigidRransformation(self.p_ecm_ac_list_psm3,self.p_ecm_rep_list_psm3)
                print("T_correct_lc_psm3_np: "+str(self.ecmac_T_ecmrep_psm3_np))
                #self.psm1_rep_T_psm1_ac=utils.convertAffineToHomogeneous(psm1_rep_T_psm1_ac)
                #self.psm1_rep_T_psm1_ac=np.vstack([psm1_rep_T_psm1_ac,[0,0,0,1]])

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




    ##########Hand-Eye Error Correction Methods
    #Gets lcac_T_lcrep, rcac_T_rcrep & simultaneously solves T_psmcorr
    # These are used in the following transforms:
    #s_T_psm=T_psmcorr*inv(lci_T_si)*inv(ecm_T_lc)*ecmi_T_ecm*ecmac_T_ecmrep*ecm_T_psm
    #lc_T_s=lcac_T_lcrep*inv(ecm_T_lc)*inv(ecmi_T_ecm)*ecm_T_lc*lci_T_si
    #rc_T_s=rcac_T_lrrep*inv(ecm_T_lr)*inv(ecmi_T_ecm)*ecm_T_lr*rci_T_si


    def startHandEyeErrorCalib(self):
        self.handeye_points_count=0
        point_si=self.model_scene_points[self.handeye_points_count]
        point_si_list=point_si.tolist()        
        point_si_list.append(1)
        point_si=glm.vec4(point_si_list)
        print("point_si: "+str(point_si))

        #Projecting point on left ecm frame to show 
        proj_point=self.projectPointOnImagePlane(self.lci_T_si,point_si,self.aruco_tracker_left.mtx)
        self.project_point_psm1_left=proj_point
        #Projecting point on right ecm frame to show
        proj_point=self.projectPointOnImagePlane(self.rci_T_si,point_si,self.aruco_tracker_right.mtx)
        self.project_point_psm1_right=proj_point
        self.is_handeyeerror_started=True
            

    def grabHandEyePointCallback(self):
        if self.is_handeyeerror_started:
            self.is_new_handeye_points=True

            #############Gets actual point location in left cam frame
            point_si=self.model_scene_points[self.handeye_points_count]
            point_si_list=point_si.tolist()        
            point_si_list.append(1)
            point_si=glm.vec4(point_si_list)

            #Find current cam-to-scene transform
            lc_T_s=self.aruco_tracker_left.calibrateSceneDirect(self.frame_left)
            print("lc_T_s: "+str(lc_T_s))

            #Left Camera
            point_lc_ac=lc_T_s*point_si

            point_lc_ac_list=[point_lc_ac[0],point_lc_ac[1],point_lc_ac[2]]

            point_lc_ac=np.array(point_lc_ac_list,dtype=np.float32)

            if self.p_lc_ac_list is None:
                self.p_lc_ac_list=point_lc_ac
            else:
                self.p_lc_ac_list=np.vstack((self.p_lc_ac_list,point_lc_ac))

            

            #############Gets Reported Point for left camera based on ecm motion, also gets reported point of psm tip for s_T_psm transform
            
            #Get ecmi_T_ecm transform
            cart_T_ecm=self.ecm.setpoint_cp()
            cart_T_ecm=utils.enforceOrthogonalPyKDL(cart_T_ecm)
            cart_T_ecm=utils.convertPyDK_To_GLM(cart_T_ecm)
            ecmi_T_ecm=utils.invHomogeneousGLM(self.cart_T_ecmi)*cart_T_ecm

            point_lc_rep=utils.invHomogeneousGLM(self.ecm_T_lc)*ecmi_T_ecm*self.ecm_T_lc*self.lci_T_si*point_si

            point_lc_rep_list=[point_lc_rep[0],point_lc_rep[1],point_lc_rep[2]]

            point_lc_rep=np.array(point_lc_rep_list,dtype=np.float32)

            if self.p_lc_rep_list is None:
                self.p_lc_rep_list=point_lc_rep
            else:
                self.p_lc_rep_list=np.vstack((self.p_lc_rep_list,point_lc_rep))


            #Gets same point that PSM is touching:
            '''
            ecm_T_psm_rep=self.psm1.measured_cp()
            ecm_T_psm_rep=utils.enforceOrthogonalPyKDL(ecm_T_psm_rep)
            ecm_T_psm_rep=pm.toMatrix(ecm_T_psm_rep) #Numpy array

            tool_tip_point=np.array([0,0,tool_tip_offset+0.001,1],dtype=np.float32)

            point_lc_rep=utils.invHomogeneousNumpy(self.ecm_T_lc_np)@utils.invHomogeneousNumpy(self.ecmac_T_ecmrep_psm1_np)@ecm_T_psm_rep@tool_tip_point
            point_lc_rep=point_lc_rep[0:3]

            if self.p_lc_rep_list is None:
                self.p_lc_rep_list=point_lc_rep
            else:
                self.p_lc_rep_list=np.vstack((self.p_lc_rep_list,point_lc_rep))

            print("point_lc_ac: "+str(point_lc_rep))
            print("point_lc_rep: "+str(point_lc_rep))
            '''


            ###########Updates Visual Point for next iteration 
            self.handeye_points_count+=1
            count=0
            if self.handeye_points_count>4:
                count=8
            point_si=self.model_scene_points[count]
            point_si_list=point_si.tolist()        
            point_si_list.append(1)
            point_si=glm.vec4(point_si_list)
            #Projecting point on left ecm frame to show 
            proj_point=self.projectPointOnImagePlane(self.lci_T_si,point_si,self.aruco_tracker_left.mtx)
            self.project_point_psm1_left=proj_point
            #Projecting point on right ecm frame to show
            proj_point=self.projectPointOnImagePlane(self.rci_T_si,point_si,self.aruco_tracker_right.mtx)
            self.project_point_psm1_right=proj_point


    def calibrateHandEyeErrorCallback(self):

        #lc:
        #Store the lc points first
        if self.is_new_handeye_points:
            np.save(ArucoTracker.DEFAULT_CAMCALIB_DIR+'API_Error_Offset/p_lc_ac_list.npy',self.p_lc_ac_list)
            np.save(ArucoTracker.DEFAULT_CAMCALIB_DIR+'API_Error_Offset/p_lc_rep_list.npy',self.p_lc_rep_list)
        else:
            self.p_lc_ac_list=np.load(ArucoTracker.DEFAULT_CAMCALIB_DIR+'API_Error_Offset/p_lc_ac_list.npy')
            self.p_lc_rep_list=np.load(ArucoTracker.DEFAULT_CAMCALIB_DIR+'API_Error_Offset/p_lc_rep_list.npy')


        #Finding the error compensation transform, lcac_T_lcrep (for left camera error correction)

        p_lc_points_shape=self.p_lc_rep_list.shape


        if p_lc_points_shape is not None:
            if p_lc_points_shape[0]>3:
                


                self.lcac_T_lcrep_np,inliers=utils.ransacRigidRransformation(self.p_lc_ac_list,self.p_lc_rep_list)


                #Saving Results
                lcac_T_lcrep_list=self.lcac_T_lcrep_np.tolist()
                data_lc={'lcac_T_lcrep':lcac_T_lcrep_list}
                with open(ArucoTracker.DEFAULT_CAMCALIB_DIR+'API_Error_Offset/lcac_T_lcrep.yaml','w') as f:
                    yaml.dump(data_lc,f)

                self.lcac_T_lcrep=glm.mat4(*self.lcac_T_lcrep_np.T.flatten())

                #Finding translation error:
                translation_diff_list=[]
                for i in range(p_lc_points_shape[0]): #Loops for the number of points captured
                    p_lc_rep_point=self.p_lc_rep_list[i].tolist()
                    p_lc_rep_point.append(1)
                    p_lc_rep_point=np.array(p_lc_rep_point,dtype=np.float32)

                    est_point=self.lcac_T_lcrep_np@p_lc_rep_point
                    est_point=est_point[0:3]


                    trans_diff=est_point-self.p_lc_ac_list[i]
                    trans_diff=np.linalg.norm(trans_diff)
                    translation_diff_list.append(trans_diff)
                translation_diff_list=np.array(translation_diff_list,dtype=np.float32)
                trans_diff=np.mean(translation_diff_list)
                print('registration error handeye lc: '+str(trans_diff))

                self.is_psmerror_calib=True



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

    def run(self,frame_rate=100):
        #self.get_time()
        #self.check_events()
        #self.instrument_kinematics([glm.pi()/6,glm.pi()/6,glm.pi()/6,glm.pi()/6],start_pose)
        #self.move_objects()
        #self.camera.update()
        #self.render()
        pyglet.clock.schedule_interval(self.render,1/frame_rate)
        pyglet.app.run()

    def frameCallbackRight(self,data):
        self.frame_right=self.bridge.compressed_imgmsg_to_cv2(data,'passthrough')

    def frameCallbackLeft(self,data):        
        self.frame_left=self.bridge.compressed_imgmsg_to_cv2(data,'passthrough')

    def cvFrame2Gl(self,frame):
        #print('frame conversion')
        #Flips the frame vertically
        frame_new=cv2.flip(frame,0)
        #Converts the frame to RGB
        frame_new=cv2.cvtColor(frame_new,cv2.COLOR_BGR2RGB)
        return frame_new


    def render(self,dt):

        self.window_left.switch_to()       #Switch to left window
        self.ctx_left.clear()              #Switch to left context

        if self.frame_left is not None:
            self.ctx_left.disable(mgl.DEPTH_TEST)

            if self.calibrate_on:
                self.frame_left_converted=self.aruco_tracker_left.arucoTrackingScene(self.frame_left)
                if not self.aruco_tracker_left.calibrate_done:
                    self.lci_T_si=None
                    self.aruco_tracker_left.calibrateScene()
                    self.lci_T_si=self.aruco_tracker_left.ci_T_si

                if self.lci_T_si is not None:
                    cv2.drawFrameAxes(self.frame_left_converted,self.aruco_tracker_left.mtx,\
                                      self.aruco_tracker_left.dist,self.aruco_tracker_left.rvec_scene,self.aruco_tracker_left.tvec_scene,0.05)
                    
                    if self.project_point_psm1_left is not None: 
                        #We are doing error correction and need to project the point to guide person
                        point_2d=tuple((int(self.project_point_psm1_left[0]),int(self.project_point_psm1_left[1])))
                        self.frame_left_converted=cv2.circle(self.frame_left_converted,point_2d,radius=6,color=(255,0,0),thickness=2)
                    if self.project_point_psm3_left is not None: 
                        #We are doing error correction and need to project the point to guide person
                        point_2d=tuple((int(self.project_point_psm3_left[0]),int(self.project_point_psm3_left[1])))
                        self.frame_left_converted=cv2.circle(self.frame_left_converted,point_2d,radius=6,color=(0,255,0),thickness=2)
                        


                    if self.validate_calibration_on: #We project a test point to validate the transform
                        
                        mul_mat=glm.mat4x3()        #Selection matrix to take [Xc,Yc,Zc]
                        cam_mat=self.aruco_tracker_left.mtx 
                        cam_mat=glm.mat3(*cam_mat.T.flatten()) #3x3 camera intrinsics, numpy=>glm transpose the matrix

                        test_point_camera=self.lci_T_si*test_point_scene #Gets point in camera coordinate system

                        test_point_camera=mul_mat*test_point_camera #Extracts top three entries [Xc,Yc,Zc]
                        proj_point=cam_mat*test_point_camera #Projects to image plane

                        proj_point[0]=proj_point[0]/proj_point[2] #Normalize by Zc
                        proj_point[1]=proj_point[1]/proj_point[2]

                        point_2d=tuple((int(proj_point[0]),int(proj_point[1])))
                        self.frame_left_converted=cv2.circle(self.frame_left_converted,point_2d,radius=10,color=(0,255,0),thickness=3)
                        #print("test point: "+str(test_point_scene))
                        # print("test_point_scene: "+str(test_point_scene))
                        #print("mul_mat: "+str(mul_mat))
                        #print("cam_mat: "+str(cam_mat))
                        #print("proj_point"+str(proj_point))


                        #Project with the project points function
                        # test_point_scene_np=np.array([[0.025537,0.0,0.0]],dtype=np.float32)
                        # image_points,_=cv2.projectPoints(test_point_scene_np,self.aruco_tracker_left.rvec_scene,self.aruco_tracker_left.tvec_scene,self.aruco_tracker_left.mtx,self.aruco_tracker_left.dist)
                        
                        # proj_point_np=tuple((int(image_points[0][0][0]),int(image_points[0][0][1])))
                        #self.frame_left_converted=cv2.circle(self.frame_left_converted,proj_point_np,radius=10,color=(0,0,255),thickness=3)

                    if self.validate_ecmmotion_on: #Validate that when moving the ECM we are still able to track point
                        mul_mat=glm.mat4x3()        #Selection matrix to take [Xc,Yc,Zc]
                        cam_mat=self.aruco_tracker_left.mtx 
                        cam_mat=glm.mat3(*cam_mat.T.flatten()) #3x3 camera intrinsics, numpy=>glm transpose the matrix

                        #Gets ECM motion:
                        cart_T_ecm=self.ecm.setpoint_cp()
                        cart_T_ecm=utils.enforceOrthogonalPyKDL(cart_T_ecm)
                        cart_T_ecm=utils.convertPyDK_To_GLM(cart_T_ecm)


                        ecmi_T_ecm=utils.invHomogeneousGLM(self.cart_T_ecmi)*cart_T_ecm
                        #ecmi_T_ecm=utils.scaleGLMTranform(ecmi_T_ecm,5)
                        #print("ecmi_T_ecm: "+str(ecmi_T_ecm))

                        #test_point_camera=utils.invHomogeneousGLM(self.lcac_T_lcrep)*utils.invHomogeneousGLM(self.ecm_T_lc)*utils.invHomogeneousGLM(ecmi_T_ecm)*self.ecm_T_lc*self.lci_T_si*test_point_scene
                        test_point_camera=opengl_T_opencv*utils.invHomogeneousGLM(self.ecm_T_lc)*utils.invHomogeneousGLM(ecmi_T_ecm)*self.ecm_T_lc*self.lci_T_si*test_point_scene



                        test_point_camera=mul_mat*test_point_camera #Extracts top three entries [Xc,Yc,Zc]
                        proj_point=cam_mat*test_point_camera #Projects to image plane
                        
                        #print("X: "+str(proj_point[0]/proj_point[2]))
                        #print("Y: "+str(proj_point[1]/proj_point[2]))
                        u,v=utils.projectPointsWithDistortion(proj_point[0],proj_point[1],proj_point[2],self.aruco_tracker_left.dist,self.aruco_tracker_left.mtx)
                        point_2d=tuple((int(u),int(v)))

                        #proj_point[0]=proj_point[0]/proj_point[2] #Normalize by Zc
                        #proj_point[1]=proj_point[1]/proj_point[2]
                        #point_2d=tuple((int(proj_point[0]),int(proj_point[1])))
                        
                        #print("point_2d: "+str(point_2d))
                        self.frame_left_converted=cv2.circle(self.frame_left_converted,point_2d,radius=10,color=(0,255,0),thickness=3)


                
                    
                self.frame_left_converted=self.cvFrame2Gl(self.frame_left_converted)

            elif self.validate_HandEye_on: 
                #Hand-Eye Validation On

                #######Get point in Camera Coordinates from PSM1 !!!
                ecm_T_psm1=self.psm1.measured_cp() #Gets ecm_T_psm1 from API
                ecm_T_psm1=utils.enforceOrthogonalPyKDL(ecm_T_psm1)
                ecm_T_psm1=utils.convertPyDK_To_GLM(ecm_T_psm1)

                #test_point_camera=utils.invHomogeneousGLM(self.ecm_T_lc)*ecm_T_psm1*test_point_psm #Point in camera coordinate frame

                #print("ecm_T_psm1: "+str(ecm_T_psm1))
                #print("test_point_camera: "+str(test_point_camera))
                #print("hand-eye left")



                ######Project point to image frame

                mul_mat=glm.mat4x3()        #Selection matrix to take [Xc,Yc,Zc]
                cam_mat=self.aruco_tracker_left.mtx 
                cam_mat=glm.mat3(*cam_mat.T.flatten()) #3x3 camera intrinsics, numpy=>glm transpose the matrix
                

                #test_point_camera=mul_mat*test_point_camera #Extracts top three entries [Xc,Yc,Zc]
                #proj_point=cam_mat*test_point_camera #Projects to image plane

                #proj_point[0]=proj_point[0]/proj_point[2] #Normalize by Zc
                #proj_point[1]=proj_point[1]/proj_point[2]

                #print("proj_point: "+str(proj_point))


                #point_2d=tuple((int(proj_point[0]),int(proj_point[1])))
                #print("point_2d: "+str(point_2d))
                #Showing point

                #self.frame_left_converted=cv2.circle(self.frame_left,point_2d,radius=10,color=(255,0,0),thickness=3)
                self.frame_left_converted=self.frame_left

                

                #######Get point in Camera Coordinates from PSM3 !!!############
                ecm_T_psm3=self.psm3.measured_cp() #Gets ecm_T_psm3 from API
                ecm_T_psm3=utils.enforceOrthogonalPyKDL(ecm_T_psm3)
                ecm_T_psm3=utils.convertPyDK_To_GLM(ecm_T_psm3)

                #test_point_camera=utils.invHomogeneousGLM(self.ecm_T_lc)*ecm_T_psm3*test_point_psm #Point in camera coordinate frame

                #print("ecm_T_psm1: "+str(ecm_T_psm1))
                #print("test_point_camera: "+str(test_point_camera))



                ######Project point to image frame

               

                #test_point_camera=mul_mat*test_point_camera #Extracts top three entries [Xc,Yc,Zc]
                #proj_point=cam_mat*test_point_camera #Projects to image plane

                #proj_point[0]=proj_point[0]/proj_point[2] #Normalize by Zc
                #proj_point[1]=proj_point[1]/proj_point[2]

                #print("proj_point: "+str(proj_point))


                #point_2d=tuple((int(proj_point[0]),int(proj_point[1])))
                #print("point_2d: "+str(point_2d))
                #Showing point

                #self.frame_left_converted=cv2.circle(self.frame_left,point_2d,radius=10,color=(255,0,0),thickness=3)
                
                if self.validate_error_correction_on:

                    #Error Correction Validation On
                    #######Get point in Camera Coordinates from !!!!PSM1!!!!!!

                    #PSM1

                    # test_point_camera=utils.invHomogeneousGLM(self.T_correct_lc_psm1)*utils.invHomogeneousGLM(self.ecm_T_lc)*ecm_T_psm1*test_point_psm #Point in camera coordinate frame
                    #test_point_camera=utils.invHomogeneousGLM(self.ecm_T_lc)*utils.invHomogeneousGLM(self.ecmac_T_ecmrep_psm1)*ecm_T_psm1*test_point_psm
                    
                    input_pose=glm.mat4()

                    #test_pose_camera=utils.invHomogeneousGLM(self.T_correct_lc_psm1)*utils.invHomogeneousGLM(self.ecm_T_lc)*ecm_T_psm1*input_pose #Pose of PSM1 in camera coordinate system
                    test_pose_camera=utils.invHomogeneousGLM(self.ecm_T_lc)*utils.invHomogeneousGLM(self.ecmac_T_ecmrep_psm1)*ecm_T_psm1*input_pose

                    '''
                    print("test_point_camera_corrected: "+str(test_point_camera))

                    ######Project point to image frame

                    mul_mat=glm.mat4x3()        #Selection matrix to take [Xc,Yc,Zc]
                    cam_mat=self.aruco_tracker_left.mtx 
                    cam_mat=glm.mat3(*cam_mat.T.flatten()) #3x3 camera intrinsics, numpy=>glm transpose the matrix
                    

                    test_point_camera=mul_mat*test_point_camera #Extracts top three entries [Xc,Yc,Zc]
                    proj_point=cam_mat*test_point_camera #Projects to image plane

                    proj_point[0]=proj_point[0]/proj_point[2] #Normalize by Zc
                    proj_point[1]=proj_point[1]/proj_point[2]

                    #print("proj_point: "+str(proj_point))


                    point_2d=tuple((int(proj_point[0]),int(proj_point[1])))
                    print("point_2d corrected: "+str(point_2d))

                    #Showing point:

                    #self.frame_left_converted=cv2.circle(self.frame_left_converted,point_2d,radius=10,color=(0,0,255),thickness=3)
                    '''
                    ##########Showing pose
                    rvec,tvec=utils.convertHomoToRvecTvec_GLM(test_pose_camera)
                    cv2.drawFrameAxes(self.frame_left_converted,self.aruco_tracker_left.mtx,\
                                      self.aruco_tracker_left.dist,rvec,tvec,0.008,thickness=2)
                    

                    #PSM3

                    test_pose_camera=utils.invHomogeneousGLM(self.ecm_T_lc)*utils.invHomogeneousGLM(self.ecmac_T_ecmrep_psm3)*ecm_T_psm3*input_pose
                    
                    rvec,tvec=utils.convertHomoToRvecTvec_GLM(test_pose_camera)
                    cv2.drawFrameAxes(self.frame_left_converted,self.aruco_tracker_left.mtx,\
                                      self.aruco_tracker_left.dist,rvec,tvec,0.008,thickness=2)
                    
                if self.validate_handeye_error_correction_on:
                    #Hand Eye Error Correction Validation On

                    #######Get point in Camera Coordinates from PSM

                    # test_point_camera=utils.invHomogeneousGLM(self.T_correct_lc_psm1)*utils.invHomogeneousGLM(self.ecm_T_lc)*ecm_T_psm1*test_point_psm #Point in camera coordinate frame
                    #test_point_camera=utils.invHomogeneousGLM(self.lcac_T_lcrep)*utils.invHomogeneousGLM(self.ecm_T_lc)*utils.invHomogeneousGLM(self.ecmac_T_ecmrep_psm1)*ecm_T_psm1*test_point_psm
                    
                    input_pose=glm.mat4()

                    #test_pose_camera=utils.invHomogeneousGLM(self.T_correct_lc_psm1)*utils.invHomogeneousGLM(self.ecm_T_lc)*ecm_T_psm1*input_pose #Pose of PSM1 in camera coordinate system
                    test_pose_camera=utils.invHomogeneousGLM(self.lcac_T_lcrep)*utils.invHomogeneousGLM(self.ecm_T_lc)*utils.invHomogeneousGLM(self.ecmac_T_ecmrep_psm1)*ecm_T_psm1*input_pose

                    
                    print("test_point_camera_corrected: "+str(test_point_camera))

                    ######Project point to image frame
                    '''
                    mul_mat=glm.mat4x3()        #Selection matrix to take [Xc,Yc,Zc]
                    cam_mat=self.aruco_tracker_left.mtx 
                    cam_mat=glm.mat3(*cam_mat.T.flatten()) #3x3 camera intrinsics, numpy=>glm transpose the matrix
                    

                    test_point_camera=mul_mat*test_point_camera #Extracts top three entries [Xc,Yc,Zc]
                    proj_point=cam_mat*test_point_camera #Projects to image plane

                    proj_point[0]=proj_point[0]/proj_point[2] #Normalize by Zc
                    proj_point[1]=proj_point[1]/proj_point[2]

                    #print("proj_point: "+str(proj_point))


                    point_2d=tuple((int(proj_point[0]),int(proj_point[1])))
                    print("point_2d corrected: "+str(point_2d))

                    #Showing point:

                    #self.frame_left_converted=cv2.circle(self.frame_left_converted,point_2d,radius=10,color=(0,0,255),thickness=3)
                    '''
                    ##########Showing pose
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




        '''


        self.window_right.switch_to()       #Switch to right window
        self.ctx_right.clear()              #Switch to right context     


        if self.frame_right is not None:
            self.ctx_right.disable(mgl.DEPTH_TEST)

            if self.calibrate_on:
                self.frame_right_converted=self.aruco_tracker_right.arucoTrackingScene(self.frame_right)
                if not self.aruco_tracker_right.calibrate_done:
                    self.rci_T_si=None
                    self.aruco_tracker_right.calibrateScene()
                    self.rci_T_si=self.aruco_tracker_right.ci_T_si

                if self.rci_T_si is not None:
                    cv2.drawFrameAxes(self.frame_right_converted,self.aruco_tracker_right.mtx,\
                                      self.aruco_tracker_right.dist,self.aruco_tracker_right.rvec_scene,self.aruco_tracker_right.tvec_scene,0.05)
                    
                    if self.project_point_psm1_right is not None: 
                        #We are doing error correction and need to project the point to guide person
                        point_2d=tuple((int(self.project_point_psm1_right[0]),int(self.project_point_psm1_right[1])))
                        self.frame_right_converted=cv2.circle(self.frame_right_converted,point_2d,radius=6,color=(255,0,0),thickness=2)
                        

                    
                    if self.validate_calibration_on: #We project a test point to validate the transform
                        
                        mul_mat=glm.mat4x3()        #Selection matrix to take [Xc,Yc,Zc]
                        cam_mat=self.aruco_tracker_right.mtx 
                        cam_mat=glm.mat3(*cam_mat.T.flatten()) #3x3 camera intrinsics, numpy=>glm transpose the matrix

                        test_point_camera=self.rci_T_si*test_point_scene #Gets point in camera coordinate system

                        test_point_camera=mul_mat*test_point_camera #Extracts top three entries [Xc,Yc,Zc]
                        proj_point=cam_mat*test_point_camera #Projects to image plane
                        
                        proj_point[0]=proj_point[0]/proj_point[2] #Normalize by Zc
                        proj_point[1]=proj_point[1]/proj_point[2]

                        point_2d=tuple((int(proj_point[0]),int(proj_point[1])))
                        self.frame_right_converted=cv2.circle(self.frame_right_converted,point_2d,radius=10,color=(0,255,0),thickness=3)
                
                
                self.frame_right_converted=self.cvFrame2Gl(self.frame_right_converted)

            elif self.validate_HandEye_on: 
                    #Hand-Eye Validation On

                    #######Get point in Camera Coordinates from PSM
                    ecm_T_psm1=self.psm1.measured_cp() #Gets ecm_T_psm1 from API
                    ecm_T_psm1=utils.enforceOrthogonalPyKDL(ecm_T_psm1)
                    ecm_T_psm1=utils.convertPyDK_To_GLM(ecm_T_psm1)
                    

                    test_point_camera=utils.invHomogeneousGLM(self.ecm_T_rc)*ecm_T_psm1*test_point_psm #Point in camera coordinate frame

                    #print("ecm_T_psm1 right: "+str(ecm_T_psm1))
                    #print("test_point_camera right: "+str(test_point_camera))



                    ######Project point to image frame
                    mul_mat=glm.mat4x3()        #Selection matrix to take [Xc,Yc,Zc]
                    cam_mat=self.aruco_tracker_right.mtx 
                    cam_mat=glm.mat3(*cam_mat.T.flatten()) #3x3 camera intrinsics, numpy=>glm transpose the matrix
                    

                    test_point_camera=mul_mat*test_point_camera #Extracts top three entries [Xc,Yc,Zc]
                    proj_point=cam_mat*test_point_camera #Projects to image plane

                    proj_point[0]=proj_point[0]/proj_point[2] #Normalize by Zc
                    proj_point[1]=proj_point[1]/proj_point[2]

                    


                    point_2d=tuple((int(proj_point[0]),int(proj_point[1])))
                    #print("point_2d: "+str(proj_point))

                    #Showing point

                    self.frame_right_converted=cv2.circle(self.frame_right,point_2d,radius=10,color=(255,0,0),thickness=3)

                    self.frame_right_converted=self.cvFrame2Gl(self.frame_right_converted)



            else:
                self.frame_right_converted=self.cvFrame2Gl(self.frame_right)



            self.texture_right.write(self.frame_right_converted)
            self.texture_right.use()
            self.vertex_array_right.render()
            self.ctx_right.enable(mgl.DEPTH_TEST)
        '''

        self.gui_window.update()


if __name__ == '__main__':
    
    #Initializing rospy

    rospy.init_node('ExpertPlayback')
    rospy.Rate(10000)

    #Initialize the Renderer Class for both right and left eyes
    tool_renderer=Renderer()

    rospy.Subscriber(name = RightFrame_Topic, data_class=CompressedImage, callback=tool_renderer.frameCallbackRight,queue_size=1,buff_size=2**18)
    rospy.Subscriber(name = LeftFrame_Topic, data_class=CompressedImage, callback=tool_renderer.frameCallbackLeft,queue_size=1,buff_size=2**18)
    #rospy.Subscriber(Clutch_Topic,Joy,tool_renderer.clutchPedalCallback,queue_size=1,buff_size=1000000)
    #rospy.Subscriber(Camera_Topic,Joy,tool_renderer.cameraPedalCallback,queue_size=1,buff_size=1000000)
    #Running Application
    tool_renderer.run(frame_rate=1000)