#Use GLFW instead of pygame as we can create multiple windows
#from glfw.GLFW import *
#from glfw import _GLFWwindow as GLFWwindow
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
import os
#Ros and frame grabber imports
import cv2
from cv_bridge import CvBridge

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
import rospy

import skimage
import tf_conversions.posemath as pm
import tf_conversions
from sensor_msgs.msg import Joy
import yaml
import pandas as pd

CONSOLE_VIEWPORT_WIDTH=1400
CONSOLE_VIEWPORT_HEIGHT=986

MOVE_SPEED=0.5
METERS_TO_RENDER_SCALE=1000 #Maybe add in later (multiply l_pitch2yaw by this)

##########Parameters for NDI Validation##########


#Path of apple 3 ROM file
PATH_TO_NDI_APPLE3='../resources/NDI_DRF_Models/APPLE03.rom' 
PATH_TO_VALIDATION_CSV='../resources/validation/'

MOTIONS_ROOT='../resources/Motions/'

#DH Parameter things (we directly substitute the DH parameters in the operations to save computation)

#DH parameters for 4_C_5 (placing shaft)
T_5_alpha=glm.pi()/2
T_5_cos_alpha=glm.cos(T_5_alpha)
T_5_sin_alpha=glm.sin(T_5_alpha)

#DH parameters for 5_C_6 (placing body)
T_6_alpha=glm.pi()/2
T_6_cos_alpha=glm.cos(T_6_alpha)
T_6_sin_alpha=glm.sin(T_6_alpha)

T_6_a=0.0091*METERS_TO_RENDER_SCALE #body length 


#Rotation Matrices to convert between the standard DH frames and the frame system of the 3D models
T_5_s=glm.mat4(glm.vec4(1,0,0,0),
               glm.vec4(0,0,1,0),
               glm.vec4(0,-1,0,0),
               glm.vec4(0,0,0,1))  #Transform between C5 and Cs (shaft frame)

T_6shift_6=glm.mat4(glm.vec4(1,0,0,0),
               glm.vec4(0,1,0,0),
               glm.vec4(0,0,1,0),
               glm.vec4(T_6_a,0,0,1))

T_6shift_b=glm.mat4(glm.vec4(1,0,0,0),
               glm.vec4(0,0,-1,0),
               glm.vec4(0,1,0,0),
               glm.vec4(0,0,0,1))  #Transform between C6 and Cb (body frame)

T_6_b=utils.invHomogeneousGLM(T_6shift_6)*T_6shift_b

T_jl_jlocal=glm.mat4(glm.vec4(1,0,0,0),
               glm.vec4(0,0,1,0),
               glm.vec4(0,-1,0,0),
               glm.vec4(0,0,0,1))  #Transform between Cjl and Cl (left jaw frame)

T_jr_jrlocal=glm.mat4(glm.vec4(1,0,0,0),
               glm.vec4(0,0,1,0),
               glm.vec4(0,-1,0,0),
               glm.vec4(0,0,0,1))  #Transform between Cjr and Cr (left jaw frame)

tool_tip_offset=0.0102 #meters

T_7_7trans=glm.mat4(glm.vec4(1,0,0,0),
                     glm.vec4(0,1,0,0),
                     glm.vec4(0,0,1,0),
                     glm.vec4(tool_tip_offset,0,0,1)) #Translation from reported tool tip to how we define C7
'''
T_7trans_tpsm=glm.mat4(glm.vec4(0,0,-1,0),
                     glm.vec4(0,1,0,0),
                     glm.vec4(1,0,0,0),
                     glm.vec4(0,0,0,1))
'''
T_7trans_tpsm=glm.mat4(glm.vec4(0,0,1,0),
                     glm.vec4(0,-1,0,0),
                     glm.vec4(1,0,0,0),
                     glm.vec4(0,0,0,1))

T_7_psm=T_7_7trans*T_7trans_tpsm
#T_7_psm=T_7_7trans*T_7trans_tpsm   #How we translate between reported tool tip coordinates and our coordinates
T_psm_7=utils.invHomogeneousGLM(T_7_psm)
obj_names=['shaft','body','jaw_right','jaw_left']



#Transformation to convert OpenCV camera frame to OpenGL frame
opengl_T_opencv=glm.mat4(glm.vec4(1,0,0,0),
                     glm.vec4(0,-1,0,0),
                     glm.vec4(0,0,-1,0),
                     glm.vec4(0,0,0,1))

###For Testing:
start_pose=glm.mat4(glm.vec4(1,0,0,0),
                glm.vec4(0,1,0,0),
                glm.vec4(0,0,1,0),
                glm.vec4(0,0,-30,1))
start_pose2=glm.mat4(glm.vec4(1,0,0,0),
                glm.vec4(0,1,0,0),
                glm.vec4(0,0,1,0),
                glm.vec4(10,-10,-30,1))


##################Init Parameters for Grabbing Frames###################
RightFrame_Topic='ubc_dVRK_ECM/right/decklink/camera/image_raw/compressed'
LeftFrame_Topic='ubc_dVRK_ECM/left/decklink/camera/image_raw/compressed'



#################Header for recording Motions CSV######################
repeat_string=["Tx","Ty","Tz","R00","R01","R02","R10","R11","R12","R20","R21","R22"] #First number is row of R, second number is column
MOTION_HEADER=["Time","si_T_psm1"]+repeat_string+["si_T_psm3"]+repeat_string+["si_T_ecm"]+repeat_string+["joint_vars_psm1"]+["q1","q2","q3","q4","q5","q6","jaw"]+["joint_vars_psm3"]+["q1","q2","q3","q4","q5","q6","jaw"]
class Renderer:
    def __init__(self,win_size=(CONSOLE_VIEWPORT_WIDTH,CONSOLE_VIEWPORT_HEIGHT)):
        #Init pygame modules
        '''
        pg.init()

        #Set the window size
        self.WIN_SIZE=win_size

        #Set opengl attr
        pg.display.gl_set_attribute(pg.GL_CONTEXT_MAJOR_VERSION,3)
        pg.display.gl_set_attribute(pg.GL_CONTEXT_MINOR_VERSION,3)
        pg.display.gl_set_attribute(pg.GL_CONTEXT_PROFILE_MASK,pg.GL_CONTEXT_PROFILE_CORE)

        #Create the opengl context
        pg.display.set_mode(self.WIN_SIZE,flags=pg.OPENGL | pg.DOUBLEBUF)
        '''
        '''
        self.WIN_SIZE=win_size
        #INIT glfw
        glfwInit()
        glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR,3)
        glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR,3)
        glfwWindowHint(GLFW_OPENGL_PROFILE,GLFW_OPENGL_CORE_PROFILE)

        self.window1=glfwCreateWindow(CONSOLE_VIEWPORT_WIDTH,CONSOLE_VIEWPORT_HEIGHT,"First Window",None,None) #This is where we can set the monitor
        glfwMakeContextCurrent(self.window1)
        '''
        self.move_rads=0
        self.WIN_SIZE=win_size

        #Creates the pyglet window
        config=Config(major_version=3,minor_version=3,depth_size=3,double_buffer=True)
        self.window_left = pyglet.window.Window(width=win_size[0], height=win_size[1], config=config,caption='Left Eye')
        self.window_right = pyglet.window.Window(width=win_size[0], height=win_size[1], config=config,caption='Right Eye')
        
        #Background Texture Things:
        #Background Vertex Program
        vertex_source,fragment_source=self.get_program_background('background')
        
        
        
        ############Left Window Initialization
        self.window_left.switch_to()
        self.window_left.on_mouse_motion=self.on_mouse_motion_left
        self.window_left.on_key_press=self.on_key_press
        self.window_left.on_key_release=self.on_key_release
        self.ctx_left=mgl.create_context(require=330,standalone=False)
        self.ctx_left.enable(flags=mgl.DEPTH_TEST|mgl.CULL_FACE) #CULL_FACE does not render invisible faces
        self.ctx_left.enable(mgl.BLEND)
        self.camera_left=Camera.Camera(self)

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
        self.camera_right=Camera.Camera(self)

        #Initializing right background
        self.window_right.switch_to()
        self.background_program_right=self.ctx_right.program(vertex_shader=vertex_source,fragment_shader=fragment_source)
        self.background_program_right['texture0']=0
        self.vertex_array_right=self.init_vertex_array(self.ctx_right,self.background_program_right)

        #self.texture_right=self.ctx_right.texture(self.test_image.size,3,data=self.test_image.tobytes())
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

        print("Done Window and Light Constructor")



     
        #Object to track timemouse
        #self.clock=pg.time.Clock()
        self.clock=pyglet.clock.Clock()
        self.time=0
        self.delta_time=0

        #Initialize the instrument frames
        self.instrument_dict_PSM1=None
        self.instrument_dict_PSM3=None
        self.instrument_kinematics([0,0,0,0],start_pose,'PSM1')
        self.instrument_kinematics([0,0,0,0],start_pose2,'PSM3')

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

        self.frame_right_converted=None
        self.frame_left_converted=None

        self.bridge=CvBridge()



        ############GUI Setup##############
        self.gui_window=tk.Tk()
        self.gui_window.title("dVRK Playback App")

        self.gui_window.rowconfigure([0,1,2,3,4,5,6],weight=1)
        self.gui_window.columnconfigure([0,1,2],weight=1)
        self.gui_window.minsize(300,100)

        #Title at top
        self.welcome_text=tk.Label(self.gui_window,text="Welcome to the dVRK Playback App")
        self.welcome_text.grid(row=0,column=1,sticky='n')
        #Show ArUco Button
        self.aruco_button=tk.Button(self.gui_window,text="Show ArUco",command=self.arucoToggleCallback)
        self.aruco_button.grid(row=1,column=1,sticky="nsew")

        #Calibrate Scene Button
        self.calibrate_scene_button=tk.Button(self.gui_window,text="Calibrate Scene",command=self.calibrateToggleCallback)
        self.calibrate_scene_button.grid(row=2,column=0,sticky="nsew")

        #Select PSM (to record/playback) Checkboxes
        self.checkbox_PSM1=tk.Checkbutton(self.gui_window,text="PSM1",onvalue=1,offvalue=0,command=self.psm1Checkbox)
        self.checkbox_PSM1.grid(row=2,column=1,sticky='nsew')

        self.checkbox_PSM3=tk.Checkbutton(self.gui_window,text="PSM3",onvalue=1,offvalue=0,command=self.psm3Checkbox)
        self.checkbox_PSM3.grid(row=2,column=2,sticky='nsew')

        #Calibrate Gaze Button
        self.calibrate_gaze_button=tk.Button(self.gui_window,text="Calibrate Gaze",command=self.calibrateGazeCallback)
        self.calibrate_gaze_button.grid(row=3,column=0,sticky="nsew")

        #Playback (render) Tools Button
        self.render_button=tk.Button(self.gui_window,text="Playback Tools",command=self.renderButtonPressCallback)
        self.render_button.grid(row=3,column=1,sticky="nsew")

        #Record Expert Motions Button
        self.record_motions_button=tk.Button(self.gui_window,text="Record Motions",command=self.rocordMotionsCallback)
        self.record_motions_button.grid(row=4,column=0,sticky="nsew")

        #Playback the ECM Motion Button (indicates on the screen arrows showing direction for ECM movement)
        self.playback_ecm_button=tk.Button(self.gui_window,text="Playback ECM Motion",command=self.playbackECMCallback)
        self.playback_ecm_button.grid(row=4,column=1,sticky="nsew")

        #Message box to send text to the user
        self.message_box = tk.Text(self.gui_window, height=10, width=80)
        self.message_box.config(state='disabled')
        self.message_box.grid(row=5, column=0, columnspan=3, sticky='nsew', padx=10, pady=10)
        self.displayMessage("Ensure calibration parameters are in file: ../resources/Calib_Best/  \
                            Also, motions will be played from the 'Best' folder: \
                             ../resources/Motions/Motion_Best")

        #Button for NDI Validation
        self.ndi_toggle_button=tk.Button(self.gui_window,text="Start NDI Tracker and Validate",command=self.ToggleTrackerCallback)
        self.ndi_toggle_button.grid(row=6,column=0,sticky="nsew")



        ##############Init Parameters###############
        
        self.aruco_on=False #Whether we show and update aruco poses
        self.calibrate_on=False
        #Selecting PSM buttons
        self.PSM1_on=False
        self.PSM3_on=False

        self.calib_gaze_on=False
        self.render_on=False
        self.record_motions_on=False
        self.playback_ecm_on=False

        #Params for validating the calibration
        self.NDI_TrackerToggle=False #Whether the NDI tracker is on and we want to validate, validation is started with the calibration button
        self.validation_trial_count=None
        self.csv_name=None

        self.record_filename=None #Current Filename that we are writing to

 
        ####Aruco Tracking Setup
        self.aruco_tracker_left=ArucoTracker.ArucoTracker(self,'left')
        self.aruco_tracker_right=ArucoTracker.ArucoTracker(self,'right')


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



        #################Frame Transformations#####################

        #Recorded Value for PSMs with respect to base frame (si)
        self.si_T_psm3_recorded=None    #Gives w_T_7
        self.si_T_psm1_recorded=None    #Gives w_T_7

        #CUrrent Value for PSMw with respect to base frame
        self.si_T_psm1=None
        self.si_T_psm3=None

        ####Recorded Value for ECM w.r.t. base frame (si)
        self.si_T_ecm_recorded=None

        ####Current Value for ECM w.r.t. base
        self.si_T_ecm=None

        #####Current Values (updated every loop, or pre-defined)
        self.lci_T_si=None #The left camera w.r.t. scene base frame (calibrated origin)
        self.rci_T_si=None #The right camera w.r.t. scene base frame (calibrated origin)

        self.ecm_T_lc=None #left hand-eye
        self.ecm_T_rc=None #right hand-eye

        self.ecmini_T_ecm=None #current ecm pose w.r.t. initial ecm pose


        #####Initial ECM Pose
        self.ecm_init_pose=None

        #####PSM w.r.t. ECM
        self.ecm_T_psm1=None
        self.ecm_T_psm3=None

        #####Joint Variables (for recording)
        self.joint_vars_psm1=None
        self.joint_vars_psm3=None
        self.joint_vars_psm1_recorded=None
        self.joint_vars_psm3_recorded=None

        
        ###Getting hand-eye calibration matrices
        with open(ArucoTracker.DEFAULT_CAMCALIB_DIR+'calibration_params_right/hand_eye_calibration_right.yaml','r') as file:
            right_handeye=yaml.load(file)

        with open(ArucoTracker.DEFAULT_CAMCALIB_DIR+'calibration_params_left/hand_eye_calibration_left.yaml','r') as file:
            left_handeye=yaml.load(file)

        self.ecm_T_lc=left_handeye['ecm_T_leftcam']
        self.ecm_T_lc=np.array(self.ecm_T_lc,dtype='float32')
        
        self.ecm_T_lc=utils.EnforceOrthogonalityNumpy_FullTransform(self.ecm_T_lc)
        print("left hand eye: "+str(self.ecm_T_lc))
        self.ecm_T_lc=glm.mat4(*self.ecm_T_lc.flatten())
        
        self.ecm_T_rc=right_handeye['ecm_T_rightcam']
        self.ecm_T_rc=np.array(self.ecm_T_rc,dtype='float32')
        
        self.ecm_T_rc=utils.EnforceOrthogonalityNumpy_FullTransform(self.ecm_T_rc)
        print("right hand eye: "+str(self.ecm_T_rc))
        self.ecm_T_rc=glm.mat4(*self.ecm_T_rc.flatten())
        

        ##Getting current ecm pose
        ecm_pose=self.ecm.measured_cp()
        #print("Initial PyKDL Pose: "+str(ecm_pose.M))
        ecm_pose=utils.enforceOrthogonalPyKDL(ecm_pose)
        ecm_pose=utils.convertPyDK_To_GLM(ecm_pose)
        #print("Initial GLM Pose: "+str(ecm_pose))
        self.ecm_init_pose=ecm_pose


        ##Time Variables
        self.delta_time=None
        self.record_time=0
        self.render_time=0

        self.render_filename=None #Filename of the .csv that we are rendering from
        self.render_times_list=None #Variable that holds a list of the recorded times
        rospy.sleep(0.2)

    def displayMessage(self,message):
        #function to insert messages in the text box
        self.message_box.config(state='normal')
        self.message_box.delete("1.0",tk.END)

        self.message_box.insert(tk.END, message + "\n")
        self.message_box.see(tk.END)

        self.message_box.config(state='disabled')


    def arucoToggleCallback(self):
        self.aruco_on=not self.aruco_on


    def calibrateToggleCallback(self):
        self.calibrate_on=not self.calibrate_on
        self.aruco_tracker_left.calibrate_done=False
        self.aruco_tracker_right.calibrate_done=False


        if self.NDI_TrackerToggle and self.calibrate_on: #We want to validate the pose estimates

            if not os.path.isfile(self.csv_name):
                self.validation_trial_count=1
                with open(self.csv_name,'w',newline='') as file_object:
                    writer_object=csv.writer(file_object)
                    writer_object.writerow(["Timestamp","Trial #","NDI_Tracker",\
                                            "Tx","Ty","Tz","Q0","Qx","Qy","Qz","Tracking Quality",\
                                            "Visual_Tracking","Tx","Ty","Tz","Q0","Qx","Qy","Qz"])
                    file_object.close()
            else:
                self.validation_trial_count+=1
                with open(self.csv_name,'a',newline='') as file_object:
                    writer_object=csv.writer(file_object)
                    writer_object.writerow("\n")
                    file_object.close()

    def psm1Checkbox(self):
        self.PSM1_on=not self.PSM1_on

    def psm3Checkbox(self):
        self.PSM3_on=not self.PSM3_on

    def calibrateGazeCallback(self):
        self.calib_gaze_on=not self.calib_gaze_on

    def renderButtonPressCallback(self):
        self.render_on=not self.render_on
        #Set render time to zero (where we are in the rendering)
        self.render_time=0
        self.render_filename=MOTIONS_ROOT+'Motion_Best.csv'

        #We extract the first column of the csv which is the time recorded
        with open(self.render_filename,'r') as file_object:
            csv_reader=csv.reader(file_object)

            #Skip the first 9 rows because we don't care about that info
            for _ in range(9):
                next(csv_reader,None)
            
            #Read the first column
            self.render_times_list=[float(row[0]) for row in csv_reader if row] #Loops for the rows and returns time as a list of floats
            self.render_times_list=np.array(self.render_times_list,dtype='float32')


            file_object.close()





    def rocordMotionsCallback(self):
        self.record_motions_on=not self.record_motions_on
        self.record_filename

        self.record_time=0

        if self.record_motions_on: 
            ###########Initialize a new csv file to write motions
            file_count=1
            file_name=MOTIONS_ROOT+'Motion_'+str(file_count)+'.csv'
            #Gets a new filename
            while True:
                if os.path.isfile(file_name):
                    file_count+=1
                    file_name=MOTIONS_ROOT+'Motion_'+str(file_count)+'.csv'                
                else:
                    break
            
            self.record_filename=file_name

            #Writing .csv file and initial values
            with open(self.record_filename,'w',newline='') as file_object:
                writer_object=csv.writer(file_object)
                writer_object.writerow(MOTION_HEADER)    #Writing the Header

                #Writing si_T_lci (scene to left camera initial)
                writer_object.writerow(['lci_T_si'])
                lci_T_si_numpy=np.array(glm.transpose(self.lci_T_si).to_list(),dtype='float32')
                lci_T_si_list=utils.convertHomogeneousToCSVROW(lci_T_si_numpy)
                writer_object.writerow(["0"]+lci_T_si_list)

                #Writing si_T_rci (scene to right camera initial)
                writer_object.writerow(['rci_T_si'])
                rci_T_si_numpy=np.array(glm.transpose(self.rci_T_si).to_list(),dtype='float32')
                rci_T_si_list=utils.convertHomogeneousToCSVROW(rci_T_si_numpy)
                writer_object.writerow(["0"]+rci_T_si_list)

                #Writing lc_T_ecm (hand-eye left)
                writer_object.writerow(['ecm_T_lc'])
                ecm_T_lc_numpy=np.array(glm.transpose(self.ecm_T_lc).to_list(),dtype='float32')
                ecm_T_lc_list=utils.convertHomogeneousToCSVROW(ecm_T_lc_numpy)
                writer_object.writerow(["0"]+ecm_T_lc_list)

                #Writing rc_T_ecm (hand-eye right)
                writer_object.writerow(['ecm_T_rc'])
                ecm_T_rc_numpy=np.array(glm.transpose(self.ecm_T_rc).to_list(),dtype='float32')
                ecm_T_rc_list=utils.convertHomogeneousToCSVROW(ecm_T_rc_numpy)
                writer_object.writerow(["0"]+ecm_T_rc_list)



                
                file_object.close()
        


    def playbackECMCallback(self):
        self.playback_ecm_on=not self.playback_ecm_on
        

    def ToggleTrackerCallback(self):
        if not self.NDI_TrackerToggle: #Tracker is not running, so we toggle on
            ##########NDI Tracker Setup
            #print("Path To Tracker: "+str(PATH_TO_NDI_APPLE3))
            settings={
                    "tracker type": "polaris",
                    "romfiles": [PATH_TO_NDI_APPLE3]
                }
            self.ndi_tracker=NDITracker(settings)
            self.ndi_tracker.use_quaternions=True
            self.ndi_tracker.start_tracking()
            self.ndi_toggle_text.config(text='Started NDI Tracker')

            #Create csv to store data
            now=datetime.now()
            dt_string=now.strftime("%d-%m-%Y_%H-%M-%S")
            self.csv_name=PATH_TO_VALIDATION_CSV+'validation_'+dt_string+'_.csv'

        else:   #Tracker is running so we toggle off
            self.ndi_tracker.stop_tracking()
            self.ndi_tracker.close()
            self.ndi_toggle_text.config(text='Stopped NDI Tracker')
            self.csv_name=None



        self.NDI_TrackerToggle=not self.NDI_TrackerToggle

    def frameCallbackRight(self,data):
        self.frame_right=self.bridge.compressed_imgmsg_to_cv2(data,'passthrough')

    def frameCallbackLeft(self,data):        
        self.frame_left=self.bridge.compressed_imgmsg_to_cv2(data,'passthrough')

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
            self.render_button.destroy()
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

    def check_events(self):
        #if(glfwGetKey(self.window1,GLFW_KEY_ESCAPE)==GLFW_PRESS):
         #   glfwTerminate()
          #  sys.exit()
        #print("Key Pressed")writeToNDIVal
        if self.keys[key.ESCAPE]:
            print("Escape Pressed")
           # self.window.close()
            sys.exit()

        
        #for event in pg.event.get():
         #   if event.type==pg.QUIT or (event.type==pg.KEYDOWN and event.key==pg.K_ESCAPE):
          #      self.mesh.destroy()
           #     pg.quit()
            #    sys.exit()



    def render(self,dt):

        #self.move_rads+=0.01

        #Get Time
        #self.get_time() 
        #print("delta time: "+str(self.delta_time))
        self.delta_time=dt
        #print("dt: "+str(dt))
        #print("Calibration Done Left? "+str(self.aruco_tracker_left.calibrate_done))
        #print("Calibration Done Right? "+str(self.aruco_tracker_right.calibrate_done))
        if self.aruco_tracker_left.calibrate_done and self.aruco_tracker_right.calibrate_done:
            #Maybe move these commands under if statements in next block
            #Getting Current Poses (ecmini_T_ecm = curr ecm w.r.t. initial ecm), ecm_T_psm1=psm1 w.r.t. ecm, ecm_T_psm3 = psm3 w.r.t. ecm
            ecm_pose=self.ecm.measured_cp()
            #print("Updated PyKDL Pose: "+str(ecm_pose.M))
            ecm_pose=utils.enforceOrthogonalPyKDL(ecm_pose)
            ecm_pose=utils.convertPyDK_To_GLM(ecm_pose)
            #print("Updated GLM Pose: "+str(ecm_pose))
            self.ecmini_T_ecm=utils.invHomogeneousGLM(self.ecm_init_pose)*ecm_pose
            #print("ecmini_T_ecm: "+str(self.ecmini_T_ecm))
            self.si_T_ecm=utils.invHomogeneousGLM(self.lci_T_si)*utils.invHomogeneousGLM(self.ecm_T_lc)*self.ecmini_T_ecm
            if not self.render_on:
                if self.PSM1_on:
                    psm1_pose=self.psm1.measured_cp()
                    psm1_pose=utils.enforceOrthogonalPyKDL(psm1_pose)
                    self.ecm_T_psm1=utils.convertPyDK_To_GLM(psm1_pose)
                    #print("ecm_T_psm1: "+str(self.ecm_T_psm1))

                    joint_vars_psm1=self.psm1.measured_js()[0]
                    jaw_angle_psm1=self.psm1.jaw.measured_js()[0]
                    self.joint_vars_psm1=[joint_vars_psm1[0],joint_vars_psm1[1],joint_vars_psm1[2],joint_vars_psm1[3],joint_vars_psm1[4],joint_vars_psm1[5],jaw_angle_psm1[0]]
                    #print("joint vars psm1: "+str(self.joint_vars_psm1))
                    self.si_T_psm1=self.si_T_ecm*self.ecm_T_psm1
                
                if self.PSM3_on:

                    psm3_pose=self.psm3.measured_cp()
                    psm3_pose=utils.enforceOrthogonalPyKDL(psm3_pose)
                    self.ecm_T_psm3=utils.convertPyDK_To_GLM(psm3_pose)
                    #print("ecm_T_psm3: "+str(self.ecm_T_psm3))

                    #Get joint positions
                    joint_vars_psm3=self.psm3.measured_js()[0]
                    jaw_angle_psm3=self.psm3.jaw.measured_js()[0]
                    self.joint_vars_psm3=[joint_vars_psm3[0],joint_vars_psm3[1],joint_vars_psm3[2],joint_vars_psm3[3],joint_vars_psm3[4],joint_vars_psm3[5],jaw_angle_psm3[0]]
                    self.si_T_psm3=self.si_T_ecm*self.ecm_T_psm3

            ################Move Instruments if Playback is Pushed###############
            if self.render_on:
                self.render_time+=self.delta_time   #Update the time since start of rendering
                #Get the row of recording with time closest to rendering time
                index=np.argmin(np.abs(self.render_times_list-self.render_time))    #Gets index of row in recorded time closest to current time
                print("Index: "+str(index))
                data_row=pd.read_csv(self.render_filename,skiprows=9+index,nrows=1)
                data_list=data_row.iloc[0].to_list() #Converts the pd row to a list
                if index>=(len(self.render_times_list)-1):    #Reset the rendering time, so this inherently does looping for us
                    self.render_time=0.0



                if self.PSM1_on:
                    #PSM1:
                    self.si_T_psm1_recorded=data_list[2:14]
                    #print("si_T_psm1_list: "+str(self.si_T_psm1_recorded))
                    self.si_T_psm1_recorded=self.ConvertDataRow_ToGLMPose(self.si_T_psm1_recorded)
                    self.joint_vars_psm1_recorded=data_list[44:48]
                    #print("joint vars psm1 recorded: "+str(self.joint_vars_psm1_recorded))
                    self.instrument_kinematics(self.joint_vars_psm1_recorded,self.si_T_psm1_recorded,'PSM1')
                if self.PSM3_on:
                    #PSM3:
                    self.si_T_psm3_recorded=data_list[15:27]
                    self.si_T_psm3_recorded=self.ConvertDataRow_ToGLMPose(self.si_T_psm3_recorded)
                    self.joint_vars_psm3_recorded=data_list[52:58]

                    self.instrument_kinematics(self.joint_vars_psm3_recorded,self.si_T_psm3_recorded,'PSM3')

                self.move_objects() 
            
            ######################Recording Surgeon Movements#####################
            if self.record_motions_on and not self.render_on:
                self.record_time+=self.delta_time
                #We update the motions .csv
                with open(self.record_filename,'a',newline='') as file_object:
                    writer_object=csv.writer(file_object)

                    si_T_ecm_numpy=np.array(glm.transpose(self.si_T_ecm).to_list(),dtype='float32')
                    si_T_ecm_list=utils.convertHomogeneousToCSVROW(si_T_ecm_numpy)
                    
                    if self.PSM1_on and self.PSM3_on:
                        si_T_psm3_numpy=np.array(glm.transpose(self.si_T_psm3).to_list(),dtype='float32')
                        si_T_psm3_list=utils.convertHomogeneousToCSVROW(si_T_psm3_numpy)
                        si_T_psm1_numpy=np.array(glm.transpose(self.si_T_psm1).to_list(),dtype='float32')
                        si_T_psm1_list=utils.convertHomogeneousToCSVROW(si_T_psm1_numpy)

                        joint_list_psm1=[str(num) for num in self.joint_vars_psm1]
                        joint_list_psm3=[str(num) for num in self.joint_vars_psm3]

                        row_to_write=[str(self.record_time),""]+si_T_psm1_list+[""]+si_T_psm3_list+[""]+si_T_ecm_list+[""]+joint_list_psm1+[""]+joint_list_psm3
                        writer_object.writerow(row_to_write)
                    elif self.PSM1_on:

                        si_T_psm1_numpy=np.array(glm.transpose(self.si_T_psm1).to_list(),dtype='float32')
                        si_T_psm1_list=utils.convertHomogeneousToCSVROW(si_T_psm1_numpy)

                        joint_list_psm1=[str(num) for num in self.joint_vars_psm1]

                        row_to_write=[str(self.record_time),""]+si_T_psm1_list+[""]+[""]*12+[""]+si_T_ecm_list+[""]+joint_list_psm1+[""]+[""]*7
                        writer_object.writerow(row_to_write)

                    elif self.PSM3_on:
                        si_T_psm3_numpy=np.array(glm.transpose(self.si_T_psm3).to_list(),dtype='float32')
                        si_T_psm3_list=utils.convertHomogeneousToCSVROW(si_T_psm3_numpy)

                        joint_list_psm3=[str(num) for num in self.joint_vars_psm3]

                        row_to_write=[str(self.record_time),""]+[""]*12+[""]+si_T_psm3_list+[""]+si_T_ecm_list+[""]+[""]*7+[""]+joint_list_psm3
                        writer_object.writerow(row_to_write)

                    else:
                        row_to_write=[str(self.record_time),""]+[""]*12+[""]+[""]*12+[""]+si_T_ecm_list+[""]+[""]*7+[""]+[""]*12
                        writer_object.writerow(row_to_write)


                    file_object.close()



        ###########################Render Left Window##########################
        self.window_left.switch_to()       #Switch to left window
        self.ctx_left.clear()              #Switch to left context

        ####Updating Background Image
        if self.frame_left is not None:
            self.ctx_left.disable(mgl.DEPTH_TEST)
            #background_image_left=Image.open('textures/Sunflower.jpg').transpose(Image.FLIP_TOP_BOTTOM)
            
            if self.aruco_on:
                self.frame_left_converted=self.aruco_tracker_left.arucoTrackingScene(self.frame_left)

                #If Scene Calibration is On
                if self.calibrate_on and (not self.aruco_tracker_left.calibrate_done): #Sets Scene Base Frame 

                    self.lci_T_si=None
                    self.aruco_tracker_left.calibrateScene()
                    self.lci_T_si=self.aruco_tracker_left.ci_T_si

                    #Writing to CSV For Validation
                    if self.NDI_TrackerToggle:
                        NDI_dat=self.ndi_tracker.get_frame() #Grabs the NDI tracker data
                        self.writeToNDIVal(NDI_dat)
      
                if self.lci_T_si is not None and (not self.calib_gaze_on) and (not self.record_motions_on) and (not self.render_on) and (not self.playback_ecm_on): #We show scene coord system                      
                    
                    cv2.drawFrameAxes(self.frame_left_converted,self.aruco_tracker_left.mtx,\
                                      self.aruco_tracker_left.dist,self.aruco_tracker_left.rvec_scene,self.aruco_tracker_left.tvec_scene,0.05)
                  
                self.frame_left_converted=self.cvFrame2Gl(self.frame_left_converted)
            
            else:
                self.frame_left_converted=self.cvFrame2Gl(self.frame_left)



            self.texture_left.write(self.frame_left_converted)
            self.texture_left.use()
            self.vertex_array_left.render()
            self.ctx_left.enable(mgl.DEPTH_TEST)

        ######Rendering Left Screen Instruments
        if self.render_on and self.aruco_tracker_left.calibrate_done and self.aruco_tracker_right.calibrate_done:
            #self.camera_left.update(None) 
            lc_T_si=utils.invHomogeneousGLM(self.ecm_T_lc)*utils.invHomogeneousGLM(self.ecmini_T_ecm)*self.ecm_T_lc*self.lci_T_si

            #Convert opencv frame to opengl frame
            lc_T_si=opengl_T_opencv*lc_T_si
            #print("Camera Left Pose: "+str(cam_left_pose))
            self.camera_left.update(None)
            #self.camera_left.update(lc_T_si)
            #self.scene.render(self.ctx_left)
            if self.PSM1_on:
                self.scene_PSM1.render(self.ctx_left)
            if self.PSM3_on:
                self.scene_PSM3.render(self.ctx_left)
            #print("Render Update Left")
                
        
        ###########################Render the Right Window##########################
        self.window_right.switch_to()
        self.ctx_right.clear()

        ######Updating Background Image
        if self.frame_right is not None:
            self.ctx_right.disable(mgl.DEPTH_TEST)

            if self.aruco_on:
                self.frame_right_converted=self.aruco_tracker_right.arucoTrackingScene(self.frame_right)                
                
                #If Scene Calibration is On
                if self.calibrate_on and (not self.aruco_tracker_right.calibrate_done): #Sets Scene Base Frame

                    self.rci_T_si=None
                    self.aruco_tracker_right.calibrateScene()
                    self.rci_T_si=self.aruco_tracker_right.ci_T_si

                if (self.rci_T_si is not None) and (not self.calib_gaze_on) and (not self.record_motions_on) and (not self.render_on) and (not self.playback_ecm_on):

                    cv2.drawFrameAxes(self.frame_right_converted,self.aruco_tracker_right.mtx,\
                                      self.aruco_tracker_right.dist,self.aruco_tracker_right.rvec_scene,self.aruco_tracker_right.tvec_scene,0.05) #Look at rvec and tvec, look at frame_right_converted, look at calibrate on
                  
                self.frame_right_converted=self.cvFrame2Gl(self.frame_right_converted)
            
            else:
                self.frame_right_converted=self.cvFrame2Gl(self.frame_right)


            self.texture_right.write(self.frame_right_converted)
            self.texture_right.use()
            self.vertex_array_right.render()
            self.ctx_right.enable(mgl.DEPTH_TEST)

        
        #Render the right screen 

        if self.render_on and self.aruco_tracker_left.calibrate_done and self.aruco_tracker_right.calibrate_done:
            self.camera_right.update(None)
            rc_T_si=utils.invHomogeneousGLM(self.ecm_T_rc)*utils.invHomogeneousGLM(self.ecmini_T_ecm)*self.ecm_T_rc*self.rci_T_si

            #Convert cam pose in opencv to opengl pose
            rc_T_si=opengl_T_opencv*rc_T_si

            #self.camera_left.update(rc_T_si)
            #print("Camera Right Pose: "+str(cam_right_pose))
            #self.scene.render(self.ctx_right)
            if self.PSM1_on:
                self.scene_PSM1.render(self.ctx_right)
            if self.PSM3_on:
                self.scene_PSM3.render(self.ctx_right)
            #print("Render Update Right")
                   
        ####Updates Gui

        self.gui_window.update()

    '''
    def drawAxes(self,img,corners,imgpts):
        print("Corners: "+str(corners))
        corner=tuple(int(x) for x in corners)
        def tupleOfInts(arr):
            return tuple(int(x) for x in arr)
        print("Corner new: "+str(corner))
        #corner = tupleOfInts(corners.ravel())
        img = cv2.line(img,corner,tupleOfInts(imgpts[0].ravel()),(255,0,0),5)
        img = cv2.line(img,corner,tupleOfInts(imgpts[1].ravel()),(0,255,0),5)
        img = cv2.line(img,corner,tupleOfInts(imgpts[2].ravel()),(0,0,255),5)
        return img
    '''
    def writeToNDIVal(self,NDI_dat):
        timestamp=NDI_dat[1]    #Timestamp
        quat_list=NDI_dat[3] #quaternion
        quality=NDI_dat[4]  #Tracking Quality

        data_list=[" "]*19 #Initializes the list of data as empty spaces

        data_list[0]=timestamp
        data_list[1]=self.validation_trial_count

        if len(quat_list)>0: #Quaternion is found                            
            quat_list=quat_list[0][0].tolist()
            data_list[3:9]=quat_list
            data_list[10]=quality[0]
        if self.si_T_lci is not None: #We got the calibrated frame
            translation=[self.si_T_lci[3,0],self.si_T_lci[3,1],self.si_T_lci[3,2]]
            rotation=glm.mat3(self.si_T_lci) #Takes top left of transform matrix
            quaternion=glm.quat_cast(rotation)
            data_list[12:14]=translation
            data_list[15:18]=quaternion
        with open(self.csv_name,'a',newline='') as file_object:
            writer_object=csv.writer(file_object)
            writer_object.writerow(data_list)
            file_object.close()

    def cvFrame2Gl(self,frame):
        #print('frame conversion')
        #Flips the frame vertically
        frame_new=cv2.flip(frame,0)
        #Converts the frame to RGB
        frame_new=cv2.cvtColor(frame_new,cv2.COLOR_BGR2RGB)
        return frame_new

    def get_time(self):
        #Time in seconds (float)
        #self.time=pg.time.get_ticks()*0.001
        self.delta_time=self.clock.update_time()

    ########### Methods to position instrument/camera
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

    def instrument_kinematics(self,joint_angles,w_T_psm,PSM_Type):
        '''
        Update: q5=q4, q6=q5, q7=q6, and jaw based on the way that the dVRK reports it
        Input: 
        - list of angles of the intrument joints: shaft rotation (q5), body rotation (q6), jaw rotation (q7), jaw seperation (theta_j)
        - Tool tip with respect to world (w_T_psm)
        Output: 
        - Frames for each instrument segment: shaft frame (Ts), body frame (Tb), left jaw (Tl), right jaw (Tr)  

        !!!! Note, input "shaft base" should be a 4x4 frame matrix (orientation and pose)
        Also note, T4 is the base of the instrument, and T5 is the shaft rotated frame, T6 is the body rotated frame, T7 is the jaw base (no seperation)
        '''
        print("Joint Angles: "+str(joint_angles))
        #Enforce >=0 jaw angle
        #print("Joint Angles: "+str(joint_angles))
        if joint_angles[3]<0:
            joint_angles[3]=0
        print("w_T_psm: "+str(w_T_psm))
        print("T_psm_7: "+str(T_psm_7))
        w_T_jlocal=w_T_psm*T_psm_7*self.Rotz(-joint_angles[3]/2)*T_jl_jlocal    #Finds world to jaw left in object coords (local)
        w_T_jrlocal=w_T_psm*T_psm_7*self.Rotz(joint_angles[3]/2)*T_jr_jrlocal   #Finds world to right jaw
        w_T_bodylocal=w_T_psm*T_psm_7*utils.invHomogeneousGLM(self.transform_6_T_7(joint_angles[2]))*T_6_b
        w_T_shaftlocal=w_T_psm*T_psm_7*utils.invHomogeneousGLM(self.transform_6_T_7(joint_angles[2]))*utils.invHomogeneousGLM(self.transform_5_T_6(joint_angles[1]))*T_5_s

        #First convert reported tool-tip pose (T7_rep) to our tool tip coodinate system (T7) using DH parameters
        #T7=T7_rep*T_7rep_7

        #Then work backwards from tool tip to get each frame
        #T6=T7*utils.invHomogeneousGLM(self.transform_6_T_7(joint_angles[2]))
        #T5=T6*utils.invHomogeneousGLM(self.transform_5_T_6(joint_angles[1]))
        #T4=T5*utils.invHomogeneousGLM(self.transform_4_T_5(joint_angles[0]))
        #Tjl=T7*self.Rotz(-joint_angles[3]/2)
        #Tjr=T7*self.Rotz(joint_angles[3]/2)


        #This is going forward (if we are given T4), maybe revert to this later
        '''
        T5=T4*self.transform_4_T_5(joint_angles[0])
        T6=T5*self.transform_5_T_6(joint_angles[1])
        T7=T6*self.transform_6_T_7(joint_angles[2])
        Tjl=T7*self.Rotz(-joint_angles[3]/2)
        Tjr=T7*self.Rotz(joint_angles[3]/2)
        '''

        #Next we convert these standard coordinates to the coordinates of our 3D objects (these are augmented)
        
        #Ts=T5*C_5_s
        #Tb=glm.translate(T6,glm.vec3(-T_6_a,0,0))*C_6_b #Brings back body frame to base of 3D model        
        #Tl=Tjl*C_l_jl
        #Tr=Tjr*C_r_jr


        if PSM_Type=='PSM1':
            self.instrument_dict_PSM1={
                'shaft': w_T_shaftlocal,
                'body': w_T_bodylocal,
                'jaw_right': w_T_jrlocal,
                'jaw_left': w_T_jlocal
            }

        elif PSM_Type=='PSM3':
            self.instrument_dict_PSM3={
                'shaft': w_T_shaftlocal,
                'body': w_T_bodylocal,
                'jaw_right': w_T_jrlocal,
                'jaw_left': w_T_jlocal
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
    def ConvertDataRow_ToGLMPose(self,data_list):
        transform=glm.mat4(glm.vec4(data_list[3],data_list[6],data_list[9],0),
                           glm.vec4(data_list[4],data_list[7],data_list[10],0),
                           glm.vec4(data_list[5],data_list[8],data_list[11],0),
                           glm.vec4(data_list[0],data_list[1],data_list[2],1))
        return transform



    
    ############ Main Render Loop
    def run(self,frame_rate=100):
        #self.get_time()
        #self.check_events()
        #self.instrument_kinematics([glm.pi()/6,glm.pi()/6,glm.pi()/6,glm.pi()/6],start_pose)
        #self.move_objects()
        #self.camera.update()
        #self.render()
        pyglet.clock.schedule_interval(self.render,1/frame_rate)
        pyglet.app.run()
        #glfwPollEvents()
        #self.delta_time=self.clock.tick(delay)
        
        
        '''
        #Used to start the application
        while True:
            self.get_time()
            self.check_events()
            self.instrument_kinematics([glm.pi()/6,glm.pi()/6,glm.pi()/6,glm.pi()/6],start_pose)
            self.move_objects()
            self.camera.update()
            self.render()
            self.delta_time=self.clock.tick(60)
        '''