import pygame as pg

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

CONSOLE_VIEWPORT_WIDTH=1400
CONSOLE_VIEWPORT_HEIGHT=986

MOVE_SPEED=0.5
METERS_TO_RENDER_SCALE=1000

##########Parameters for Teleoperation###########
#PSM Position Motion Scale Factor
PSM_SCALE_FACTOR=0.5

##########Parameters for NDI Validation##########


#Path of apple 3 ROM file
PATH_TO_NDI_APPLE3='../resources/NDI_DRF_Models/APPLE03.rom' 
PATH_TO_VALIDATION_CSV='../resources/validation/'

#DH Parameter things (we directly substitute the DH parameters in the operations to save computation)

#DH parameters for 4_C_5 (placing shaft)
T_5_alpha=glm.pi()/2
T_5_cos_alpha=glm.cos(T_5_alpha)
T_5_sin_alpha=glm.sin(T_5_alpha)

#DH parameters for 5_C_6 (placing body)
T_6_alpha=glm.pi()/2
T_6_cos_alpha=glm.cos(T_6_alpha)
T_6_sin_alpha=glm.sin(T_6_alpha)

T_6_a=METERS_TO_RENDER_SCALE*0.0091 #body length 


#Rotation Matrices to convert between the standard DH frames and the frame system of the 3D models
C_5_s=glm.mat4(glm.vec4(1,0,0,0),
               glm.vec4(0,0,1,0),
               glm.vec4(0,-1,0,0),
               glm.vec4(0,0,0,1))  #Transform between C5 and Cs (shaft frame)

C_6_b=glm.mat4(glm.vec4(1,0,0,0),
               glm.vec4(0,0,-1,0),
               glm.vec4(0,1,0,0),
               glm.vec4(0,0,0,1))  #Transform between C6 and Cb (body frame)

C_l_jl=glm.mat4(glm.vec4(1,0,0,0),
               glm.vec4(0,0,1,0),
               glm.vec4(0,-1,0,0),
               glm.vec4(0,0,0,1))  #Transform between Cjl and Cl (left jaw frame)

C_r_jr=glm.mat4(glm.vec4(1,0,0,0),
               glm.vec4(0,0,1,0),
               glm.vec4(0,-1,0,0),
               glm.vec4(0,0,0,1))  #Transform between Cjr and Cr (left jaw frame)

obj_names=['shaft','body','jaw_right','jaw_left']

tool_tip_offset=0.0102 #meters
tool_tip_translation=PyKDL.Vector(0,0,0.0102)


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



##############Numbering system to label corners for hand-eye calibration
CORNER_NUMBERS_STRING=['0','1','2','3','4','5','6','7','8','9','10','11','12','13','14','15']
MIN_NUMBER_FORHANDEYE=4 #Minimum Number of points for Hand-Eye Calibration

#Standalone function to take inverse of homogeneous transform with glm
def invHomogenous(transform):
    #Function to take inverse of homogenous transform
    R=glm.mat3(transform[0,0],transform[0,1],transform[0,2],
                transform[1,0],transform[1,1],transform[1,2],
                transform[2,0],transform[2,1],transform[2,2],)

    R_trans=glm.transpose(R)

    d=glm.vec3(transform[3,0],transform[3,1],transform[3,2])
    neg_R_trans_d=-R_trans*d
    inverted_transform=glm.mat4(glm.vec4(R_trans[0,0],R_trans[0,1],R_trans[0,2],0),
                                glm.vec4(R_trans[1,0],R_trans[1,1],R_trans[1,2],0),
                                glm.vec4(R_trans[2,0],R_trans[2,1],R_trans[2,2],0),
                                glm.vec4(neg_R_trans_d,1))
    return inverted_transform

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

        #####GUI Setup
        self.gui_window=tk.Tk()
        self.gui_window.title("Expert Playback App")

        self.gui_window.rowconfigure([0,1,2,3,4,5,6,7],weight=1)
        self.gui_window.columnconfigure([0,1,2],weight=1)


        #Button to start/top aruco tracking
        self.aruco_toggle_button=tk.Button(text="Start/Stop Aurco Tracking",width=20,command=self.arucoToggleCallback)
        self.aruco_toggle_button.grid(row=0,column=0,sticky="nsew")
        
        #Label to give number of points collected for hand-eye calibration
        self.calibrate_scene_text=tk.Label(text="Perform Hand-Eye-Scene Calibration with right hand (collect ~8 points with no camera movement)",width=100)
        self.calibrate_scene_text.grid(row=1,column=1,sticky='nsew')

        #Drop down to indicate which corner we are touching
        self.handeye_selected_corner=tk.StringVar()
        self.handeye_dropdown=tk.OptionMenu(self.gui_window,self.handeye_selected_corner,*CORNER_NUMBERS_STRING)
        self.handeye_dropdown.grid(row=2,column=0,sticky='nsew')

        #Button to capture points for hand-eye calibration
        self.handeye_point_button=tk.Button(text="Capture Point for Hand-Eye",width=20,command=self.capturePointCallback)
        self.handeye_point_button.grid(row=2,column=1,sticky="nsew")

        #Label to give number of points collected for hand-eye calibration
        self.calibrate_scene_text=tk.Label(text="# Collected = 0",width=50)
        self.calibrate_scene_text.grid(row=2,column=2,sticky='nsew')



        #Button to "calibrate" the scene and perform hand-eye calibration (a.k.a. find the scene) => Press when camera is stable
        self.calibrate_scene_button=tk.Button(text="Complete Hand-Eye and Scene Calibration",width=20,command=self.calibrateToggleCallback)
        self.calibrate_scene_button.grid(row=3,column=0,sticky="nsew")

       


        #Button to start/stop Rendering
        self.render_button=tk.Button(text="Start/Stop Rendering",width=20,command=self.renderButtonPressCallback)
        self.render_button.grid(row=4,column=0,sticky="nsew")

        #CheckButtons to Render PSM1 and PSM2 
        self.checkbox_PSM1=tk.Checkbutton(text="PSM1",width=10,onvalue=1,offvalue=0,command=self.psm1Checkbox)
        self.checkbox_PSM1.grid(row=4,column=1,sticky='nsew')

        self.checkbox_PSM3=tk.Checkbutton(text="PSM3",width=10,onvalue=1,offvalue=0,command=self.psm3Checkbox)
        self.checkbox_PSM3.grid(row=4,column=2,sticky='nsew')


        #Button to start ndi tracker
        self.ndi_toggle_button=tk.Button(text="Start NDI Tracker",width=20,command=self.ToggleTrackerCallback)
        self.ndi_toggle_button.grid(row=5,column=0,sticky="nsew")

        #Label to show that the tracker is running
        self.ndi_toggle_text=tk.Label(text="NDI Tracker Is Off",width=20)
        self.ndi_toggle_text.grid(row=5,column=1,sticky='nsew')

        #Button to start teleoperation
        self.teleop_button=tk.Button(text="Start/Stop Teleoperation",width=20,command=self.teleopButtonCallback)
        self.teleop_button.grid(row=6,column=0,sticky="nsew")

        self.teleop_lockout_checkbox=tk.Checkbutton(text="Lock Wrist",width=10,onvalue=1,offvalue=0,command=self.teleopLockoutCallback)
        self.teleop_lockout_checkbox.grid(row=6,column=1,sticky='nsew')

        #Button to run ndi tracking validation
        #self.validation_start=tk.Button(text="Run NDI Validation",width=20,command=self.ValidationCallback)
        #self.validation_start.grid(row=4,column=0,sticky="nsew")


        #Label for validation
        #self.validation_text=tk.Label(text="",width=20)
        #self.validation_text.grid(row=4,column=1,sticky='nsew')

        self.render_on=False
        self.aruco_on=False #Whether we show and update aruco poses
        self.calibrate_on=False
        self.capture_point_toggle=False

        self.teleop_on=False
        self.teleop_init=False #Checks whether teleoperation has started

        self.teleop_folow_init=False
        self.teleop_lockout=False #Whether we lockout wrist or not

        self.clutch_val=0
        self.camera_val=0


        self.num_points_capture=0 #The number of points we have captured for hand-eye-scene calibration


        #Params for validating the calibration
        self.NDI_TrackerToggle=False #Whether the NDI tracker is on and we want to validate, validation is started with the calibration button
        self.validation_trial_count=None
        self.csv_name=None
        #self.Pose_Validation=False

        #Selecting PSM buttons
        self.PSM1_on=False
        self.PSM3_on=False


        ####Aruco Tracking Setup
        self.aruco_tracker_left=ArucoTracker.ArucoTracker(self,'left')
        self.aruco_tracker_right=ArucoTracker.ArucoTracker(self,'right')


        #################dVRK API Config###################
        self.psm1=dvrk.psm("PSM1")#Mapped to left hand
        self.psm3=dvrk.psm("PSM3") #Mapped to right hand
        self.mtml=dvrk.mtm("MTML")
        self.mtmr=dvrk.mtm("MTMR")

        #Enabling and Homing
        self.psm1.enable()
        self.psm1.home()

        self.psm3.enable()
        self.psm3.home()

        self.mtml=dvrk.mtm("MTML")
        self.mtmr=dvrk.mtm("MTMR")

        self.mtml.enable()
        self.mtml.home()
        self.mtmr.enable()
        self.mtmr.home()


        #################Frame Transformations#####################
        
        self.world_base=glm.mat4() #This is the world base frame (set to be the corner of scene)
        self.si_T_lci=None #The left camera w.r.t. scene base frame (calibrated origin)
        self.si_T_rci=None #The right camera w.r.t. scene base frame (calibrated origin)


        #self.lci_T_si=None #Scene base frame w.r.t. to left camera
        #self.rvec_scene=None #How PnP returns rotation (for printing coordinate to scene)
        #self.tvec_scene=None #How PnP returns translation (for printing coordinate to scene)

        self.lc_T_e=None #Hand-eye calibration for left camera to endoscope
        self.rc_T_e=None #Hand-eye calibraiton for right camera to endoscope

        self.si_T_e=None #World base to endoscope base

        self.ei_T_e=glm.mat4() #Transform between initial endoscope position and current position

        #Camera pose:
        self.cam_left_pose=None
        self.cam_right_pose=None

        ####Teleoperation Transforms:
        self.orientation_offset_mtml=None
        self.orientation_offset_mtmr=None

        #Used at the start of follow mode
        self.display_T_mtml_ini=None
        self.display_T_mtmr_ini=None
        self.ecm_T_psm1_ini=None
        self.ecm_T_psm3_ini=None


        #Object Points for Hand-Eye Calibration
        self.corner_points_handeye=[] #By adding this to initial corner is essentially the translation
        self.robot_points_handeye=[]

        
    def teleopLockoutCallback(self):
        self.teleop_lockout=not self.teleop_lockout
    def teleopButtonCallback(self):
        self.teleop_on=not self.teleop_on
        self.teleop_init=False

    def capturePointCallback(self):
        self.capture_point_toggle=not self.capture_point_toggle
    def psm1Checkbox(self):
        self.PSM1_on=not self.PSM1_on

    def psm3Checkbox(self):
        self.PSM3_on=not self.PSM3_on


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
    '''
    def ValidationCallback(self):
        #Initialize the csv if it does not exist
        if not os.path.isfile(PATH_TO_VALIDATION_CSV):
            self.validation_trial_count=1
            with open(PATH_TO_VALIDATION_CSV,'w',newline='') as file_object:
                writer_object=csv.writer(file_object)
                writer_object.writerow(["Timestamp","Trial #","NDI_Tracker",\
                                        "Tx","Ty","Tz","Q0","Qx","Qy","Qz","Tracking Quality",\
                                        "Visual_Tracking","Tx","Ty","Tz","Q0","Qx","Qy","Qz","Tracking Quality"])
        else:
            self.validation_trial_count+=1
            with open(PATH_TO_VALIDATION_CSV,'a',newline='') as file_object:
                writer_object=csv.writer(file_object)
                writer_object.writerow("\n")

        
        self.Pose_Validation=not self.Pose_Validation
        self.validation_text.config(text='Validation Started')
    '''

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


    def arucoToggleCallback(self):
        self.aruco_on=not self.aruco_on
    
    def renderButtonPressCallback(self):
        self.render_on=not self.render_on

    def clutchPedalCallback(self,data):
        if self.teleop_on:
            self.clutch_val=int(data.buttons[0])

            print("Clutch Val: "+str(self.clutch_val))

    def cameraPedalCallback(self,data):
        if self.teleop_on:
            self.camera_val=int(data.buttons[0])
            print("Camera Val: "+str(self.camera_val))
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
        #print("Key Pressed")
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
    def enforceOrthogonalPyKDL(self,pykdl_frame):
        pykdl_frame_new=pm.toMatrix(pykdl_frame)
        pykdl_frame_new[0:3,0:3]=HandEye.EnforceOrthogonality(pykdl_frame_new[0:3,0:3])
        pykdl_frame_new=pm.fromMatrix(pykdl_frame_new)
        return pykdl_frame_new

    def render(self,dt):

        #################Teleoperation#####################

        if self.teleop_on:  #Teleoperation On

            #############Aligning the MTMs############
            if not self.teleop_init: 

                self.teleop_init=True
                #We set position of MTML equal to its position, and the orientation of MTML=PSM1
                #We set position of MTMR equal to its position, and the orientation of MTMR=PSM3

                #PSMs w.r.t. ECMs
                ecm_T_psm1=self.psm1.setpoint_cp()

                ecm_T_psm3=self.psm3.setpoint_cp()

                #MTMs w.r.t. display

                display_T_mtml=self.mtml.measured_cp()
                display_T_mtmr=self.mtmr.measured_cp()


                #Moving the MTMs 

                ecm_T_psm1=self.enforceOrthogonalPyKDL(ecm_T_psm1)
                ecm_T_psm3=self.enforceOrthogonalPyKDL(ecm_T_psm3)


                display_T_mtml.M=ecm_T_psm1.M
                display_T_mtmr.M=ecm_T_psm3.M

                self.mtml.move_cp(display_T_mtml).wait()
                self.mtmr.move_cp(display_T_mtmr).wait()

                self.orientation_offset_mtml=self.mtml.measured_cp().M.Inverse()*ecm_T_psm1.M                
                self.orientation_offset_mtmr=self.mtmr.measured_cp().M.Inverse()*ecm_T_psm3.M
                print("Initialized Teleoperation")
                #alignment_offset_angle, _ = self.orientation_offset_mtml.GetRotAngle()

                #print("offset angle: "+str(alignment_offset_angle * (180/np.pi)))
            else: 
                
                if self.clutch_val==1:
                    ########Clutch Mode############
                    zero_wrench=np.array([0.0,0.0,0.0,0.0,0.0,0.0])
                    self.mtml.body.servo_cf(zero_wrench)
                    self.mtmr.body.servo_cf(zero_wrench)
                    print("Entered Clutching")
                    '''
                    ecm_T_psm1=self.psm1.setpoint_cp()
                    ecm_T_psm3=self.psm3.setpoint_cp()
                    display_T_mtml=self.mtml.measured_cp()
                    display_T_mtmr=self.mtmr.measured_cp()
                    ecm_T_psm1=self.enforceOrthogonalPyKDL(ecm_T_psm1)
                    ecm_T_psm3=self.enforceOrthogonalPyKDL(ecm_T_psm3)

                    display_T_mtml.M=ecm_T_psm1.M*self.orientation_offset_mtml.Inverse()
                    display_T_mtmr.M=ecm_T_psm3.M*self.orientation_offset_mtmr.Inverse()

                    self.mtml.move_cp(display_T_mtml).wait()
                    self.mtmr.move_cp(display_T_mtmr).wait()
                    '''                
                else:


                    #########Follow Mode###########

                    if not self.teleop_folow_init: #We are starting follow mode
                        self.teleop_folow_init=True
                        
                        #Set gravity compensation and "free" the joints:
                        zero_wrench=np.array([0.0,0.0,0.0,0.0,0.0,0.0])
                        self.mtml.use_gravity_compensation(True)
                        self.mtmr.use_gravity_compensation(True)
                        self.mtml.body.servo_cf(zero_wrench)
                        self.mtmr.body.servo_cf(zero_wrench)

                        #Get the MTM and PSM pose
                        self.display_T_mtml_ini=self.mtml.measured_cp()
                        self.display_T_mtmr_ini=self.mtmr.measured_cp()
                        self.ecm_T_psm1_ini=self.psm1.setpoint_cp()
                        self.ecm_T_psm3_ini=self.psm3.setpoint_cp()
                        print("Folow Mode Init Done")
                    else:
                        #Follow mode on each iteration
                        display_T_mtml_new=self.mtml.setpoint_cp()
                        display_T_mtmr_new=self.mtmr.setpoint_cp()

                        mtml_jaw_angle=self.mtml.gripper.measured_js()
                        mtmr_jaw_angle=self.mtmr.gripper.measured_js()


                        ecm_T_psm1_next=self.ecm_T_psm1_ini
                        ecm_T_psm3_next=self.ecm_T_psm3_ini

                        mtmlInit_T_mtmlNew=display_T_mtml_new
                        mtmrInit_T_mtmrNew=display_T_mtmr_new

                        #Orientation Control
                        #How much we have moved the mtm's orientation
                        mtmlInit_T_mtmlNew.M=display_T_mtml_new.M*(self.display_T_mtml_ini.M.Inverse())
                        mtmrInit_T_mtmrNew.M=display_T_mtmr_new.M*(self.display_T_mtmr_ini.M.Inverse())
                        test=pm.Quaternion()
                        
                        #mtmlInit_T_mtmlNew.M=(self.display_T_mtml_ini.M.Inverse())*display_T_mtml_new.M
                        #mtmrInit_T_mtmrNew.M=(self.display_T_mtmr_ini.M.Inverse())*display_T_mtmr_new.M

                        rot_angle, _ = mtmlInit_T_mtmlNew.M.GetRotAngle()
                        print("Rotation Change " + str(rot_angle))

                        ecm_T_psm1_next.M=mtmlInit_T_mtmlNew.M*self.ecm_T_psm1_ini.M #*self.orientation_offset_mtml
                        ecm_T_psm3_next.M=mtmrInit_T_mtmrNew.M*self.ecm_T_psm3_ini.M #*self.orientation_offset_mtmr
                        #ecm_T_psm1_next.M=display_T_mtml_new.M*self.orientation_offset_mtml
                        #ecm_T_psm3_next.M=display_T_mtmr_new.M*self.orientation_offset_mtmr
                        #Position Control
                        mtml_translation=display_T_mtml_new.p-self.display_T_mtml_ini.p
                        mtmr_translation=display_T_mtmr_new.p-self.display_T_mtmr_ini.p


                        ecm_T_psm1_next.p=PSM_SCALE_FACTOR*mtml_translation+self.ecm_T_psm1_ini.p
                        ecm_T_psm3_next.p=PSM_SCALE_FACTOR*mtmr_translation+self.ecm_T_psm3_ini.p

                        #Moving 
                        #ecm_T_psm1_next=self.enforceOrthogonalPyKDL(ecm_T_psm1_next)
                        #ecm_T_psm3_next=self.enforceOrthogonalPyKDL(ecm_T_psm3_next)
                        self.psm1.servo_cp(ecm_T_psm1_next)
                        self.psm1.jaw.servo_jp(np.array(mtml_jaw_angle[0]))
                        self.psm3.servo_cp(ecm_T_psm3_next)
                        self.psm3.jaw.servo_jp(np.array(mtmr_jaw_angle[0]))
                        print("Folow Mode")
                        

                        #Updating mtml and psm poses
                        self.display_T_mtml_ini=display_T_mtml_new
                        self.display_T_mtmr_ini=display_T_mtmr_new

                        self.ecm_T_psm1_ini=ecm_T_psm1_next
                        self.ecm_T_psm3_ini=ecm_T_psm3_next
                        #rospy.sleep(0.01)
                        







        self.move_rads+=0.01

        #Get Time
        self.get_time() 

        ################Move Instruments
        if self.PSM1_on:
            #PSM1:
            self.instrument_kinematics([glm.pi()/6,glm.pi()/6,glm.pi()/6,self.move_rads],start_pose,'PSM1')
        if self.PSM3_on:
            #PSM3:
            self.instrument_kinematics([glm.pi()/3,glm.pi()/3,glm.pi()/3,self.move_rads*2],start_pose2,'PSM3')

        self.move_objects() 
        


        ####Render Left Window
        self.window_left.switch_to()       
        self.ctx_left.clear()

        #Updating Background Image
        if self.frame_left is not None:
            self.ctx_left.disable(mgl.DEPTH_TEST)
            #background_image_left=Image.open('textures/Sunflower.jpg').transpose(Image.FLIP_TOP_BOTTOM)
            
            if self.aruco_on:
                self.frame_left_converted=self.aruco_tracker_left.arucoTrackingScene(self.frame_left)

                if self.capture_point_toggle: 
                    self.capture_point_toggle=False
                    corner_number=int(self.handeye_selected_corner.get())
                    corner_point=self.aruco_tracker_left.returnObjectPointForHandeye(corner_number)
                    if corner_point:
                        self.num_points_capture+=1
                        self.corner_points_handeye.append(corner_point)   
                        #Get point of PSM1 from da Vinci API
                        tool_pose=self.psm1.measured_cp()
                        tool_pose.p=tool_pose.p+tool_tip_translation
                        tool_point=[tool_pose.p.x(),tool_pose.p.y(),tool_pose.p.z()]
                        self.robot_points_handeye.append(tool_point)

                        print("tool point: "+str(tool_pose.p))
                        
                        print("corner point list: "+str(self.corner_points_handeye))
                        print("tool point list: "+str(self.robot_points_handeye))


                        self.calibrate_scene_text.config(text="# Collected = "+str(self.num_points_capture))
                    else:
                        print("Requested Corner Not In View")
                    #Capture Points for Hand-Eye-Scene Calibration (capture point in endoscope coord system and in scene coord system)
                    #These points are for both left and right cameras (doesn't matter because it is for robot)

                
                if self.calibrate_on and (not self.aruco_tracker_left.calibrate_done): #Sets Scene Base Frame
                    if self.num_points_capture>=MIN_NUMBER_FORHANDEYE:
                        
                        self.si_T_lci=None
                        self.aruco_tracker_left.calibrateScene()
                        self.si_T_lci=self.aruco_tracker_left.si_T_ci

                        #Writing to CSV For Validation
                        if self.NDI_TrackerToggle:
                            NDI_dat=self.ndi_tracker.get_frame() #Grabs the NDI tracker data
                            self.writeToNDIVal(NDI_dat)
                    else:
                        print("Not Enough Points for Hand-Eye-Scene Calibration, collect more")
      
                if self.si_T_lci is not None: #We Perform Hand-Eye Calibration and show the scene coord system for left camera
                    
                    #First find scene to endoscope base transform (si_T_e) by solving a RANSAC registration
                    src=np.array(self.corner_points_handeye)
                    dst=np.array(self.robot_points_handeye)

                    model_robust,inliers=skimage.measure.ransac((src,dst),skimage.transform.AffineTransform,\
                                                                min_samples=3,residual_threshold=2,max_trials=100)
                    
                    
                    
                    cv2.drawFrameAxes(self.frame_left_converted,self.aruco_tracker_left.mtx,\
                                      self.aruco_tracker_left.dist,self.aruco_tracker_left.rvec_scene,self.aruco_tracker_left.tvec_scene,0.05)
                  
                self.frame_left_converted=self.cvFrame2Gl(self.frame_left_converted)
            
            else:
                self.frame_left_converted=self.cvFrame2Gl(self.frame_left)



            self.texture_left.write(self.frame_left_converted)
            self.texture_left.use()
            self.vertex_array_left.render()
            self.ctx_left.enable(mgl.DEPTH_TEST)

        #Rendering Left Instruments
        if self.render_on:
            self.camera_left.update(None) 
            #self.cam_left_pose=self.world_base*self.si_T_lci*self.lc_T_e*self.ei_T_e*glm.transpose(self.lc_T_e)
            #self.camera_left.update(self.cam_left_pose)
            #self.scene.render(self.ctx_left)
            if self.PSM1_on:
                self.scene_PSM1.render(self.ctx_left)
            if self.PSM3_on:
                self.scene_PSM3.render(self.ctx_left)
                
        
        #########Render the Right Window
        self.window_right.switch_to()
        self.ctx_right.clear()

        #Updating Background Image
        if self.frame_right is not None:
            self.ctx_right.disable(mgl.DEPTH_TEST)
            
            if self.aruco_on:
                self.frame_right_converted=self.aruco_tracker_right.arucoTrackingScene(self.frame_right)                
                if self.calibrate_on and (not self.aruco_tracker_right.calibrate_done): #Sets Scene Base Frame
                    if self.num_points_capture>=MIN_NUMBER_FORHANDEYE:
                        self.si_T_rci=None
                        self.aruco_tracker_right.calibrateScene()
                        self.si_T_rci=self.aruco_tracker_right.si_T_ci

                    else:
                        print("Not Enough Points for Hand-Eye-Scene Calibration, collect more")
      
                if (self.si_T_rci is not None) and (self.si_T_rci is not None): #We Perform Hand-Eye Calibration and show the scene coord system for right camera
                    
                    
                    
                    
                    
                    cv2.drawFrameAxes(self.frame_right_converted,self.aruco_tracker_right.mtx,\
                                      self.aruco_tracker_right.dist,self.aruco_tracker_right.rvec_scene,self.aruco_tracker_right.tvec_scene,0.05) #Look at rvec and tvec, look at frame_right_converted, look at calibrate on
                  
                self.frame_right_converted=self.cvFrame2Gl(self.frame_right_converted)
            
            else:
                self.frame_right_converted=self.cvFrame2Gl(self.frame_right)


            self.texture_right.write(self.frame_right_converted)
            self.texture_right.use()
            self.vertex_array_right.render()
            self.ctx_right.enable(mgl.DEPTH_TEST)

        if self.render_on:
            self.camera_right.update(None)
            #self.cam_right_pose=self.world_base*self.si_T_rci*self.rc_T_e*self.ei_T_e*glm.transpose(self.rc_T_e)
            #                            self.camera_left.update(self.cam_left_pose)
            #self.scene.render(self.ctx_right)
            if self.PSM1_on:
                self.scene_PSM1.render(self.ctx_right)
            if self.PSM3_on:
                self.scene_PSM3.render(self.ctx_right)

        
        #Resestting hand-eye variables:
        if self.aruco_tracker_right.calibrate_done and self.aruco_tracker_left.calibrate_done:
            self.num_points_capture=0
            self.calibrate_scene_text.config(text="# Collected = "+str(self.num_points_capture))
            self.corner_points_handeye=[]
            self.robot_points_handeye=[]
            self.aruco_tracker_left.calibrate_done=False
            self.aruco_tracker_right.calibrate_done=False

            
        ####Gui Updates

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

    def instrument_kinematics(self,joint_angles,T4,PSM_Type):
        '''
        Input: 
        - list of angles of the intrument joints: shaft rotation (q5), body rotation (q6), jaw rotation (q7), jaw seperation (theta_j)
        - Shaft Base Frame (T4)
        Output: 
        - Frames for each instrument segment: shaft frame (Ts), body frame (Tb), left jaw (Tl), right jaw (Tr)  

        !!!! Note, input "shaft base" should be a 4x4 frame matrix (orientation and pose)
        Also note, T4 is the base of the instrument, and T5 is the shaft rotated frame, T6 is the body rotated frame, T7 is the jaw base (no seperation)
        '''
        #Enforce >=0 jaw angle
        if joint_angles[3]<0:
            joint_angles[3]=0


        #First get frames by applying forward kinematics using conventional DH coordinates frames
        T5=T4*self.transform_4_T_5(joint_angles[0])
        T6=T5*self.transform_5_T_6(joint_angles[1])
        T7=T6*self.transform_6_T_7(joint_angles[2])
        Tjl=T7*self.Rotz(-joint_angles[3]/2)
        Tjr=T7*self.Rotz(joint_angles[3]/2)

        #Next we convert these standard coordinates to the coordinates of our 3D objects (these are augmented)
        
        Ts=T5*C_5_s
        Tb=glm.translate(T6,glm.vec3(-T_6_a,0,0))*C_6_b #Brings back body frame to base of 3D model        
        Tl=Tjl*C_l_jl
        Tr=Tjr*C_r_jr
        if PSM_Type=='PSM1':
            self.instrument_dict_PSM1={
                'shaft': Ts,
                'body': Tb,
                'jaw_right': Tr,
                'jaw_left':Tl
            }

        elif PSM_Type=='PSM3':
            self.instrument_dict_PSM3={
                'shaft': Ts,
                'body': Tb,
                'jaw_right': Tr,
                'jaw_left':Tl
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

    def frameCallbackRight(self,data):
        self.frame_right=self.bridge.compressed_imgmsg_to_cv2(data,'passthrough')

    def frameCallbackLeft(self,data):
        
        self.frame_left=self.bridge.compressed_imgmsg_to_cv2(data,'passthrough')
        #cv2.imshow('Frame Callback',self.frame_left)
        #cv2.waitKey(1)


    
    ############ Main Render Loop
    def run(self,delay=100):
        #self.get_time()
        #self.check_events()
        #self.instrument_kinematics([glm.pi()/6,glm.pi()/6,glm.pi()/6,glm.pi()/6],start_pose)
        #self.move_objects()
        #self.camera.update()
        #self.render()
        pyglet.clock.schedule_interval(self.render,1/delay)
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