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

test_point_scene=glm.vec4(ARUCO_SIDELENGTH+ARUCO_SEPERATION,ARUCO_SEPERATION+ARUCO_SIDELENGTH,2*ARUCO_HEIGHT_OFFSET,1)


RightFrame_Topic='ubc_dVRK_ECM/right/decklink/camera/image_raw/compressed'
LeftFrame_Topic='ubc_dVRK_ECM/left/decklink/camera/image_raw/compressed'

CONSOLE_VIEWPORT_WIDTH=1400
CONSOLE_VIEWPORT_HEIGHT=986

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


        #############Right Window Initialization
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


        #Frames passed by endoscope
        self.frame_right=None #Updated by the callback in rospy
        self.frame_left=None
        self.frame_left_converted=None
        self.frame_right_converted=None

        self.bridge=CvBridge()


        ##############GUI SETUP################
        self.gui_window=tk.Tk()
        self.gui_window.title("dVRK Playback App")

        self.gui_window.rowconfigure([0],weight=1)
        self.gui_window.columnconfigure([0,1,2],weight=1)
        self.gui_window.minsize(300,100)

        
        
        self.calibrate_scene_button=tk.Button(self.gui_window,text="Calibrate Scene",command=self.calibrateToggleCallback)
        self.calibrate_scene_button.grid(row=0,column=0,sticky="nsew")

        self.validate_calibration_button=tk.Button(self.gui_window,text="Validate Calibration",command=self.validateCalibrationCallback)
        self.validate_calibration_button.grid(row=0,column=1,sticky="nsew")

        self.validate_handeye_button=tk.Button(self.gui_window,text="Validate Hand-Eye",command=self.validateHandEyeCallback)
        self.validate_calibration_button.grid(row=0,column=2,sticky="nsew")



        self.aruco_tracker_left=ArucoTracker.ArucoTracker(self,'left')
        self.aruco_tracker_right=ArucoTracker.ArucoTracker(self,'right')

        self.calibrate_on=False
        self.validate_HandEye_on=False

        self.validate_calibration_on=False
        self.validate_calibration_on=False

        self.lci_T_si=None
        self.rci_T_si=None

        self.psm1=dvrk.psm("PSM1") #Mapped to left hand
        self.psm3=dvrk.psm("PSM3") #Mapped to right hand

        #Enabling and Homing
        self.psm1.enable()
        self.psm1.home()

        self.psm3.enable()
        self.psm3.home()


        rospy.sleep(1)

    def validateHandEyeCallback(self):
        self.validate_HandEye_on=True

    def calibrateToggleCallback(self):
        self.calibrate_on=not self.calibrate_on
        self.aruco_tracker_left.calibrate_done=False
        self.aruco_tracker_left.corner_scene_list=[]
        self.aruco_tracker_left.ids_scene_list=[]

        self.aruco_tracker_right.calibrate_done=False
        self.aruco_tracker_right.corner_scene_list=[]
        self.aruco_tracker_right.ids_scene_list=[]

    def validateCalibrationCallback(self):
        self.validate_calibration_on=True



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
                        print("test point: "+str(test_point_scene))
                        # print("test_point_scene: "+str(test_point_scene))
                        print("mul_mat: "+str(mul_mat))
                        print("cam_mat: "+str(cam_mat))
                        print("proj_point"+str(proj_point))


                        #Project with the project points function
                        # test_point_scene_np=np.array([[0.025537,0.0,0.0]],dtype=np.float32)
                        # image_points,_=cv2.projectPoints(test_point_scene_np,self.aruco_tracker_left.rvec_scene,self.aruco_tracker_left.tvec_scene,self.aruco_tracker_left.mtx,self.aruco_tracker_left.dist)
                        
                        # proj_point_np=tuple((int(image_points[0][0][0]),int(image_points[0][0][1])))
                        #self.frame_left_converted=cv2.circle(self.frame_left_converted,proj_point_np,radius=10,color=(0,0,255),thickness=3)





                    
                self.frame_left_converted=self.cvFrame2Gl(self.frame_left_converted)
                

            else:
                self.frame_left_converted=self.cvFrame2Gl(self.frame_left)
                

            self.texture_left.write(self.frame_left_converted)
            self.texture_left.use()
            self.vertex_array_left.render()
            self.ctx_left.enable(mgl.DEPTH_TEST)






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

            else:
                self.frame_right_converted=self.cvFrame2Gl(self.frame_right)



            self.texture_right.write(self.frame_right_converted)
            self.texture_right.use()
            self.vertex_array_right.render()
            self.ctx_right.enable(mgl.DEPTH_TEST)

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