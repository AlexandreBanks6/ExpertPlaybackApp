import pygame as pg

#Use pyglet instead of pygame as we can create multiple windows
import pyglet

import moderngl as mgl
import sys
from include import Model as mdl
from include import Camera
from include import Light
from include import mesh
from include import scene
import glm


import math
import numpy as np

CONSOLE_VIEWPORT_WIDTH=1400
CONSOLE_VIEWPORT_HEIGHT=986

MOVE_SPEED=0.5
METERS_TO_RENDER_SCALE=1000

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

###For Testing:
start_pose=glm.mat4(glm.vec4(1,0,0,0),
                glm.vec4(0,1,0,0),
                glm.vec4(0,0,1,0),
                glm.vec4(0,0,-10,1))


##################Init Parameters for Grabbing Frames###################
RightFrame_Topic='ubc_dVRK_ECM/right/decklink/camera/image_raw/compressed'
LeftFrame_Topic='ubc_dVRK_ECM/left/decklink/camera/image_raw/compressed'

class Renderer:
    def __init__(self,win_size=(CONSOLE_VIEWPORT_WIDTH,CONSOLE_VIEWPORT_HEIGHT)):
        '''
        #Init pygame modules
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
        window=pyglet.window.Window(width=CONSOLE_VIEWPORT_WIDTH,heigh=CONSOLE_VIEWPORT_HEIGHT)

        #Detect and use opengl context
        self.ctx=mgl.create_context()


        #Enable Depth Testing 
        self.ctx.enable(flags=mgl.DEPTH_TEST|mgl.CULL_FACE) #CULL_FACE does not render invisible faces

     
        #Object to track time
        #self.clock=pg.time.Clock()
        
        self.time=0
        self.delta_time=0

        #Initialize the instrument frames
        self.instrument_dict=None
        self.instrument_kinematics([0,0,0,0],start_pose)

        #Lighting instance
        self.light=Light.Light()

        #Create an instance of the camera clas
        self.camera=Camera.Camera(self)

        #Mesh instance:
        self.mesh=mesh.Mesh(self)

        #Scene
        self.scene=scene.Scene(self)

    def check_events(self):
        for event in pg.event.get():
            if event.type==pg.QUIT or (event.type==pg.KEYDOWN and event.key==pg.K_ESCAPE):
                self.mesh.destroy()
                pg.quit()
                sys.exit()

    def render(self):
        #This method renders the screen

        self.ctx.clear(color=(0.08,0.16,0.18))

        #Render the scene
        self.scene.render()
        #Swap the buffers
        pg.display.flip()
    def get_time(self):
        #Time in seconds (float)
        self.time=pg.time.get_ticks()*0.001

    ########### Methods to position instrument/camera
    def move_objects(self):
        #This is the method where we pass the matrix to move_obj to move the object and also 
        #select which object to move: "shaft", "body","jaw_right","jaw_left"

        for obj_name in obj_names:
            #if obj_name=='jaw_right' or obj_name=='jaw_left':
             #   continue
            #print(obj_name)
            move_mat=self.instrument_dict[obj_name]

            self.scene.move_obj(obj_name,move_mat)

    def instrument_kinematics(self,joint_angles,C4):
        '''
        Input: 
        - list of angles of the intrument joints: shaft rotation (q5), body rotation (q6), jaw rotation (q7), jaw seperation (theta_j)
        - Shaft Base Frame (C4)
        Output: 
        - Frames for each instrument segment: shaft frame (Cs), body frame (Cb), left jaw (Cl), right jaw (Cr)  

        !!!! Note, input "shaft base" should be a 4x4 frame matrix (orientation and pose)
        Also note, C4 is the base of the instrument, and C5 is the shaft rotated frame, C6 is the body rotated frame, C7 is the jaw base (no seperation)
        '''
        #Enforce >=0 jaw angle
        if joint_angles[3]<0:
            joint_angles[3]=0


        #First get frames by applying forward kinematics using conventional DH coordinates frames
        C5=C4*self.transform_4_T_5(joint_angles[0])
        C6=C5*self.transform_5_T_6(joint_angles[1])
        C7=C6*self.transform_6_T_7(joint_angles[2])
        Cjl=C7*self.Rotz(-joint_angles[3]/2)
        Cjr=C7*self.Rotz(joint_angles[3]/2)

        #Next we convert these standard coordinates to the coordinates of our 3D objects (these are augmented)
        
        Cs=C5*C_5_s
        Cb=glm.translate(C6,glm.vec3(-T_6_a,0,0))*C_6_b #Brings back body frame to base of 3D model        
        Cl=Cjl*C_l_jl
        Cr=Cjr*C_r_jr

        self.instrument_dict={
            'shaft': Cs,
            'body': Cb,
            'jaw_right': Cr,
            'jaw_left':Cl
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
    def invHomogenous(self,transform):
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
    

    #def rightFrameGrabber(self):

    #def leftFrameGrabber(self):
    
    ############ Main Render Loop
    def updateWindow(self,delay=60):
        self.get_time()
        self.check_events()
        self.instrument_kinematics([glm.pi()/6,glm.pi()/6,glm.pi()/6,glm.pi()/6],start_pose)
        self.move_objects()
        self.camera.update()
        self.render()
        self.delta_time=self.clock.tick(delay)
        
        
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