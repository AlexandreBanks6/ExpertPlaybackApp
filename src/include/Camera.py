import glm
import pygame as pg
from pyglet.window import key
#from glfw.GLFW import *
#from glfw import _GLFWwindow as GLFWwindow

FOV=50
NEAR=0.1
FAR=1000
SPEED=0.01
SENSITIVITY=0.05 #Mouse sensitivity for orientation changes

class Camera:
    def __init__(self,app,position=(0,0,4),yaw=-90,pitch=0):
        self.app=app
        self.aspect_ratio=app.WIN_SIZE[0]/app.WIN_SIZE[1]
        #Setting up Camera position
        self.position=glm.vec3(position)
        self.up=glm.vec3(0,1,0)
        self.right=glm.vec3(1,0,0)
        self.forward=glm.vec3(0,0,-1)
        self.yaw=yaw
        self.pitch=pitch
        self.mouse_x=1
        self.mouse_y=1

        #View Matrix
        self.m_view=self.get_view_matrix()

        #Projection matrix
        self.m_proj=self.get_projection_matrix()
    def rotate(self):
        #self.yaw+=self.mouse_x*SENSITIVITY
        #self.pitch-=self.mouse_y*SENSITIVITY
        self.pitch=max(-89,min(89,self.pitch))

    def update_camera_vectors(self):
        yaw,pitch=glm.radians(self.yaw),glm.radians(self.pitch)

        self.forward.x=glm.cos(yaw)*glm.cos(pitch)

        self.forward.y=glm.sin(pitch)

        self.forward.z=glm.sin(yaw)*glm.cos(pitch)

        self.forward=glm.normalize(self.forward)
        self.right=glm.normalize(glm.cross(self.forward,glm.vec3(0,1,0)))
        self.up=glm.normalize(glm.cross(self.right,self.forward))
    def update(self):
        self.move()
        self.rotate()
        self.update_camera_vectors()
        self.m_view=self.get_view_matrix()

    def move(self):
        velocity=SPEED*self.app.delta_time
        print("Move Entered")
        keys=self.app.keys
        
        if keys[key.W]:
            self.position+=self.forward*velocity
        if keys[key.S]:
            self.position-=self.forward*velocity
        if keys[key.A]:
            self.position+=self.right*velocity
        if keys[key.D]:
            self.position-=self.right*velocity
        if keys[key.Q]:
            self.position+=self.up*velocity
        if keys[key.E]:
            self.position-=self.up*velocity


    '''
    def move(self):
        velocity=SPEED*self.app.delta_time
        keys=pg.key.get_pressed()

        if keys[pg.K_w]:
            self.position+=self.forward*velocity
        if keys[pg.K_s]:
            self.position-=self.forward*velocity
        if keys[pg.K_a]:
            self.position+=self.right*velocity
        if keys[pg.K_d]:
            self.position-=self.right*velocity
        if keys[pg.K_q]:
            self.position+=self.up*velocity
        if keys[pg.K_e]:
            self.position-=self.up*velocity
    '''
    

    '''
    def move(self):
        velocity=SPEED*self.app.delta_time
        #keys=glfwGetKey(self.app.window1)

        if glfwGetKey(self.app.window1,GLFW_KEY_W)==GLFW_PRESS:
            self.position+=self.forward*velocity
        if glfwGetKey(self.app.window1,GLFW_KEY_S)==GLFW_PRESS:
            self.position-=self.forward*velocity
        if glfwGetKey(self.app.window1,GLFW_KEY_A)==GLFW_PRESS:
            self.position+=self.right*velocity
        if glfwGetKey(self.app.window1,GLFW_KEY_D)==GLFW_PRESS:
            self.position-=self.right*velocity
        if glfwGetKey(self.app.window1,GLFW_KEY_Q)==GLFW_PRESS:
            self.position+=self.up*velocity
        if glfwGetKey(self.app.window1,GLFW_KEY_E)==GLFW_PRESS:
            self.position-=self.up*velocity
    '''

    def get_view_matrix(self):
        return glm.lookAt(self.position,self.position+self.forward,self.up)

    def get_projection_matrix(self):
        return glm.perspective(glm.radians(FOV),self.aspect_ratio,NEAR,FAR) #Returns the perspective matrix