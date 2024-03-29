import glm
#from glfw.GLFW import *
#from glfw import _GLFWwindow as GLFWwindow

FOV=50
NEAR=0.1
FAR=1000
SPEED=100
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
        self.mouse_x=0
        self.mouse_y=0
        self.old_mouse_x=0
        self.old_mouse_y=0

        #View Matrixc
        self.m_view=self.get_view_matrix()

        #Projection matrix
        self.m_proj=self.get_projection_matrix()
    def rotate(self):
        new_mouse_x=self.mouse_x-self.app.WIN_SIZE[0]/2
        new_mouse_y=self.mouse_y-self.app.WIN_SIZE[1]/2
        mouse_dif_x=new_mouse_x-self.old_mouse_x
        mouse_dif_y=new_mouse_y-self.old_mouse_y
        self.yaw+=mouse_dif_x*SENSITIVITY
        self.pitch+=mouse_dif_y*SENSITIVITY
        self.pitch=max(-89,min(89,self.pitch))
        self.old_mouse_x=new_mouse_x
        self.old_mouse_y=new_mouse_y

    def update_camera_vectors(self):
        yaw,pitch=glm.radians(self.yaw),glm.radians(self.pitch)

        self.forward.x=glm.cos(yaw)*glm.cos(pitch)

        self.forward.y=glm.sin(pitch)

        self.forward.z=glm.sin(yaw)*glm.cos(pitch)

        self.forward=glm.normalize(self.forward)
        self.right=glm.normalize(glm.cross(self.forward,glm.vec3(0,1,0)))
        self.up=glm.normalize(glm.cross(self.right,self.forward))
    def update(self,camera_pos):
        #input: camera_pos is a glm mat of the pose of the camera w.r.t. scene "world" origin
        if camera_pos is None: #If we want to use mouse control over camera
            self.move()
            self.rotate()
            self.update_camera_vectors()
            self.m_view=self.get_view_matrix()
        elif camera_pos is not None:    #if we want automatic camera control (playback)
            #Convert camera_pose to opengl view matrix pose ([i_gl,j_gl,k_gl]=[i0,-j0,-k0])

            #Negate second column of camera pos for view matrix (j)
            camera_pos[0][1]=-camera_pos[0][1]
            camera_pos[1][1]=-camera_pos[1][1]
            camera_pos[2][1]=-camera_pos[2][1]

            #Negate third column of camera pos for view matrix (k)
            camera_pos[0][2]=-camera_pos[0][2]
            camera_pos[1][2]=-camera_pos[1][2]
            camera_pos[2][2]=-camera_pos[2][2]
            self.m_view=camera_pos



    def move(self):
        velocity=SPEED*self.app.delta_time
        #print(self.app.delta_time)
        key_dict=self.app.key_dict
        if key_dict['W']:
            self.position+=self.forward*velocity
        if key_dict['S']:
            self.position-=self.forward*velocity
        if key_dict['A']:
            self.position+=self.right*velocity
        if key_dict['D']:
            self.position-=self.right*velocity
        if key_dict['Q']:
            self.position+=self.up*velocity
        if key_dict['E']:
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