import numpy as np
import glm
import pygame as pg

class BaseModel:
    def __init__(self,app,vao_name,tex_id,pos=(0,0,0),rot=(0,0,0),scale=(1,1,1)):
        self.app=app
        self.pos=pos
        self.rot=glm.vec3([glm.radians(a) for a in rot])
        self.scale=scale
        self.m_model=self.get_model_matrix()
        self.tex_id=tex_id
        #print(app.mesh.vao.vaos)
        self.vao=app.mesh.vao.vaos[vao_name]
        self.program=self.vao.program
        self.camera=self.app.camera
    def update(self):...

    def get_model_matrix(self):
        #Important that the order is translate, rotate, and then scale
        m_model=glm.mat4()

        #Translate
        m_model=glm.translate(m_model,self.pos)

        #Rotate
        #m_model=glm.rotate(m_model,self.rot.x,glm.vec3(1,0,0))
        #m_model=glm.rotate(m_model,self.rot.y,glm.vec3(0,1,0))
        #m_model=glm.rotate(m_model,self.rot.z,glm.vec3(0,0,1))

        #Scale
        #m_model=glm.scale(m_model,self.scale)
        return m_model
    def move(self,new_model):
        #Method to move objects in the world, essentially just set the model matrix
        self.m_model=new_model
    def render(self):
        self.update()
        self.vao.render()



class Cube(BaseModel):
    def __init__(self,app,vao_name='cube',tex_id=0,pos=(0,0,0),rot=(0,0,0),scale=(1,1,1)):
        super().__init__(app,vao_name,tex_id,pos,rot,scale)
        self.on_init()


    def update(self):
        self.texture.use()
        self.program['m_model'].write(self.m_model)
        self.program['m_view'].write(self.camera.m_view)
        self.program['camPos'].write(self.camera.position)

    
    def on_init(self):

        #Specify the name of the texture variable
        self.texture=self.app.mesh.texture.textures[self.tex_id]
        self.program['u_texture_0']=0
        self.texture.use()

        #Pass the projection matrix
        self.program['m_proj'].write(self.camera.m_proj)
        self.program['m_view'].write(self.camera.m_view)
        self.program['m_model'].write(self.m_model)

        #Light Source
        self.program['light.position'].write(self.app.light.position) #Light Position
        self.program['light.Ia'].write(self.app.light.Ia) #Ambient light component
        self.program['light.Id'].write(self.app.light.Id) #Diffuse Component
        self.program['light.Is'].write(self.app.light.Is) #Specular Intensity


class Shaft(BaseModel):
    def __init__(self,app,vao_name='shaft',tex_id='shaft',pos=(0,0,0),rot=(0,0,0),scale=(1,1,1)):
        super().__init__(app,vao_name,tex_id,pos,rot,scale)
        self.on_init()


    def update(self):
        self.texture.use()
        self.program['m_model'].write(self.m_model)
        self.program['m_view'].write(self.camera.m_view)
        self.program['camPos'].write(self.camera.position)

    
    def on_init(self):

        #Specify the name of the texture variable
        self.texture=self.app.mesh.texture.textures[self.tex_id]
        self.program['u_texture_0']=0
        self.texture.use()

        #Pass the projection matrix
        self.program['m_proj'].write(self.camera.m_proj)
        self.program['m_view'].write(self.camera.m_view)
        self.program['m_model'].write(self.m_model)

        #Light Source
        self.program['light.position'].write(self.app.light.position) #Light Position
        self.program['light.Ia'].write(self.app.light.Ia) #Ambient light component
        self.program['light.Id'].write(self.app.light.Id) #Diffuse Component
        self.program['light.Is'].write(self.app.light.Is) #Specular Intensity

class Body(BaseModel):
    def __init__(self,app,vao_name='body',tex_id='body',pos=(0,0,0),rot=(0,0,0),scale=(1,1,1)):
        super().__init__(app,vao_name,tex_id,pos,rot,scale)
        self.on_init()


    def update(self):
        self.texture.use()
        self.program['m_model'].write(self.m_model)
        self.program['m_view'].write(self.camera.m_view)
        self.program['camPos'].write(self.camera.position)

    
    def on_init(self):

        #Specify the name of the texture variable
        self.texture=self.app.mesh.texture.textures[self.tex_id]
        self.program['u_texture_0']=0
        self.texture.use()

        #Pass the projection matrix
        self.program['m_proj'].write(self.camera.m_proj)
        self.program['m_view'].write(self.camera.m_view)
        self.program['m_model'].write(self.m_model)

        #Light Source
        self.program['light.position'].write(self.app.light.position) #Light Position
        self.program['light.Ia'].write(self.app.light.Ia) #Ambient light component
        self.program['light.Id'].write(self.app.light.Id) #Diffuse Component
        self.program['light.Is'].write(self.app.light.Is) #Specular Intensity

class LeftJaw(BaseModel):
    def __init__(self,app,vao_name='jaw_left',tex_id='jaw_left',pos=(0,0,0),rot=(0,0,0),scale=(1,1,1)):
        super().__init__(app,vao_name,tex_id,pos,rot,scale)
        self.on_init()


    def update(self):
        self.texture.use()
        self.program['m_model'].write(self.m_model)
        self.program['m_view'].write(self.camera.m_view)
        self.program['camPos'].write(self.camera.position)

    
    def on_init(self):

        #Specify the name of the texture variable
        self.texture=self.app.mesh.texture.textures[self.tex_id]
        self.program['u_texture_0']=0
        self.texture.use()

        #Pass the projection matrix
        self.program['m_proj'].write(self.camera.m_proj)
        self.program['m_view'].write(self.camera.m_view)
        self.program['m_model'].write(self.m_model)

        #Light Source
        self.program['light.position'].write(self.app.light.position) #Light Position
        self.program['light.Ia'].write(self.app.light.Ia) #Ambient light component
        self.program['light.Id'].write(self.app.light.Id) #Diffuse Component
        self.program['light.Is'].write(self.app.light.Is) #Specular Intensity

class RightJaw(BaseModel):
    def __init__(self,app,vao_name='jaw_right',tex_id='jaw_right',pos=(0,0,0),rot=(0,0,0),scale=(1,1,1)):
        super().__init__(app,vao_name,tex_id,pos,rot,scale)
        self.on_init()


    def update(self):
        self.texture.use()
        self.program['m_model'].write(self.m_model)
        self.program['m_view'].write(self.camera.m_view)
        self.program['camPos'].write(self.camera.position)

    
    def on_init(self):

        #Specify the name of the texture variable
        self.texture=self.app.mesh.texture.textures[self.tex_id]
        self.program['u_texture_0']=0
        self.texture.use()

        #Pass the projection matrix
        self.program['m_proj'].write(self.camera.m_proj)
        self.program['m_view'].write(self.camera.m_view)
        self.program['m_model'].write(self.m_model)

        #Light Source
        self.program['light.position'].write(self.app.light.position) #Light Position
        self.program['light.Ia'].write(self.app.light.Ia) #Ambient light component
        self.program['light.Id'].write(self.app.light.Id) #Diffuse Component
        self.program['light.Is'].write(self.app.light.Is) #Specular Intensity