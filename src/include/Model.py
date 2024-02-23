import glm

class BaseModel:
    def __init__(self,app,vao_name,tex_id,pos=(0,0,0),rot=(0,0,0),scale=(1,1,1)):
        self.app=app
        self.pos=pos
        self.rot=glm.vec3([glm.radians(a) for a in rot])
        self.scale=scale
        self.m_model=self.get_model_matrix()
        self.tex_id=tex_id
        #print(app.mesh.vao.vaos)
        self.app.window_left.switch_to()
        self.vao_left=app.mesh.vao_left.vaos[vao_name]        
        self.program_left=self.vao_left.program        
        self.camera_left=self.app.camera_left

        self.app.window_right.switch_to()
        self.vao_right=app.mesh.vao_right.vaos[vao_name]
        self.camera_right=self.app.camera_right
        self.program_right=self.vao_right.program
    def update(self,left_right):...

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
    def render(self,ctx):
        if ctx==self.app.ctx_left:
            #print("Left Render")
            self.update('left')
            self.vao_left.render()
        elif ctx==self.app.ctx_right:
           #print("Right Render")
            self.update('right')
            self.vao_right.render()


class BackgroundModel:
    def __init__(self,app,vao_name='background',tex_id='background'):
        self.app=app
        self.tex_id=tex_id
        self.app.window_left.switch_to()
        self.vao_left=app.mesh.vao_left.vaos[vao_name]        
        self.program_left=self.vao_left.program        

        self.app.window_right.switch_to()
        self.vao_right=app.mesh.vao_right.vaos[vao_name]
        self.program_right=self.vao_right.program

        self.app.window_left.switch_to()
        self.texture_left=self.app.mesh.texture_left.textures[self.tex_id]
        self.program_left['u_texture_0']=0
        self.texture_left.use()

        self.app.window_right.switch_to()
        self.texture_right=self.app.mesh.texture_left.textures[self.tex_id]        
        self.program_right['u_texture_0']=0        
        self.texture_right.use()
    
    def update(self,left_right):
        if left_right=='left':
            self.texture_left.use()

        elif left_right=='right':    
            self.texture_right.use()

    def render(self,ctx):
        if ctx==self.app.ctx_left:
            #print("Left Render")
            self.update('left')
            self.vao_left.render()
        elif ctx==self.app.ctx_right:
           #print("Right Render")
            self.update('right')
            self.vao_right.render()

class Cube(BaseModel):
    def __init__(self,app,vao_name='cube',tex_id=0,pos=(0,0,0),rot=(0,0,0),scale=(1,1,1)):
        super().__init__(app,vao_name,tex_id,pos,rot,scale)
        self.on_init()


    def update(self,left_right):
        print("Update Called")
        if left_right=='left':
            self.texture_left.use(location=0)
            self.program_left['m_model'].write(self.m_model)
            print("\nLeft: ")
            print(self.m_model)
            self.program_left['m_view'].write(self.camera_left.m_view)
            self.program_left['camPos'].write(self.camera_left.position)

        elif left_right=='right':    
            self.texture_right.use(location=0)
            self.program_right['m_model'].write(self.m_model)
            print("\nRight: ")
            print(self.m_model)
            self.program_right['m_view'].write(self.camera_right.m_view)
            self.program_right['camPos'].write(self.camera_right.position)

    
    def on_init(self):

        #Specify the name of the texture variable
        self.app.window_left.switch_to()
        self.texture_left=self.app.mesh.texture_left.textures[self.tex_id]
        self.program_left['u_texture_0']=0
        self.texture_left.use(location=0)

        #Pass the projection matrix
        self.program_left['m_proj'].write(self.camera_left.m_proj)
        self.program_left['m_view'].write(self.camera_left.m_view)
        self.program_left['m_model'].write(self.m_model)

        #Light Source
        self.program_left['light.position'].write(self.app.light.position) #Light Position
        self.program_left['light.Ia'].write(self.app.light.Ia) #Ambient light component
        self.program_left['light.Id'].write(self.app.light.Id) #Diffuse Component
        self.program_left['light.Is'].write(self.app.light.Is) #Specular Intensity

        
        self.app.window_right.switch_to()
        self.texture_right=self.app.mesh.texture_left.textures[self.tex_id]        
        self.program_right['u_texture_0']=0        
        self.texture_right.use(location=0)


        self.program_right['m_proj'].write(self.camera_right.m_proj)
        self.program_right['m_view'].write(self.camera_right.m_view)
        self.program_right['m_model'].write(self.m_model)



        self.program_right['light.position'].write(self.app.light.position) #Light Position
        self.program_right['light.Ia'].write(self.app.light.Ia) #Ambient light component
        self.program_right['light.Id'].write(self.app.light.Id) #Diffuse Component
        self.program_right['light.Is'].write(self.app.light.Is) #Specular Intensity


class Shaft(BaseModel):
    def __init__(self,app,vao_name='shaft',tex_id='shaft',pos=(0,0,0),rot=(0,0,0),scale=(1,1,1)):
        super().__init__(app,vao_name,tex_id,pos,rot,scale)
        self.on_init()


    def update(self,left_right):
        if left_right=='left':
            self.texture_left.use(location=0)
            self.program_left['m_model'].write(self.m_model)
            self.program_left['m_view'].write(self.camera_left.m_view)
            self.program_left['camPos'].write(self.camera_left.position)

        elif left_right=='right':    
            self.texture_right.use(location=0)
            self.program_right['m_model'].write(self.m_model)
            self.program_right['m_view'].write(self.camera_right.m_view)
            self.program_right['camPos'].write(self.camera_right.position)

    
    def on_init(self):

        #Specify the name of the texture variable
        self.app.window_left.switch_to()
        self.texture_left=self.app.mesh.texture_left.textures[self.tex_id]
        self.program_left['u_texture_0']=0
        self.texture_left.use(location=0)

        #Pass the projection matrix
        self.program_left['m_proj'].write(self.camera_left.m_proj)
        self.program_left['m_view'].write(self.camera_left.m_view)
        self.program_left['m_model'].write(self.m_model)

        #Light Source
        self.program_left['light.position'].write(self.app.light.position) #Light Position
        self.program_left['light.Ia'].write(self.app.light.Ia) #Ambient light component
        self.program_left['light.Id'].write(self.app.light.Id) #Diffuse Component
        self.program_left['light.Is'].write(self.app.light.Is) #Specular Intensity

        
        self.app.window_right.switch_to()
        self.texture_right=self.app.mesh.texture_left.textures[self.tex_id]        
        self.program_right['u_texture_0']=0        
        self.texture_right.use(location=0)


        self.program_right['m_proj'].write(self.camera_right.m_proj)
        self.program_right['m_view'].write(self.camera_right.m_view)
        self.program_right['m_model'].write(self.m_model)



        self.program_right['light.position'].write(self.app.light.position) #Light Position
        self.program_right['light.Ia'].write(self.app.light.Ia) #Ambient light component
        self.program_right['light.Id'].write(self.app.light.Id) #Diffuse Component
        self.program_right['light.Is'].write(self.app.light.Is) #Specular Intensity


class Body(BaseModel):
    def __init__(self,app,vao_name='body',tex_id='body',pos=(0,0,0),rot=(0,0,0),scale=(1,1,1)):
        super().__init__(app,vao_name,tex_id,pos,rot,scale)
        self.on_init()


    def update(self,left_right):
        if left_right=='left':
            self.texture_left.use(location=0)
            self.program_left['m_model'].write(self.m_model)
            self.program_left['m_view'].write(self.camera_left.m_view)
            self.program_left['camPos'].write(self.camera_left.position)

        elif left_right=='right':    
            self.texture_right.use(location=0)
            self.program_right['m_model'].write(self.m_model)
            self.program_right['m_view'].write(self.camera_right.m_view)
            self.program_right['camPos'].write(self.camera_right.position)

    
    def on_init(self):

        #Specify the name of the texture variable
        self.app.window_left.switch_to()
        self.texture_left=self.app.mesh.texture_left.textures[self.tex_id]
        self.program_left['u_texture_0']=0
        self.texture_left.use(location=0)

        #Pass the projection matrix
        self.program_left['m_proj'].write(self.camera_left.m_proj)
        self.program_left['m_view'].write(self.camera_left.m_view)
        self.program_left['m_model'].write(self.m_model)

        #Light Source
        self.program_left['light.position'].write(self.app.light.position) #Light Position
        self.program_left['light.Ia'].write(self.app.light.Ia) #Ambient light component
        self.program_left['light.Id'].write(self.app.light.Id) #Diffuse Component
        self.program_left['light.Is'].write(self.app.light.Is) #Specular Intensity

        
        self.app.window_right.switch_to()
        self.texture_right=self.app.mesh.texture_left.textures[self.tex_id]        
        self.program_right['u_texture_0']=0        
        self.texture_right.use(location=0)


        self.program_right['m_proj'].write(self.camera_right.m_proj)
        self.program_right['m_view'].write(self.camera_right.m_view)
        self.program_right['m_model'].write(self.m_model)



        self.program_right['light.position'].write(self.app.light.position) #Light Position
        self.program_right['light.Ia'].write(self.app.light.Ia) #Ambient light component
        self.program_right['light.Id'].write(self.app.light.Id) #Diffuse Component
        self.program_right['light.Is'].write(self.app.light.Is) #Specular Intensity


class LeftJaw(BaseModel):
    def __init__(self,app,vao_name='jaw_left',tex_id='jaw_left',pos=(0,0,0),rot=(0,0,0),scale=(1,1,1)):
        super().__init__(app,vao_name,tex_id,pos,rot,scale)
        self.on_init()


    def update(self,left_right):
        if left_right=='left':
            self.texture_left.use(location=0)
            self.program_left['m_model'].write(self.m_model)
            self.program_left['m_view'].write(self.camera_left.m_view)
            self.program_left['camPos'].write(self.camera_left.position)

        elif left_right=='right':    
            self.texture_right.use(location=0)
            self.program_right['m_model'].write(self.m_model)
            self.program_right['m_view'].write(self.camera_right.m_view)
            self.program_right['camPos'].write(self.camera_right.position)

    
    def on_init(self):

        #Specify the name of the texture variable
        self.app.window_left.switch_to()
        self.texture_left=self.app.mesh.texture_left.textures[self.tex_id]
        self.program_left['u_texture_0']=0
        self.texture_left.use(location=0)

        #Pass the projection matrix
        self.program_left['m_proj'].write(self.camera_left.m_proj)
        self.program_left['m_view'].write(self.camera_left.m_view)
        self.program_left['m_model'].write(self.m_model)

        #Light Source
        self.program_left['light.position'].write(self.app.light.position) #Light Position
        self.program_left['light.Ia'].write(self.app.light.Ia) #Ambient light component
        self.program_left['light.Id'].write(self.app.light.Id) #Diffuse Component
        self.program_left['light.Is'].write(self.app.light.Is) #Specular Intensity

        
        self.app.window_right.switch_to()
        self.texture_right=self.app.mesh.texture_left.textures[self.tex_id]        
        self.program_right['u_texture_0']=0        
        self.texture_right.use(location=0)


        self.program_right['m_proj'].write(self.camera_right.m_proj)
        self.program_right['m_view'].write(self.camera_right.m_view)
        self.program_right['m_model'].write(self.m_model)



        self.program_right['light.position'].write(self.app.light.position) #Light Position
        self.program_right['light.Ia'].write(self.app.light.Ia) #Ambient light component
        self.program_right['light.Id'].write(self.app.light.Id) #Diffuse Component
        self.program_right['light.Is'].write(self.app.light.Is) #Specular Intensity

class RightJaw(BaseModel):
    def __init__(self,app,vao_name='jaw_right',tex_id='jaw_right',pos=(0,0,0),rot=(0,0,0),scale=(1,1,1)):
        super().__init__(app,vao_name,tex_id,pos,rot,scale)
        self.on_init()


    def update(self,left_right):
        if left_right=='left':
            self.texture_left.use(location=0)
            self.program_left['m_model'].write(self.m_model)
            self.program_left['m_view'].write(self.camera_left.m_view)
            self.program_left['camPos'].write(self.camera_left.position)

        elif left_right=='right':    
            self.texture_right.use(location=0)
            self.program_right['m_model'].write(self.m_model)
            self.program_right['m_view'].write(self.camera_right.m_view)
            self.program_right['camPos'].write(self.camera_right.position)

    
    def on_init(self):

        #Specify the name of the texture variable
        self.app.window_left.switch_to()
        self.texture_left=self.app.mesh.texture_left.textures[self.tex_id]
        self.program_left['u_texture_0']=0
        self.texture_left.use(location=0)

        #Pass the projection matrix
        self.program_left['m_proj'].write(self.camera_left.m_proj)
        self.program_left['m_view'].write(self.camera_left.m_view)
        self.program_left['m_model'].write(self.m_model)

        #Light Source
        self.program_left['light.position'].write(self.app.light.position) #Light Position
        self.program_left['light.Ia'].write(self.app.light.Ia) #Ambient light component
        self.program_left['light.Id'].write(self.app.light.Id) #Diffuse Component
        self.program_left['light.Is'].write(self.app.light.Is) #Specular Intensity

        
        self.app.window_right.switch_to()
        self.texture_right=self.app.mesh.texture_left.textures[self.tex_id]        
        self.program_right['u_texture_0']=0        
        self.texture_right.use(location=0)


        self.program_right['m_proj'].write(self.camera_right.m_proj)
        self.program_right['m_view'].write(self.camera_right.m_view)
        self.program_right['m_model'].write(self.m_model)



        self.program_right['light.position'].write(self.app.light.position) #Light Position
        self.program_right['light.Ia'].write(self.app.light.Ia) #Ambient light component
        self.program_right['light.Id'].write(self.app.light.Id) #Diffuse Component
        self.program_right['light.Is'].write(self.app.light.Is) #Specular Intensity
