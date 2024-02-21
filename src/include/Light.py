import glm


#Class for lighting of the scene
#Uses the Phong lighting model
class Light:
    def __init__(self,position=(3,3,-3),color=(1,1,1)):
        self.position=glm.vec3(position)
        self.color=glm.vec3(color)

        #intensities
        self.Ia=0.1*self.color #ambient light
        self.Id=0.8*self.color #diffuse light
        self.Is=1.0*self.color #specular light

        
