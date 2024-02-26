import numpy as np
from plyfile import PlyData, PlyElement
import moderngl as mgl
import pywavefront


LOGO_BODY_FILE='../resources/instrument_models/NewOrigin_Models/Decimated_OBJ/Body_Decimated.obj'
SHAFT_FILE='../resources/instrument_models/NewOrigin_Models/Decimated_OBJ/Shaft_Decimated.obj'
RIGHT_JAW_FILE='../resources/instrument_models/NewOrigin_Models/Decimated_OBJ/JawRight_Decimated.obj'
LEFT_JAW_FILE='../resources/instrument_models/NewOrigin_Models/Decimated_OBJ/JawLeft_Decimated.obj'


class VBO:
    def __init__(self, ctx):
        self.vbos = {}
        #self.vbos['background']=BackgroundVBO(ctx)
        self.vbos['shaft']= ShaftVBO(ctx)
        self.vbos['body']=BodyVBO(ctx)
        self.vbos['jaw_right']=RightJawVBO(ctx)
        self.vbos['jaw_left']=LeftJawVBO(ctx)


    def destroy(self):
        [vbo.destroy() for vbo in self.vbos.values()]


class BaseVBO:
    def __init__(self, ctx):
        self.ctx = ctx
        self.vbo = self.get_vbo()
        self.format: str = None
        self.attribs: list = None

    def get_vertex_data(self): ...

    def get_vbo(self):
        vertex_data = self.get_vertex_data()
        vbo = self.ctx.buffer(vertex_data)
        return vbo

    def destroy(self):
        self.vbo.release()


class CubeVBO(BaseVBO):
    def __init__(self, ctx):
        super().__init__(ctx)
        self.format = '2f 3f 3f'
        self.attribs = ['in_texcoord_0', 'in_normal', 'in_position']

    @staticmethod
    def get_data(vertices, indices):
        data = [vertices[ind] for triangle in indices for ind in triangle]
        return np.array(data, dtype='f4')

    def get_vertex_data(self):
        vertices = [(-1, -1, 1), ( 1, -1,  1), (1,  1,  1), (-1, 1,  1),
                    (-1, 1, -1), (-1, -1, -1), (1, -1, -1), ( 1, 1, -1)]

        indices = [(0, 2, 3), (0, 1, 2),
                   (1, 7, 2), (1, 6, 7),
                   (6, 5, 4), (4, 7, 6),
                   (3, 4, 5), (3, 5, 0),
                   (3, 7, 4), (3, 2, 7),
                   (0, 6, 1), (0, 5, 6)]
        vertex_data = self.get_data(vertices, indices)

        tex_coord_vertices = [(0, 0), (1, 0), (1, 1), (0, 1)]
        tex_coord_indices = [(0, 2, 3), (0, 1, 2),
                             (0, 2, 3), (0, 1, 2),
                             (0, 1, 2), (2, 3, 0),
                             (2, 3, 0), (2, 0, 1),
                             (0, 2, 3), (0, 1, 2),
                             (3, 1, 2), (3, 0, 1),]
        tex_coord_data = self.get_data(tex_coord_vertices, tex_coord_indices)

        normals = [( 0, 0, 1) * 6,
                   ( 1, 0, 0) * 6,
                   ( 0, 0,-1) * 6,
                   (-1, 0, 0) * 6,
                   ( 0, 1, 0) * 6,
                   ( 0,-1, 0) * 6,]
        normals = np.array(normals, dtype='f4').reshape(36, 3)

        vertex_data = np.hstack([normals, vertex_data])
        vertex_data = np.hstack([tex_coord_data, vertex_data])
        return vertex_data

class ShaftVBO(BaseVBO):
    def __init__(self,app):
        super().__init__(app)
        self.format='2f 3f 3f'
        self.attribs=['in_texcoord_0','in_normal','in_position']
    def get_vertex_data(self):
        objs=pywavefront.Wavefront(SHAFT_FILE)
        obj=objs.materials.popitem()[1]
        vertex_data=obj.vertices
        vertex_data=np.array(vertex_data,dtype='f4')
        return vertex_data
    

class BodyVBO(BaseVBO):
    def __init__(self,app):
        super().__init__(app)
        self.format='2f 3f 3f'
        self.attribs=['in_texcoord_0','in_normal','in_position']
    def get_vertex_data(self):
        objs=pywavefront.Wavefront(LOGO_BODY_FILE)
        obj=objs.materials.popitem()[1]
        vertex_data=obj.vertices
        vertex_data=np.array(vertex_data,dtype='f4')
        return vertex_data
    
class RightJawVBO(BaseVBO):
    def __init__(self,app):
        super().__init__(app)
        self.format='2f 3f 3f'
        self.attribs=['in_texcoord_0','in_normal','in_position']
    def get_vertex_data(self):
        objs=pywavefront.Wavefront(RIGHT_JAW_FILE)
        obj=objs.materials.popitem()[1]
        vertex_data=obj.vertices
        vertex_data=np.array(vertex_data,dtype='f4')
        return vertex_data
    
class LeftJawVBO(BaseVBO):
    def __init__(self,app):
        super().__init__(app)
        self.format='2f 3f 3f'
        self.attribs=['in_texcoord_0','in_normal','in_position']
    def get_vertex_data(self):
        objs=pywavefront.Wavefront(LEFT_JAW_FILE)
        obj=objs.materials.popitem()[1]
        vertex_data=obj.vertices
        vertex_data=np.array(vertex_data,dtype='f4')
        return vertex_data
    

'''
class BackgroundVBO(BaseVBO):
    def __init__(self,app):
        super().__init__(app)
        self.format='2f 2f'
        self.attribs=['in_texcoord_0', 'in_position']
    def get_vertex_data(self):   
        vertex_data = np.array([
            # Bottom Left
            0.0, 0.0,  # Texture Coords
            -1.0, -1.0,  # Position Coords

            # Bottom Right
            1.0, 0.0, 
            1.0, -1.0,

            # Top Right
            1.0, 1.0,
            1.0, 1.0,

            # Top Left
            0.0, 1.0,
            -1.0, 1.0,
            
            #Bottom Left
            0.0,0.0,
            -1.0,-1.0,

            # Top Right
            1.0, 1.0,
            1.0, 1.0            
            ], dtype='f4')
        

        return vertex_data

'''



    