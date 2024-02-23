import pygame as pg
import moderngl as mgl
from pyglet import image
#import glfw
from PIL import Image
#import numpy as np


class Texture:
    def __init__(self, ctx):
        self.ctx = ctx
        self.textures = {}
        #self.textures[0] = self.get_texture(path='textures/texture_test.png')
        self.textures['background']=self.get_texture(path='textures/texture_test.png')
        self.textures['shaft']=self.get_texture(path='textures/gray_texture_instruments.png')
        self.textures['body']=self.get_texture(path='textures/gray_texture_instruments.png')
        self.textures['jaw_right']=self.get_texture(path='textures/gray_texture_instruments.png')
        self.textures['jaw_left']=self.get_texture(path='textures/gray_texture_instruments.png')


    def get_texture(self, path):
        #img2=Image.open(path)
        #h,w=img2.size
        #image=image.convert('RGBA')
        #image=image.transpose(Image.FLIP_TOP_BOTTOM)
        #img_data=np.array(list(image.getdata()),np.uint8)
        #h,w,=image.size
        #texture = pg.image.load(path).convert()
        #texture = pg.transform.flip(texture, flip_x=False, flip_y=True)
        #texture = self.ctx.texture(size=(h,w), components=4,
                                   #data=img_data)
        
        #texture = pg.image.load(path).convert()
        #texture = pg.transform.flip(texture, flip_x=False, flip_y=True)

        #Try loading texture with pyglet
        img=image.load(path)
        widht,height=img.width,img.height
        raw_image=img.get_image_data()
        format = 'RGB'
        pitch = raw_image.width * len(format)
        pixels = raw_image.get_data(format, pitch)
        #img_data=
        #texture=img.get_texture()
        #texture=texture.get_transform(flip_y=False)
        #texture=texture.get_image_data()
        texture = self.ctx.texture(size=(height,widht), components=3,
                                   data=pixels)       
        
        #mipmaps
        texture.filter=(mgl.LINEAR_MIPMAP_LINEAR,mgl.LINEAR)
        texture.build_mipmaps()

        #AF
        texture.anisotropy=32.0 #elminate aliasing on surfaces

        return texture
    def get_background_texture(self):

        img=image.load('textures/texture_test.png')
        widht,height=img.width,img.height
        raw_image=img.get_image_data()
        format = 'RGB'
        pitch = raw_image.width * len(format)
        pixels = raw_image.get_data(format, pitch)
        #img_data=
        #texture=img.get_texture()
        #texture=texture.get_transform(flip_y=False)
        #texture=texture.get_image_data()
        texture = self.ctx.texture(size=(height,widht), components=3,
                                   data=pixels)       
        
        #mipmaps
        texture.filter=(mgl.LINEAR_MIPMAP_LINEAR,mgl.LINEAR)
        texture.build_mipmaps()

        #AF
        texture.anisotropy=32.0 #elminate aliasing on surfaces

        return texture

    def destroy(self):
        [tex.release() for tex in self.textures.values()]