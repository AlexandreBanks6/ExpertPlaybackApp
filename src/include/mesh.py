from include import vao
from include import texture

class Mesh:
    def __init__(self,app):
        self.app=app

        #Two instances of textures and VAOs (for right/left screens)


        #Right
        self.app.window_right.switch_to()
        self.vao_right=vao.VAO(self.app.ctx_right)
        self.texture_right=texture.Texture(self.app.ctx_right)

        #Left
        self.app.window_left.switch_to()
        self.vao_left=vao.VAO(self.app.ctx_left)
        self.texture_left=texture.Texture(self.app.ctx_left)

    def destroy(self):
        self.app.window_left.switch_to()
        self.vao_left.destroy()
        self.texture_left.destroy()


        self.app.window_right.switch_to()
        self.vao_right.destroy()
        self.texture_right.destroy()