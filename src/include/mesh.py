from include import vao
from include import texture

class Mesh:
    def __init__(self,app):
        self.app=app

        #Two instances of textures and VAOs (for right/left screens)


        #Right
        self.vao_right=vao.VAO(app.ctx_right)
        self.texture_right=texture.Texture(app.ctx_right)

        #Left
        self.vao_left=vao.VAO(app.ctx_left)
        self.texture_left=texture.Texture(app.ctx_left)

    def destroy(self):
        self.vao_left.destroy()
        self.texture_left.destroy()

        self.vao_right.destroy()
        self.texture_right.destroy()