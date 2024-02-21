from include import vao
from include import texture

class Mesh:
    def __init__(self,app):
        self.app=app
        self.vao=vao.VAO(app.ctx)
        self.texture=texture.Texture(app.ctx)

    def destroy(self):
        self.vao.destroy()
        self.texture.destroy()