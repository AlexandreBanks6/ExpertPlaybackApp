from include import vbo
from include import shader_program


class VAO:
    def __init__(self, ctx):
        self.ctx = ctx
        self.vbo = vbo.VBO(ctx)
        self.program = shader_program.ShaderProgram(ctx)
        self.vaos = {}
        

        #Background vao
        #self.vaos['background'] = self.get_vao(
         #   program=self.program.programs['background'],
          #  vbo = self.vbo.vbos['background'],isbackground=True)
        # shaft vao
        self.vaos['shaft'] = self.get_vao(
            program=self.program.programs['default'],
            vbo = self.vbo.vbos['shaft'])
        
        # body vao
        self.vaos['body'] = self.get_vao(
            program=self.program.programs['default'],
            vbo = self.vbo.vbos['body'])
        
        # right jaw vao
        self.vaos['jaw_right'] = self.get_vao(
            program=self.program.programs['default'],
            vbo = self.vbo.vbos['jaw_right'])
        
        # left jaw vao
        self.vaos['jaw_left'] = self.get_vao(
            program=self.program.programs['default'],
            vbo = self.vbo.vbos['jaw_left'])

        
    def get_vao(self, program,vbo,isbackground=False):

        print([(vbo.vbo, vbo.format, *vbo.attribs)])

        vao = self.ctx.vertex_array(program, [(vbo.vbo, vbo.format, *vbo.attribs)], skip_errors=True)
        return vao

    def destroy(self):
        self.vbo.destroy()
        self.program.destroy()