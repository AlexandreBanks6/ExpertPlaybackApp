class ShaderProgram:
    def __init__(self, ctx):
        self.ctx = ctx
        self.programs = {}
        self.programs['background']=self.get_program('background')
        self.programs['default'] = self.get_program('default')

    def get_program(self, shader_program_name):
        try:
            print("Shader Entered")
            with open(f'shaders/{shader_program_name}.vert') as file:
                vertex_shader_source = file.read()

            with open(f'shaders/{shader_program_name}.frag') as file:
                fragment_shader_source = file.read()

            program = self.ctx.program(vertex_shader=vertex_shader_source, fragment_shader=fragment_shader_source)
            return program

        except Exception as e:
            print(f"Error creating shader program '{shader_program_name}': {e}")
            return None

    def destroy(self):
        [program.release() for program in self.programs.values()]