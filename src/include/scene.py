from include import Model

class Scene:
    def __init__(self,app):
        self.app=app
        self.objects=[]
        self.load()

    def add_object(self,obj):
        self.objects.append(obj)

    def load(self):
        app=self.app
        add=self.add_object

        add(Model.Shaft(app,pos=(0,0,-20)))
        add(Model.Body(app,pos=(0,0,-20)))
        add(Model.LeftJaw(app,pos=(0,0,-20)))
        add(Model.RightJaw(app,pos=(0,0,-20)))

        #add(Model.Cube(app))
        #add(Model.Cube(app,tex_id=0,pos=(-2.5,0,0),rot=(45,0,0),scale=(1,2,1)))
        #add(Model.Cube(app,tex_id=0,pos=(2.5,0,0),rot=(0,0,45),scale=(1,1,2)))
    def render(self,ctx):
        for obj in self.objects:
            obj.render(ctx)
    def move_obj(self,obj_name,new_mat):
        for obj in self.objects:
            if obj.tex_id==obj_name:
                obj.move(new_mat)