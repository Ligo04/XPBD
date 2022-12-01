import taichi as ti
import taichi.math as tm
from RigidBody import *
from vispy import io

@ti.data_oriented
class Model:
    def __init__(self, max_vertex_num=1024, max_faces_num=1024, color=ti.Vector([0.0, 0.0, 0.0]), material=1):
        self.vertex_num = ti.field(dtype=ti.i32, shape=())
        self.vertex_num[None] = 0
        self.face_num = ti.field(dtype=ti.i32, shape=())
        self.face_num[None] = 0
        self.vertex = ti.Vector.field(4, dtype=ti.f32, shape=max_vertex_num)
        self.face = ti.Vector.field(3, dtype=ti.i32, shape=max_faces_num)
        self.color = ti.Vector.field(3, dtype=ti.f32, shape=())
        self.color[None] = color
        self.material = ti.field(dtype=ti.i32, shape=())
        self.material[None] = material
        self.position = ti.Vector([0,0,0])

    def set_color(self, color):
        self.color[None] = color

    def set_material(self, material):
        self.material[None] = material

    def add_face(self, vertex):
        for index in range(3):
            self.vertex[self.vertex_num[None] + index] = ti.Vector(
                [vertex[index, 0], vertex[index, 1], vertex[index, 2], 1])
        self.face[self.face_num[None]] = ti.Vector(
            [self.vertex_num[None], self.vertex_num[None] + 1, self.vertex_num[None] + 2])
        self.vertex_num[None] += 3
        self.face_num[None] += 1

    def clear_face(self):
        self.vertex_num[None] = 0
        self.face_num[None] = 0

    def from_obj(self, filename):
        self.clear_face()
        vertex, face, normal, texture = io.read_mesh(filename)
        for index in range(len(vertex)):
            self.vertex[index] = ti.Vector([vertex[index][0], vertex[index][1], vertex[index][2], 1])
        for index in range(len(face)):
            self.face[index] = ti.Vector([face[index][0], face[index][1], face[index][2]])
        self.vertex_num[None] = len(vertex)
        self.face_num[None] = len(face)

    #get normal
    def GetNormal(self):


    @ti.kernel
    def Gettransform(self, transform :ti.template(),vertices:ti.template()):
        worldMatrix = transform[0].GetWorldMatrix()
        for index in range(self.vertex_num[None]):
            worldVertex = worldMatrix @ self.vertex[index]
            vertices[index] = ti.Vector([worldVertex[0],worldVertex[1],worldVertex[2]])

    #get inertia(diag)
    def GetInertia(self):
        pass

    

    @staticmethod
    def GetBoxInertia(mass,width,height,depth):
        return tm.vec3((1.0/12.0)*mass*(height**2+depth**2),
                        (1.0/12.0)*mass*(width**2+height**2),
                        (1.0/12.0)*mass*(width**2+depth**2))    
    
    @staticmethod
    def GetSphereInertia(mass,radius):
        return tm.vec3((2.0/5.0)*mass*(radius**2),
                        (2.0/5.0)*mass*(radius**2),
                        (2.0/5.0)*mass*(radius**2)) 

        



@ti.data_oriented
class RigidBody:
    def __init__(self,model,position,mass,inertia=tm.vec3(0,0,0)) -> None:
        self.model = model
        self.vectices =  ti.Vector.field(3, dtype=ti.f32, shape= self.model.vertex_num[None])
        self.mass = mass 
        self.inertia = inertia
        self.tranform = Transform.field(shape=1)


    #mesh data：vectices,indices,color
    def GetSceneMeshData(self):
        self.model.Gettransform(self.tranform,self.vectices)
        return self.vectices,self.model.face,self.model.color

    def CreateRigidBodyData(self,vec=tm.vec3(0,0,0),omega=tm.vec3(0,0,0),staticFric=0,dynmanicFric=0,restitutionC=0):
        data = RigidBodyData()
        data.invMass = 1.0 / self.mass
        data.transform = self.tranform
        data.vec =vec
        data.omega= omega
        data.staticFricCoeff = staticFric
        data.dynamicFircCoeff = dynmanicFric
        data.restitutionCoeff = restitutionC
        return data



    
