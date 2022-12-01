import taichi as ti
import taichi.math as tm
import Transform as ts

ti.init()

@ti.dataclass
class RigidBodyData:
    invMass:ti.f32

@ti.dataclass
class rigibody:
    data:RigidBodyData

