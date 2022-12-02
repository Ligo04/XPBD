import taichi as ti
import taichi.math as tm
from Transform import *

@ti.dataclass
class Entity:
    invMass:ti.f32
    #curr state
    invInertia:tm.vec3
    transform:Transform
    vec:tm.vec3
    omege:tm.vec3
    #last state
    prevTarnsform:Transform
    #coefficient
    staticFricCoeff:ti.f32
    dynamicFircCoeff:ti.f32
    restitutionCoeff:ti.f32
        
    @ti.func
    def GetGeneralizedInvMass(self,normal:tm.vec3,r:tm.vec3) -> ti.f32:
        rn = tm.cross(r,normal)
        invInertia = self.invInertia
        iMatrix=tm.mat3(invInertia.x,0.0,0.0,
                        0.0,invInertia.y,0.0,
                        0.0,0.0,invInertia.z)
        wI = rn.transpose() @ iMatrix @ rn

        w=self.invMass+wI

        return w
        
    @ti.func
    def ApplRotationCorrection(self,corr:tm.vec3):
        invInertia = self.invInertia
        iMatrix=tm.mat3(invInertia.x,0.0,0.0,
                        0.0,invInertia.y,0.0,
                        0.0,0.0,invInertia.z)
        iP = iMatrix @ corr

        dq = Quaterion.SetFromValue(iP.x,iP.y,iP.z,0.0)
        dq = Quaterion.Multiply(dq,self.transform.rotation)

        self.transform.rotation += 0.5*dq
        self.transform.rotation = Quaterion.Normalized(self.transform.rotation)

    
    @ti.func
    def ApplyPosCorrection(self,corr:tm.vec3,r:tm.vec3):
        self.transform.position += corr*self.invMass

        invInertia = self.invInertia
        iMatrix=tm.mat3(invInertia.x,0.0,0.0,
                        0.0,invInertia.y,0.0,
                        0.0,0.0,invInertia.z)
        rp=tm.cross(r,corr)

        iRp = iMatrix @ rp
        deltaQuatetion = Quaterion.SetFromValue(iRp.x,iRp.y,iRp.z,0.0)
        deltaQuatetion = Quaterion.Multiply(deltaQuatetion,self.transform.rotation)
        self.transform.rotation += 0.5*deltaQuatetion

    @ti.func
    def ApplyVecCorrection(self,corr:tm.vec3,r:tm.vec3):
        self.vec += corr*self.invMass

        deltaOmega = tm.cross(r,corr)
        invInertia = self.invInertia
        iMatrix=tm.mat3(invInertia.x,0.0,0.0,
                        0.0,invInertia.y,0.0,
                        0.0,0.0,invInertia.z)

        iDeltaOmega= iMatrix @ deltaOmega
        deltaOmega = tm.vec3(iDeltaOmega.x,iDeltaOmega.y,iDeltaOmega.z)

        self.omege += deltaOmega
    