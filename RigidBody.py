import taichi as ti
import taichi.math as tm
from Transform import *

@ti.dataclass
class RigidBodyData:
    invMass:ti.f32
    #curr state
    invInertia:tm.vec3
    transform:Transform
    vec:tm.vec3
    omege:tm.vec3
    #last state
    prevInertia:tm.vec3
    prevTarnsform:Transform
    prevVec:tm.vec3
    prevOmega:tm.vec3
    #coefficient
    staticFricCoeff:ti.f32
    dynamicFircCoeff:ti.f32
    restitutionCoeff:ti.f32
        
    @ti.func
    def GetGeneralizedInvMass(self,normal:tm.vec3,r:tm.vec3) -> tm.vec3:
        rn = tm.cross(r,normal)
        invInertia = self.invInertia
        
        wI = tm.vec3(rn[0]*rn[0]*invInertia[0],rn[1]*rn[1]*invInertia[1],rn[2]*rn[2]*invInertia[2])

        w=self.invMass+wI

        return w
        
    @ti.func
    def ApplRotationCorrection(self,corr:tm.vec3):
        invInertia = self.invInertia

        dq = Quaterion.SetFromValue(invInertia[0]*corr[0],invInertia[1]*corr[1],invInertia[2]*corr[2],0.0)
        dq = Quaterion.Multiply(dq,self.transform.rotation)

        self.transform.rotation += 0.5*dq
        self.transform.rotation = Quaterion.Normalized(self.transform.rotation)

    
    @ti.func
    def ApplyPosCorrection(self,corr:tm.vec3,r:tm.vec3):
        self.transform.position += corr*self.invMass

        invInertia = self.invInertia
        rp=tm.cross(r,corr)
        deltaQuatetion = Quaterion.SetFromValue(invInertia[0]*rp[0],invInertia[1]*rp[1],invInertia[2]*rp[2],0.0)
        deltaQuatetion = Quaterion.Multiply(deltaQuatetion,self.transform.rotation)
        self.transform.rotation += 0.5*deltaQuatetion

    @ti.func
    def ApplyVecCorrection(self,corr:tm.vec3,r:tm.vec3):
        self.vec += corr*self.invMass

        deltaOmega = tm.cross(r,corr)
        invInertia = self.invInertia
        deltaOmega = tm.vec3(invInertia[0]*deltaOmega[0],invInertia[1]*deltaOmega[1],invInertia[2]*deltaOmega[2])

        self.omege += deltaOmega
    