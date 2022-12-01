import taichi as ti
import taichi.math as tm



@ti.dataclass
class PositonConstaints:
    body1Id:ti.i32
    body2Id:ti.i32
    r1:tm.vec3
    r2:tm.vec3
    #distance constraint
    maxDistance:tm.vec3
    lambda_total:ti.f32
    compliance:ti.f32

    @ti.func
    def Init(self,body1Id:ti.i32,body2Id:ti.i32,r1:tm.vec3,r2:tm.vec3,
        maxDistance:tm.vec3,compliance:ti.f32):
        self.constraintsType = 0
        self.body1Id = body1Id
        self.body2Id = body2Id

        self.r1= r1
        self.r2= r2

        self.maxDistance=maxDistance
        self.compliance=compliance

        self.lambda_total=0

#todo(做不完了)
@ti.dataclass
class CollisionConstraint:
    body1Id:ti.i32
    body2Id:ti.i32
    r1:tm.vec3
    r2:tm.vec3
    #collision constraint
    contact_normal:tm.vec3
    lambda_n:ti.f32
    lambda_t:ti.f32

    @ti.func
    def Init(self,body1Id:ti.i32,body2Id:ti.i32,r1:tm.vec3,r2:tm.vec3,normal:tm.vec3):
        self.constraintsType = 1
        self.body1Id = body1Id
        self.body2Id = body2Id

        self.r1= r1
        self.r2= r2
        self.contact_normal=normal

#Joints Constraints
@ti.dataclass
class JointsConstraint:
    #type(fixed,hinge,spherical)
    type:ti.i32

    body1Id:ti.i32
    body2Id:ti.i32
    lambda_total:ti.f32
    #fixed hinge
    compliance:ti.f32

    #Swing Limit
    hasSwingLimit:ti.i8
    minSwingAngle:ti.f32
    maxSwingAngle:ti.f32
    SwingLimitCompliance:ti.f32
    #TwingLimit
    hasTwistLimit:ti.i8
    minTwistAngle:ti.f32
    maxTwistAngle:ti.f32
    TwistLimitCompliance:ti.f32
        
        
    


