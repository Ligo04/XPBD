import taichi as ti
import taichi.math as tm
import math
from Entity import *
from Constraints import *
from model import *

ti.init(arch=ti.gpu)
gravity=tm.vec3(0.0,-9.8,0.0)

@ti.data_oriented
class XPBDSolver:
    def __init__(self,timestep:ti.f32,numSubStep:ti.f32,entities:ti.template()) -> None:
        self.timeStep = timestep
        self.numSubSteps = numSubStep
        self.deltaTime = timestep / numSubStep          #h
        self.entityField = entities      
        self.constraintField = Constraints.field()
        self.constaintSnode = ti.root.dense(ti.i,1).dynamic(ti.j,1024)      #max constraint:1024
        self.constaintSnode.place(self.constraintField)

    #0: pos 1:joints  2:collision
    def InitConstraint(self,constraint):
        pass
        
    #lambda
    @staticmethod
    @ti.func
    def GetPositionDeltaLambda(body1:ti.template(),body2:ti.template(),r1:tm.vec3,r2:tm.vec3,
                dt:ti.f32,corr:tm.vec3,compliance:ti.f32,lambda_total:ti.f32) -> ti.f32:
        #get generalized inverse mass
        normal = corr.normalized()
        c = corr.norm()

        w1 = body1.GetGeneralizedInvMass(normal,r1)
        w2 = body2.GetGeneralizedInvMass(normal,r2)

        #small step
        lambdaCompliance = compliance / (dt**2)
        dlambda = (-c -lambdaCompliance * lambda_total ) / (w1+w2+lambdaCompliance)

        return dlambda

    @staticmethod
    @ti.func
    def GetRotationDeltaLambda(body1:ti.template(),body2:ti.template(),dt:ti.f32,
                corr:tm.vec3,compliance:ti.f32,lambda_total:ti.f32) -> ti.f32:
        #get generalized inverse mass
        axis=tm.normalize(corr)
        magnitude=tm.length(corr)

        iMatrix1=tm.mat3(body1.invInertia.x,0.0,0.0,
                        0.0,body1.invInertia.y,0.0,
                        0.0,0.0,body1.invInertia.z)
        w1= axis.transpose() @ iMatrix1 @ axis

        iMatrix2=tm.mat3(body2.invInertia.x,0.0,0.0,
                        0.0,body2.invInertia.y,0.0,
                        0.0,0.0,body2.invInertia.z)
        w2 = axis.transpose() @ iMatrix2 @ axis

        #small step
        lambdaCompliance = compliance/(dt**2)
        dlambda=(-magnitude - lambda_total * lambdaCompliance )/(w1+w2+lambdaCompliance)

        return dlambda

    #first core project correction
    @ti.func
    def ApplyPosConstraintPair(self,body0:ti.template(),body1:ti.template(),r1:tm.vec3,r2:tm.vec3,corr:tm.vec3,delta_lamba:ti.f32):
        
        normal = corr.normalized()
        delta_p = delta_lamba * normal

        body0.ApplyPosCorrection(delta_p,r1)
        body1.ApplyPosCorrection(-delta_p,r2)

    #second core project correction
    @ti.func
    def ApplyRotConstraintPair(body0:ti.template(),body1:ti.template(),corr:tm.vec3,delta_lamba:ti.f32):
        #get generalized inverse mass
        magnitude = corr.normalized()
        corr= delta_lamba * magnitude          #p=lambda * n
        body0.ApplRotationCorrection(corr)
        body1.ApplRotationCorrection(corr)

    @ti.kernel
    def preSolve(self,gravity:ti.f32,moment_ext:tm.vec3):
        for i in ti.grouped(self.bodyField):
            #prev
            self.bodyField[i].prevTransform = self.bodyField[i].transform
            #clac vec
            self.bodyField[i].vec += self.deltaTime * gravity* self.bodyField[i].invMass
            self.bodyField[i].transform.position += self.deltaTime * self.bodyField[i].vec
            #clac omega
            currOmega=self.bodyField[i].omega
            currinvInertia=self.bodyField[i].invInertia
            l = currOmega / currinvInertia
            tau = moment_ext - tm.cross(currOmega,l)
            dw = currinvInertia + tau
            self.bodyField[i].omega += self.deltaTime * dw
            
            currOmega = self.bodyField[i].omega
            qw = Quaterion.SetFromValue(currOmega[0],currOmega[1],currOmega[2],0)
            self.bodyField[i].transform.rotation += self.deltaTime *0.5 *Quaterion.Multiply(qw,self.bodyField[i].transform.rotation)
            self.bodyField[i].transform.rotation = Quaterion.Normalized(self.bodyField[i].transform.rotation)

    @ti.kernel
    def UpdateAfterSolve(self):
        for i in ti.grouped(self.bodyField):
             self.bodyField[i].vec=(self.bodyField[i].transform.position-self.bodyField[i].prevTransform.position) / self.deltaTime

             qinv=Quaterion.Conjugate(self.bodyField[i].transform.rotation)
             dq=Quaterion.Multiply(self.bodyField[i].transform.rotation,qinv)
             omege = (2*tm.vec3(dq.xyz)) / self.deltaTime
             if dq.w>0 :
                self.bodyField[i].omega = omege
             else:
                self.bodyField[i].omega = -omege


   #*********************Constrain Solver**************************************
    @ti.kernel
    def SolveConstraint(self):
        for i in ti.grouped(self.constraintField):
            #pos constraint
            if self.constraintField[i].conType == 0:
                self.SolvePosConstraint(self.constraintField[i])
            #collision constraint
            elif self.constraintField[i].conType == 1:
                self.SolveCollision(self.constraintField[i])
            #joint constraints
            elif self.constraintField[i].conType == 2:
                self.SolveJointsConstraint(self.constraintField[i])
        
        
                    
    @ti.func
    def SolveJointsConstraint(self,jointsConstraint:ti.template()):
            body1 =  self.bodyField[jointsConstraint.body1Id]
            body2 =  self.bodyField[jointsConstraint.body1Id]
            q1= body1.transform.rotation
            q2= body2.transform.rotation
            #fixed joints
            if jointsConstraint.jointType == 0:
                q2Inv = Quaterion.Conjugate(q2)
                
                dq = Quaterion.Multiply(q1,q2Inv)
                omega = 2.0 * dq.xyz
                if dq.w < 0.0:
                    omega = - omega

                delta_lambda = XPBDSolver.GetRotationDeltaLambda(body1,body2,self.deltaTime,omega,
                                                jointsConstraint.compliance,jointsConstraint.lambda_total)
                self.ApplyRotConstraintPair(body1,body2,omega,delta_lambda) 
                jointsConstraint.lambda_total += delta_lambda

            #hinge joints
            elif jointsConstraint.jointType == 1:
                a0=Quaterion.GetQuatAixs0(q1)
                a1=Quaterion.GetQuatAixs0(q2)
                
                #align
                dQhinge = tm.cross(a0,a1)
                delta_lambda = XPBDSolver.GetRotationDeltaLambda(body1,body2,self.deltaTime,dQhinge,
                                                jointsConstraint.compliance,jointsConstraint.lambda_total)
                self.ApplyRotConstraintPair(body1,body2,dQhinge,delta_lambda)
                jointsConstraint.lambda_total += delta_lambda

                if jointsConstraint.hasSwingLimit == 1:
                    #update the quaterion
                    q1= body1.transform.rotation
                    q2= body2.transform.rotation
                    n=Quaterion.GetQuatAixs0(q1)
                    b1=Quaterion.GetQuatAixs1(q1)
                    b2=Quaterion.GetQuatAixs1(q2)
                    
                    dQlimit=self.limitAngle(n,b1,b2,
                        jointsConstraint.minSwingAngle,jointsConstraint.maxSwingAngle)
                    if dQlimit is not None:
                        delta_lambda = XPBDSolver.GetRotationDeltaLambda(body1,body2,self.deltaTime,dQlimit,
                                                jointsConstraint.compliance,jointsConstraint.lambda_total)
                        self.ApplyRotConstraintPair(body1,body2,dQlimit,delta_lambda)
                        jointsConstraint.lambda_total += delta_lambda


            #spherical
            elif jointsConstraint.jointType == 2:
                #swing limits
                if jointsConstraint.hasSwingLimit == 1:
                    a1=Quaterion.GetQuatAixs0(q1)
                    a2=Quaterion.GetQuatAixs0(q2)

                    n=tm.cross(a1,a2)
                    n=tm.normalize(n)
                    
                    dQlimit = self.limitAngle(n,b1,b2,
                      jointsConstraint.minSwingAngle,jointsConstraint.maxSwingAngle)
                    if dQlimit is not None:
                        delta_lambda = XPBDSolver.GetRotationDeltaLambda(body1,body2,self.deltaTime,dQlimit,
                                                jointsConstraint.compliance,jointsConstraint.lambda_total)
                        self.ApplyRotConstraintPair(body1,body2,dQlimit,delta_lambda)
                        jointsConstraint.lambda_total += delta_lambda

                #twist limits
                if jointsConstraint.hasTwistLimit == 1:
                    #update the quaterion
                    q1= body1.transform.rotation
                    q2= body2.transform.rotation

                    a1=Quaterion.GetQuatAixs0(q1)
                    a2=Quaterion.GetQuatAixs0(q2)

                    b1=Quaterion.GetQuatAixs1(q1)
                    b2=Quaterion.GetQuatAixs1(q2)

                    n=tm.normalize(a1+a2)
                    n1= b1- tm.dot(n,b1) * n
                    n1=tm.normalize(n1)
                    n2= b2- tm.dot(n,b2) * n
                    n2=tm.normalize(n2)

                    #handling gimbal lock proble
                    #maxCorr= 2.0 * math.PI if tm.dot(a1,a2) > -0.5 else 1.0 * self.deltaTime

                    dQlimit = self.limitAngle(n,n1,n2,
                      jointsConstraint.minSwingAngle,jointsConstraint.maxSwingAngle)
                    if dQlimit is not None:
                        delta_lambda = XPBDSolver.GetRotationDeltaLambda(body1,body2,self.deltaTime,dQlimit,
                                                jointsConstraint.compliance,jointsConstraint.lambda_total)
                        self.ApplyRotConstraintPair(body1,body2,dQlimit,delta_lambda)
                        jointsConstraint.lambda_total += delta_lambda

    @ti.func
    def SolvePosConstraint(self,posConstraint:ti.template()):
        body1 = self.bodyField[posConstraint.body1Id]
        body2 = self.bodyField[posConstraint.body2Id]
        #solve position constraint
        worldPos1 = body1.transform.position
        worldPos2 = body2.transform.position

        attachment_distance = worldPos1 - worldPos2
        delta_x = attachment_distance - self.posConstraint.maxDistance

        r1 = posConstraint.r1
        r2 = posConstraint.r2
        delta_lambda = XPBDSolver.GetPositionDeltaLambda(body1,body2,r1,r2,self.deltaTime,delta_x,
                                    posConstraint.compliace,posConstraint.lambda_total)
        #apply delta_x
        self.ApplyPosConstraintPair(body1,body2,r1,r2,delta_x,delta_lambda)

    @ti.func
    def SolveCollision(self,collisoConstraint:ti.template()):
        body1=self.bodyField[collisoConstraint.body1Id]
        body2=self.bodyField[collisoConstraint.body2Id]

        normal = collisoConstraint.contact_normal
        world_p1 = body1.transform.position + Quaterion.Rotate(body1.transform.rotaion,collisoConstraint.r1)
        world_p2 = body2.transform.position + Quaterion.Rotate(body2.transform.rotaion,collisoConstraint.r1)
        world_p1_prev = body1.prevtransform.position + Quaterion.Rotate(body1.prevtransform.rotaion,collisoConstraint.r2)
        world_p2_prev = body2.prevtransform.position + Quaterion.Rotate(body2.prevtransform.rotaion,collisoConstraint.r2)
        
        distance = tm.dot((world_p1 - world_p2),normal)
        #handle contact
        if distance > 0 :
            delta_x = distance * normal

            dlambda_n = XPBDSolver.GetPositionDeltaLambda(body1,body2,collisoConstraint.r1,collisoConstraint.r2,
                                                        self.deltaTime,delta_x,0.0,collisoConstraint.lambda_n)
            #apply collision constraint
            self.ApplyPosConstraintPair(body1,body2,collisoConstraint.r1,collisoConstraint.r2,delta_x,dlambda_n)
            collisoConstraint.lambda_n += dlambda_n

            #handle static friction
            dlambda_t = XPBDSolver.GetPositionDeltaLambda(body1,body2,collisoConstraint.r1,collisoConstraint.r2,
                                                self.deltaTime,delta_x,0.0,collisoConstraint.lambda_t)
            lambda_t_temp =collisoConstraint.lambda_t + dlambda_t

            mu_s = (body1.staticFricCoeff + body2.staticFricCoeff)/2
            
            if lambda_t_temp > collisoConstraint.lambda_n * mu_s :
                delta_p=(world_p1-world_p1_prev)-(world_p2-world_p2_prev)
                delta_p_t=delta_p - tm.dot(delta_p,normal) * normal

                #apply static fric
                self.ApplyPosConstraintPair(body1,body2,collisoConstraint.r1,collisoConstraint.r2,delta_p_t,dlambda_t)
                collisoConstraint.lambda_t += dlambda_t



    #*********************Velocity solver**************************************
    @ti.func
    def SolveVelocityWithRigibody(self,body1:ti.template(),body2:ti.template(),n:tm.vec3,
                        r1:tm.vec3,r2:tm.vec3,lambda_n:tm.vec3):
        vec = (body1.vec + tm.cross(body1.omega,r1)) - (body2.vec + tm.cross(body2.omega,r2))
        vec_n = tm.dot(n,vec)
        vec_t = vec - vec_n * n

        mu_d=(body1.dynamicFircCoeff + body2.dynamicFircCoeff) / 2
        #f_n = lambda_n / dt^2
        f_n = lambda_n / (self.deltaTime**2)
        deltaVec = -tm.normalize(vec_t) * min(self.deltaTime * mu_d * f_n,tm.length(vec_t)) 

        e=(body1.restitutionCoeff + body2.restitutionCoeff) / 2
        if vec_n < 2 * tm.length(gravity) * self.deltaTime:
            deltaVec += n * (-vec_n)
        else:
            deltaVec += n * ( -vec_n + tm.min(e*vec_n,0.0))

        normal=tm.normalize(deltaVec)
        #apply deltaVec
        w1 = body1.GetGeneralizedInvMass(normal,r1)
        w2 = body2.GetGeneralizedInvMass(normal,r2)
        #p = delta_v / (w1+w2)
        corr = deltaVec / (w1+w2)
        body1.ApplyVecCorrection(corr,r1)
        body1.ApplyVecCorrection(corr,r2)



    #todo
    @ti.func
    def JointsDamping(self,joints:ti.template()):
        pass

    #todo
    @ti.kernel
    def SolveVelocity(self):
        #handle collision vec correction
        for i in ti.grouped(self.colConstraintFiled):
            body1=self.bodyField[self.colConstraintFiled[i].body1Id]
            body2=self.bodyField[self.colConstraintFiled[i].body2Id]
            self.SolveVelocityWithRigibody(body1,body2,self.colConstraintFiled[i].contact_normal,
                                    self.colConstraintFiled[i].r1,self.colConstraintFiled[i].r2,self.colConstraintFiled[i].lambda_n)

            

    #### joints limit
    @ti.func
    def limitAngle(n:tm.vec3,n1:tm.vec3,n2:tm.vec3,alpha:ti.f32,beta:ti.f32):
        rhi=tm.asin(tm.dot(tm.cross(n1,n2),n))

        if tm.dot(n1,n2) < 0:
            rhi=math.pi-rhi     
        if rhi >math.pi:
            rhi=rhi-2*math.pi
        if rhi < -math.pi:
            rhi=rhi+2*math.pi
        
        if rhi<alpha|rhi>beta :
            rhi=tm.clamp(rhi,alpha,beta)

            #rot(n,rhi)
            q=Quaterion.SetFromAxisAngle(n,rhi)
            #n1 <- rot(n,rhi)n1
            n1=Quaterion.Rotate(q,n1)

            dQlimit=tm.cross(n1,n2)
            return dQlimit
        else:
            return None

    #set iterter nnum
    def SetDtAndNumsubStep(self,timestep,numsubsteps):
        self.timeStep=timestep
        self.numSubSteps=numsubsteps

    #after data init
    def Simulate(self):
        if self.numSubSteps > 0:
            self.deltaTime = self.timeStep/self.numSubSteps
            #collision detect(board-phase)
            for times in range(self.numSubSteps):
                #prev pos
                self.preSolve(gravity,tm.vec3(0.0,0.0,0.0))
                #constraint rigid body
                self.SolveConstraint()
                #get vec
                self.UpdateAfterSolve()
                #solve vecolity
                self.SolveVelocity()

    #return world matrix
    @ti.kernel
    def GetSolveredData(self,worldMatrixFiled:ti.template()):
        for i in ti.grouped(self.bodyField):
            worldMatrixFiled[i] = self.bodyField[i].transform.GetWorldMatrix()


    


