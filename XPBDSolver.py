import taichi as ti
import taichi.math as tm
import math
import time
from Entity import *
from Constraints import *
from model import *

gravity=tm.vec3(0.0,-9.8,0.0)

@ti.data_oriented
class XPBDSolver:
    def __init__(self,timestep:ti.f32,numSubStep:ti.f32,entities:list=[]) -> None:
        self.timeStep = timestep
        self.numSubSteps = numSubStep
        self.dt = 2.5e-4
        self.solverEntity = 128
        self.solverConstraint = (self.solverEntity + 1) **2
        self.root = ti.root.dense(ti.i,1)

        self.entityField = Entity.field(layout=ti.Layout.AOS)
        self.entitySnode = self.root.dynamic(ti.j,self.solverEntity)                   
        self.entitySnode.place(self.entityField)

        self.worldField = tm.mat4.field(layout=ti.Layout.AOS)
        self.worldSnode = self.root.dynamic(ti.j,self.solverEntity)    
        self.worldSnode.place(self.worldField)

        self.constraintField = Constraints.field(layout=ti.Layout.SOA)
        self.constaintSnode = self.root.dynamic(ti.j,self.solverConstraint)      #max constraint:1024
        self.constaintSnode.place(self.constraintField)

        for i in range(len(entities)):
            self.AddEntity(entities[i])
        

    @ti.kernel
    def AddEntity(self,entity:ti.template()):
        ti.append(self.entitySnode,0,entity[None])

    @ti.kernel
    def AddStraint(self,constraint:ti.template()):
        ti.append(self.constaintSnode,0,constraint[None])
        
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
        dlambda = (-c) / (w1+w2+lambdaCompliance)

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
        dlambda=(-magnitude)/(w1+w2+lambdaCompliance)

        return dlambda

    #first core project correction
    @ti.func
    def ApplyPosConstraintPair(self,body1:ti.template(),body2:ti.template(),r1:tm.vec3,r2:tm.vec3,corr:tm.vec3,delta_lamba:ti.f32):
        normal = corr.normalized()
        delta_p = delta_lamba * normal
        if body1.fixed == 0:
            body1.dp_p += delta_p
            body1.r = r1
        if body2.fixed == 0:
            body2.dp_p -= delta_p
            body2.r = r2

    #second core project correction
    @ti.func
    def ApplyRotConstraintPair(self,body1:ti.template(),body2:ti.template(),corr:tm.vec3,delta_lamba:ti.f32):
        #get generalized inverse mass
        magnitude = corr.normalized()
        corr= delta_lamba * magnitude          #p=lambda * n
        if body1.fixed == 0:
            body1.dp_r += corr
        if body2.fixed == 0:
            body2.dp_r -= corr

    @ti.kernel
    def preSolve(self,dt:ti.f32,gravity:tm.vec3,moment_ext:tm.vec3):
        for i in ti.grouped(self.entityField):
            if self.entityField[i].fixed == 0:
                #prev
                self.entityField[i].prevTransform = self.entityField[i].transform
                #clac vec
                self.entityField[i].vec += dt * gravity* self.entityField[i].invMass
                self.entityField[i].transform.position += dt * self.entityField[i].vec
                #clac omega
                currOmega=self.entityField[i].omega
                currinvInertia=self.entityField[i].invInertia
                l = currOmega / currinvInertia
                tau = moment_ext - tm.cross(currOmega,l)
                dw = currinvInertia + tau
                self.entityField[i].omega += dt * dw
            
                currOmega = self.entityField[i].omega
                qw = Quaterion.SetFromValue(currOmega[0],currOmega[1],currOmega[2],0)
                self.entityField[i].transform.rotation += dt *0.5 *Quaterion.Multiply(qw,self.entityField[i].transform.rotation)
                self.entityField[i].transform.rotation = Quaterion.Normalized(self.entityField[i].transform.rotation)

    @ti.kernel
    def UpdateAfterSolve(self,dt:ti.f32):
        for i in ti.grouped(self.entityField):
            if self.entityField[i].fixed == 0:
                self.entityField[i].ApplyPosCorrection(self.entityField[i].dp_p,self.entityField[i].r)
                self.entityField[i].ApplRotationCorrection(self.entityField[i].dp_r)

                delta_x = self.entityField[i].transform.position-self.entityField[i].prevTransform.position
                self.entityField[i].vec = delta_x / dt
                qinv=Quaterion.Conjugate(self.entityField[i].transform.rotation)
                dq=Quaterion.Multiply(self.entityField[i].transform.rotation,qinv)
                omega = (2*tm.vec3(dq.xyz)) / dt
                if dq.w>0 :
                    self.entityField[i].omega = omega
                else:
                    self.entityField[i].omega = -omega


   #*********************Constrain Solver**************************************
    @ti.kernel
    def SolveConstraint(self,dt:ti.f32):
        for i in ti.grouped(self.constraintField):
            #pos constraint
            if self.constraintField[i].conType == 0:
                self.SolvePosConstraint(self.constraintField[i],dt)
            #collision constraint
            elif self.constraintField[i].conType == 1:
                self.SolveCollision(self.constraintField[i],dt)
            #joint constraints
            elif self.constraintField[i].conType == 2:
                self.SolveJointsConstraint(self.constraintField[i],dt)

        
                    
    @ti.func
    def SolveJointsConstraint(self,jointsConstraint:ti.template(),dt:ti.f32):
            body1 =  self.entityField[0,jointsConstraint.body1Id]
            body2 =  self.entityField[0,jointsConstraint.body1Id]
            q1= body1.transform.rotation
            q2= body2.transform.rotation
            #fixed joints
            if jointsConstraint.jointType == 0:
                q2Inv = Quaterion.Conjugate(q2)
                
                dq = Quaterion.Multiply(q1,q2Inv)
                omega = 2.0 * dq.xyz
                if dq.w < 0.0:
                    omega = - omega

                if omega.norm() > 0: 
                    delta_lambda = XPBDSolver.GetRotationDeltaLambda(body1,body2,dt,omega,
                                                jointsConstraint.compliance,jointsConstraint.lambda_total)

                    self.ApplyRotConstraintPair(body1,body2,omega,delta_lambda) 
                #jointsConstraint.lambda_total += delta_lambda

            #hinge joints
            elif jointsConstraint.jointType == 1:
                a0=Quaterion.GetQuatAixs0(q1)
                a1=Quaterion.GetQuatAixs0(q2)
                
                #align
                dQhinge = tm.cross(a0,a1)
                if dQhinge.norm() > 0:
                    delta_lambda = XPBDSolver.GetRotationDeltaLambda(body1,body2,dt,dQhinge,
                                                jointsConstraint.compliance,jointsConstraint.lambda_total)
                    self.ApplyRotConstraintPair(body1,body2,dQhinge,delta_lambda)
                
                    jointsConstraint.lambda_total += delta_lambda

                if jointsConstraint.hasSwingLimit == 1:
                    #update the quaterion
                    n  = Quaterion.GetQuatAixs0(q1)
                    b1 = Quaterion.GetQuatAixs0(q1)
                    b2 = Quaterion.GetQuatAixs0(q2)

                    
                    dQlimit=XPBDSolver.limitAngle(n,b1,b2,
                        jointsConstraint.minSwingAngle,jointsConstraint.maxSwingAngle)
                    if dQlimit.norm() > 0:
                        delta_lambda = XPBDSolver.GetRotationDeltaLambda(body1,body2,dt,dQlimit,
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

                    
                    dQlimit = XPBDSolver.limitAngle(n,a1,a2,
                      jointsConstraint.minSwingAngle,jointsConstraint.maxSwingAngle)
                    if dQlimit.norm() > 0:
                        delta_lambda = XPBDSolver.GetRotationDeltaLambda(body1,body2,dt,dQlimit,
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

                    dQlimit = XPBDSolver.limitAngle(n,n1,n2,
                      jointsConstraint.minSwingAngle,jointsConstraint.maxSwingAngle)
                    if dQlimit.norm() > 0:
                        delta_lambda = XPBDSolver.GetRotationDeltaLambda(body1,body2,dt,dQlimit,
                                                jointsConstraint.compliance,jointsConstraint.lambda_total)
                        self.ApplyRotConstraintPair(body1,body2,dQlimit,delta_lambda)
                        jointsConstraint.lambda_total += delta_lambda

    @ti.func
    def SolvePosConstraint(self,posConstraint:ti.template(),dt:ti.f32):
        body1 = self.entityField[0,posConstraint.body1Id]
        body2 = self.entityField[0,posConstraint.body2Id]
        r1 = posConstraint.r1
        r2 = posConstraint.r2
        #solve position constraint
        worldPos1 = body1.transform.position + r1
        worldPos2 = body2.transform.position + r2

        attachment_distance = worldPos1 - worldPos2
        delta_x = attachment_distance.normalized() * (attachment_distance.norm()-posConstraint.maxDistance)

        delta_lambda = XPBDSolver.GetPositionDeltaLambda(body1,body2,r1,r2,dt,delta_x,
                                    posConstraint.compliance,posConstraint.lambda_total)
        
        #apply delta_x
        if delta_x.norm() > 0: 
            self.ApplyPosConstraintPair(body1,body2,r1,r2,delta_x,delta_lambda)

    @ti.func
    def SolveCollision(self,collisoConstraint:ti.template(),dt:ti.f32):
        body1 = self.entityField[0,collisoConstraint.body1Id]
        body2 = self.entityField[0,collisoConstraint.body2Id]

        normal = collisoConstraint.contact_normal
        world_p1 = body1.transform.position + Quaterion.Rotate(body1.transform.rotation,collisoConstraint.r1)
        world_p2 = body2.transform.position + Quaterion.Rotate(body2.transform.rotation,collisoConstraint.r1)
        world_p1_prev = body1.prevTransform.position + Quaterion.Rotate(body1.prevTransform.rotation,collisoConstraint.r2)
        world_p2_prev = body2.prevTransform.position + Quaterion.Rotate(body2.prevTransform.rotation,collisoConstraint.r2)
        
        distance = tm.dot((world_p1 - world_p2),normal)
        #handle contact
        if distance > 0 :
            delta_x = distance * normal

            dlambda_n = XPBDSolver.GetPositionDeltaLambda(body1,body2,collisoConstraint.r1,collisoConstraint.r2,
                                                        dt,delta_x,0.0,collisoConstraint.lambda_n)
            #apply collision constraint
            self.ApplyPosConstraintPair(body1,body2,collisoConstraint.r1,collisoConstraint.r2,delta_x,dlambda_n)
            collisoConstraint.lambda_n += dlambda_n

            #handle static friction
            dlambda_t = XPBDSolver.GetPositionDeltaLambda(body1,body2,collisoConstraint.r1,collisoConstraint.r2,
                                                dt,delta_x,0.0,collisoConstraint.lambda_t)
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
    def SolveVelocityWithRigibody(self,body1:ti.template(),body2:ti.template(),dt:ti.f32, n:tm.vec3,
                        r1:tm.vec3,r2:tm.vec3,lambda_n:ti.f32):
        vec = (body1.vec + tm.cross(body1.omega,r1)) - (body2.vec + tm.cross(body2.omega,r2))
        vec_n = tm.dot(n,vec)
        vec_t = vec - vec_n * n

        mu_d=(body1.dynamicFircCoeff + body2.dynamicFircCoeff) / 2
        #f_n = lambda_n / dt^2
        f_n = lambda_n / (dt**2)
        deltaVec = -tm.normalize(vec_t) * ti.min(dt * mu_d * f_n,tm.length(vec_t)) 

        e=(body1.restitutionCoeff + body2.restitutionCoeff) / 2
        if vec_n < 2 * tm.length(gravity) * dt:
            deltaVec += n * (-vec_n)
        else:
            deltaVec += n * ( -vec_n + tm.min(e*vec_n,0.0))

        normal=tm.normalize(deltaVec)
        #apply deltaVec
        w1 = body1.GetGeneralizedInvMass(normal,r1)
        w2 = body2.GetGeneralizedInvMass(normal,r2)
        #p = delta_v / (w1+w2)
        corr = deltaVec / (w1+w2)
        if body1.fixed == 0:
            body1.ApplyVecCorrection(corr,r1)
        if body2.fixed == 0:
            body1.ApplyVecCorrection(corr,r2)



    #todo
    @ti.func
    def JointsDamping(self,joints:ti.template()):
        pass

    #todo
    @ti.kernel
    def SolveVelocity(self,dt:ti.f32):
        #handle collision vec correction
        for i in ti.grouped(self.constraintField):
            if self.constraintField.conType == 1:
                body1=self.entityField[0,self.constraintField[i].body1Id]
                body2=self.entityField[0,self.constraintField[i].body2Id]
                self.SolveVelocityWithRigibody(body1,body2,dt,self.constraintField[i].contact_normal,
                                    self.constraintField[i].r1,self.constraintField[i].r2,self.constraintField[i].lambda_n)
    #### joints limit
    @staticmethod
    @ti.func
    def limitAngle(n:tm.vec3,n1:tm.vec3,n2:tm.vec3,alpha:ti.f32,beta:ti.f32) -> tm.vec3:
        rhi=tm.asin(tm.dot(tm.cross(n1,n2),n))
        if tm.dot(n1,n2) < 0:
            rhi=math.pi-rhi     
        if rhi >math.pi:
            rhi=rhi-2*math.pi
        if rhi < -math.pi:
            rhi=rhi+2*math.pi
        
        dQlimit = tm.vec3(0,0,0)
        if rhi < alpha or rhi > beta :
            rhi=tm.clamp(rhi,alpha,beta)

            #rot(n,rhi)
            q = Quaterion.SetFromAxisAngle(n,rhi)
            #n1 <- rot(n,rhi)n1
            n1 = Quaterion.Rotate(q,n1)

            dQlimit += tm.cross(n1,n2)

        return dQlimit

    #set iterter nnum
    def SetDtAndNumsubStep(self,timestep,numsubsteps):
        self.timeStep=timestep
        self.numSubSteps=numsubsteps

    @ti.kernel
    def InitConstraintLambda(self):
        for i in ti.grouped(self.constraintField):
            self.constraintField[i].lambda_total=0
    #after data init
    def Solver(self):
        dt = self.timeStep / self.numSubSteps
        totaltime=0
        if dt > 0 :
            #self.InitConstraintLambda()
            #todo:collision detect(board-phase) 
            for iter in range(self.numSubSteps):
                start_time = time.time()
                #prev pos
                self.preSolve(dt,gravity,tm.vec3(0.0,0.0,0.0))
                #constraint rigid body
                self.SolveConstraint(dt)
                #get vec
                self.UpdateAfterSolve(dt)
                #solve vecolity
                #self.SolveVelocity(dt)
                end_time = time.time()
                totaltime +=(end_time - start_time)
        
        print(f"solver time:{totaltime*1000}ms")

    #return world matrix
    @ti.kernel
    def UpdataWorldMatrix(self):
        for i in ti.grouped(self.entityField):
            self.worldField[i] = self.entityField[i].transform.GetWorldMatrix()

    def GetSolverData(self):
        self.UpdataWorldMatrix()
        return self.worldField

