import taichi as ti
import time as time
from XPBDSolver import *

ti.init(arch=ti.gpu,offline_cache=True,kernel_profiler=True)
WIDTH=1280
HEIGHT=720
timeStep = 1.0 / 60 
numSubSteps = 5

#
center_x = tm.vec3(0,0,0)

numBoxes = 2
numPendulum = 2
initPos = tm.vec3(0,2,0)
#solver rigid body data
bodyList=[]
entities=[]
constraintList=[]


#init scene
def IniBodyChainScene(numBoxes,masses,initPos=tm.vec3(0,0,0),detla_x=0):
    pos = initPos
    #init floor
    floorModel = Model("models/floor.obj",color=(0.5,0.5,0.5))
    floor = RigidBody(tm.vec3(0,-1,0),1.0,floorModel,fixed=1)
    bodyList.append(floor)
    entities.append(floor.entity)
    #init box
    boxModel = Model("models/box.obj",color=(0.5,0.5,0.5))
    bodyIdStart = len(bodyList)
    compliance= 0.01
    maxdistance = 1.0
    #add shpere
    for i in range(numBoxes):
        pos = pos + tm.vec3(0,detla_x,0)
        mass = masses[i]
        body = RigidBody(pos,mass,boxModel,inertia = RigidBody.GetBoxInertia(mass,1,1,1))
        bodyList.append(body)
        entities.append(body.entity)
        
        #add top shpere
        if i == numBoxes-1:
            spherePos = pos + tm.vec3(0.0,detla_x *2 ,0.0)
            sphere = RigidBody(spherePos,1.0,fixed=1)
            bodyList.append(sphere)
            entities.append(sphere.entity)
            
        
    for i in range(numBoxes):
        body1Id = bodyIdStart
        body2Id = body1Id + 1
        bodyIdStart = body2Id

        constraint = Constraints.field(shape=())
        constraint[None].InitPosConstraint(body1Id,body2Id,center_x,center_x,maxdistance,compliance)
        constraintList.append(constraint)


#init scene data
def InitPendulumScene(numPendulum,initPos=tm.vec3(0,0,0),delta_x = 0.6):
    pos = initPos
    #init floor
    floorModel = Model("models/floor.obj",color=(0.5,0.5,0.5))
    floor = RigidBody(tm.vec3(0,-1,0),1.0,floorModel,fixed=1)
    bodyList.append(floor)
    entities.append(floor.entity)
    lever = Model("models/lever.obj",color=(0.5,0.5,0.5))
    bodyIdStart = len(bodyList)
    compliance= 0.01
    maxdistance = 0.1
    for i in range(numPendulum):
        pos += tm.vec3(0,delta_x + maxdistance ,0)
        body = RigidBody(pos,1.0,lever,inertia=RigidBody.GetBoxInertia(1.0,0.1,0.6,0.1))
        bodyList.append(body)
        entities.append(body.entity)
                #add top shpere
        if i == numPendulum - 1:
            spherePos = pos + tm.vec3(0.0,delta_x / 2 + maxdistance ,0.0)
            sphere = RigidBody(spherePos,1.0,fixed=1)
            bodyList.append(sphere)
            entities.append(sphere.entity)

        body1Id = bodyIdStart
        body2Id = body1Id + 1
        bodyIdStart = body2Id
        
        #pos constraint
        constraint = Constraints.field(shape=())
        constraint[None].InitPosConstraint(body1Id,body2Id,center_x,center_x,maxdistance,compliance)
        constraintList.append(constraint)

        #joint constraint
        constraint = Constraints.field(shape=())
        constraint[None].InitHingeConstraint(body1Id,body2Id,center_x,center_x,1,0.0,math.pi,compliance)
        constraintList.append(constraint)

#init xpbdSolver
def InitXPBDSolver():
    xpbd = XPBDSolver(timeStep,numSubSteps,entities)
    for i in range(len(constraintList)):
        xpbd.AddStraint(constraintList[i])
    return xpbd
        

if __name__ == '__main__':
    window = ti.ui.Window("XPBD with rigid body simulation", (WIDTH, HEIGHT))
    canvas = window.get_canvas()
    scene = ti.ui.Scene()
    #init camera
    camera = ti.ui.Camera()
    camera.z_far(1000.0)
    camera.z_near(1.0)
    camera.position(0.0, 4.0, 2.0)
    camera.lookat(0.0,4.0,0.0)
    #init scene mesh
    #InitPendulumScene(2)
    masses = []
    for i in range(numBoxes):
        masses.append(10.0)
    #IniBodyChainScene(numBoxes,masses,initPos=tm.vec3(0,1,0),detla_x = 1.2)
    InitPendulumScene(numPendulum,initPos)
    #init XPBDSolver
    xpbd = InitXPBDSolver()
    frame = 0
    #main loop
    while window.running:
        scene.set_camera(camera)
        camera.track_user_inputs(window, movement_speed=0.03, hold_key=ti.ui.RMB)
        scene.ambient_light((0.7,0.7,0.7))
        scene.point_light(pos=(0, 10, 0), color=(1.0,1.0,1.0))

        start_time = time.time()
        ################################## logic tick #######################################
        #logic tick
        xpbd.Solver()
        worldMatData=xpbd.GetSolverData()
        ###################################  render tick  ####################################
        #add mesh to sence(entities)
        for i in range(len(bodyList)):
            worldM = worldMatData[0,i]
            if bodyList[i].model == None:
                print(bodyList[i].worldVectices)
                scene.particles(bodyList[i].worldVectices,0.05)
            else:    
                vertices,indices,normals,color = bodyList[i].GetSceneData(worldM)
                scene.mesh(vertices,indices,normals=normals,color=color)
        

        end_time = time.time()
        print(f"frame: {frame}, time={(end_time - start_time)*1000:03f}ms")
        canvas.scene(scene)
        window.show()
        frame +=1

        # if frame==2:
        #      window.running = False



#ti.profiler.print_kernel_profiler_info()  # The default mode: 'count'
