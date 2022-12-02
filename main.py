import taichi as ti
import time as time
from XPBDSolver import *

WIDTH=1280
HEIGHT=720
timeStep = 1.0 / 60 
numSubSteps = 20

#solver rigid body data
bodyList=[]
constraintList=[]

#init scene
def InitBoxHingeScene(numBoxes,masses,initPos=tm.vec3(0,0,0),detla_x=0):
    #init floor
    floorModel = Model("models/floor.obj",color=(0.5,0.5,0.5))
    floor = RigidBody(floorModel,tm.vec3(0,-1,0),1.0,fixed=1)
    bodyList.append(floor)
    #init box
    boxModel = Model("models/box.obj",color=(0.5,0.5,0.5))
    for i in range(numBoxes):
        pos = initPos + tm.vec3(detla_x*i,0,0)
        mass = masses[i]
        body = RigidBody(boxModel,pos,mass,inertia = RigidBody.GetBoxInertia(mass,1,1,1))
        bodyList.append(body)

#init scene data
def InitPendulumScene(numPendulum,initPos=tm.vec3(0,0,0)):
    #init floor
    floorModel = Model("models/lever.obj",color=(0.5,0.5,0.5))
    floor = RigidBody(floorModel,tm.vec3(0,0,0),1000.0)
    staticBodyList.append(floor)
    lever = Model("models/lever.obj",color=(0.5,0.5,0.5))
    for i in range(numPendulum):
        pos = initPos + tm.vec3(0,0.65*i,0)
        body= RigidBody(lever,pos,1.0,RigidBody.GetBoxInertia(1.0,0.6,0.1,0.1))
        rigidBodyList.append(body)

    # vertices,indices,color = boxes.GetSceneMeshData()
    # entityList.append(body)
    #load model

#init xpbdSolver
def InitXPBDSolver():
    xpbd = XPBDSolver(timeStep,numSubSteps)
    for i in range(len(bodyList)):
        xpbd.AddEntity(bodyList[i].entity)
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
    camera.position(0.0, 2.5, -5.0)
    #init scene mesh
    #InitPendulumScene(2)
    masses = [1.0,1.0,1.0,1.0,1.0]
    InitBoxHingeScene(1,masses,initPos=tm.vec3(0,4,0),detla_x= 2.0)
    #init XPBDSolver
    xpbd = InitXPBDSolver()
    
    frame = 0
    #main loop
    while window.running:
        camera.track_user_inputs(window, movement_speed=0.03, hold_key=ti.ui.RMB)
        scene.set_camera(camera)
        scene.ambient_light((0.7,0.7,0.7))
        scene.point_light(pos=(0, 10, 0), color=(1.0,1.0,1.0))

        start_time = time.time()
        ################################## logic tick #######################################
        #logic tick
        xpbd.Solver()
        worldMatData=xpbd.GetSolverData()
        ###################################  render tick  #################################33
        #add mesh to sence(entities)
        for i in range(len(bodyList)):
            worldM = worldMatData[0,i]
            vertices,indices,color = bodyList[i].GetSceneData(worldM)
            scene.mesh(vertices,indices,color=color)

        end_time = time.time()
        print(f"frame: {frame}, time={(end_time - start_time)*1000:03f}ms")
        canvas.scene(scene)
        window.show()
        frame +=1



#ti.profiler.print_kernel_profiler_info()  # The default mode: 'count'

