import taichi as ti
from XPBDSolver import *

WIDTH=1280
HEIGHT=720


#solver rigid body data
staticBodyList=[]
rigidBodyList=[]
posConstraintList=[]
jointsConstraintList=[]
collisionConstrainList=[]


#xpbd solver
xpbd = XPBDSolver()
xpbd.SetDtAndNumsubStep(1.0/60.0,40)

#transfomr field
solverWorldMatrixs=None

#init scene
def InitBoxHingeScene(numBoxes,masses,initPos=tm.vec3(0,0,0),detla_x=0):
    #init floor
    floorModel = Model("models/floor.obj",color=(0.5,0.5,0.5))
    floor = RigidBody(floorModel,tm.vec3(0,-1,0),1.0)
    staticBodyList.append(floor)
    #init box
    boxModel = Model("models/box.obj",color=(0.5,0.5,0.5))
    for i in range(numBoxes):
        pos = initPos + tm.vec3(detla_x*i,0,0)
        mass = masses[i]
        body = RigidBody(boxModel,pos,mass,RigidBody.GetBoxInertia(mass,1,1,1))
        rigidBodyList.append(body)

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
    entityFiled = Entity.field(shape=len(rigidBodyList))
    for i in range(len(rigidBodyList)):
        entityFiled[i]=rigidBodyList[i].entity

    #pos constraint
    # posConstraintFiled = Entity.field(shape=len(posConstraintList))
    # for i in range(len(posConstraintList)):
    #     posConstraintFiled[i] = posConstraintList[i]
    # XPBDSolver.InitConstraint(0,posConstraintFiled)
    # #jointConstrintFiled
    # jointConstraintFiled = Entity.field(shape=len(jointsConstraintList))
    # for i in range(len(jointsConstraintList)):
    #     jointConstraintFiled[i] = jointsConstraintList[i]
    # XPBDSolver.InitConstraint(1,jointConstraintFiled)
    # #collision ocnstraint
    # colConstraintFiled = Entity.field(shape=len(collisionConstrainList))
    # for i in range(len(collisionConstrainList)):
    #     colConstraintFiled[i] = collisionConstrainList[i]
    # XPBDSolver.InitConstraint(2,posConstraintFiled)
        

if __name__ == '__main__':
    window = ti.ui.Window("XPBD with rigid body simulation", (WIDTH, HEIGHT))
    canvas = window.get_canvas()
    scene = ti.ui.Scene()
    #init camera
    camera = ti.ui.Camera()
    camera.z_far(1000.0)
    camera.z_near(1.0)
    camera.position(0.0, 2.5, -5.0)
    camera.lookat(0.0, 4, 0)
    #init scene mesh
    #InitPendulumScene(2)
    InitBoxHingeScene(2,[1.0,1.0],initPos=tm.vec3(0,4,0),detla_x= 2.0)
    #init XPBDSolver

    #InitXPBDSolver()
    #main loop
    while window.running:
        camera.track_user_inputs(window, movement_speed=0.03, hold_key=ti.ui.RMB)
        scene.set_camera(camera)
        scene.ambient_light((0.7,0.7,0.7))
        scene.point_light(pos=(0, 10, 0), color=(1.0,1.0,1.0))
        

        ################################## logic tick #######################################
        #logic tick
        # xpbd.Simulate()
        # xpbd.GetSolveredData(solverWorldMatrixs)

        ###################################  render tick  #################################33
        #add mesh to sence(static):
        for i in range(len(staticBodyList)):
            vertices,indices,color=staticBodyList[i].GetSceneData()
            scene.mesh(vertices,indices,color=color)
        #add mesh to sence(rigid)
        for i in range(len(rigidBodyList)):
            #rigidBodyList[i].SetTransform(solverWorldMatrixs[i])
            vertices,indices,color=rigidBodyList[i].GetSceneData()
            scene.mesh(vertices,indices,color=color)


        canvas.scene(scene)
        window.show()

