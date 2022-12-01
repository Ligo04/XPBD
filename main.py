import taichi as ti
from XPBDSolver import *

WIDTH=1024
HEIGHT=1024


#init scene data
def Init2PendulumScene():
    ribodyList = []
    #load model
    boxes=Model(material=0,color=(0.5,0.5,0.5))
    boxes.from_obj("models/floor.obj")
    body=RigidBody(boxes,tm.vec3(0,0,0),10.0)
    ribodyList.append(body)

##def AddMeshToScene(scene:ti.ui.Scene,model:Model):


if __name__ == '__main__':
    window = ti.ui.Window("XPBD with rigid body simulation", (WIDTH, HEIGHT))
    canvas = window.get_canvas()
    scene = ti.ui.Scene()
    #init camera
    camera = ti.ui.Camera()
    camera.position(0,0,-5)
    camera.lookat(0,0,0)
    camera.z_far(1000.0)
    camera.z_near(1.0)
    boxes=Model(material=0,color=(0.5,0.5,0.5))
    boxes.from_obj("models/floor.obj")
    body=RigidBody(boxes,tm.vec3(0,0,0),10.0)
    vertices,indices,color = body.GetSceneMeshData()
    scene.mesh(vertices,indices,color=color)

    #main loop
    while window.running:
        scene.set_camera(camera)
        scene.ambient_light((0.5, 0.5, 0.5))
        scene.point_light(pos=(0.5, 1.5, 0.5), color=(1, 1, 1))
        scene.point_light(pos=(0.5, 1.5, 1.5), color=(1, 1, 1))
        
        scene.mesh()
        canvas.scene(scene)
        window.show()

