import taichi as ti

from Constraints import *

ti.init()

#pos=PositonConstaints.field()
col=Constraints.field()
# # col=CollisionConstraint.field()
# # joints=JointsConstraint.field()

root=ti.root.dense(ti.i,1)
tree=root.dynamic(ti.j,1024)
tree.place(col)


# # for i in range(3):
# #     pos1=PositonConstaints()
# #     ti.append(constraintTree,i,pos1)
# # constraintTree.dynamic(PositonConstaints,)

@ti.func
def test(c:ti.template()):
    c.jointType =1 

@ti.kernel
def init():
    col1 = Constraints()
    ti.append(tree,0,col1)
    for i in ti.grouped(col):
        print(i)
        test(col[i])
        print(col[i].jointType)
    
init()