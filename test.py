import taichi as ti
from Entity import *
ti.init()

# x=Entity.field()
# ti.root.dense(ti.i,1000000).place(x)
# total=Entity.field()
# ti.root.dense(ti.i,1).place(total)
# x.fill(1)

# @ti.func
# def test1(total:ti.template(),x:ti.f32):
#     total.transform.position = tm.vec3(x,x,x)

# @ti.kernel  
# def sum():
#     for i in x:
#         #test1(total,x[i])
#         # Approach 1: OK
#         #total[None] += x[i]
  
#         # # Approach 2: OK
#         # ti.atomic_add(total[None], x[i])

#         # # Approach 3: Wrong result since the operation is not atomic.
#         #total[None] = total[None] + x[i]
#         test1(total[0],i)
#     print(total[0].transform.position.x)
#     print(total[0].transform.position.y)
#     print(total[0].transform.position.z)

x=tm.vec3(1,0,0)
y=tm.vec3(0,1,0)
z=tm.vec3(0,0,1)

total = ((x+y).normalized() + z).normalized()
print(total.x)
print(total.y)
print(total.z)

total1 = (x+y+z).normalized()
print(total1.x)
print(total1.y)
print(total1.z)

m=ti.Matrix()

