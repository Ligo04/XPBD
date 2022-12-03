import taichi as ti
from Constraints import *

ti.init(kernel_profiler=True)
cons = Constraints.field(shape=10)
cons.fill(1)

@ti.kernel
def test(temp:ti.template()):
    for i in ti.grouped(temp):
        print(temp[i].body1Id)


test(cons)
ti.profiler.print_kernel_profiler_info()