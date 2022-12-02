import taichi as ti

ti.init(arch=ti.cpu, kernel_profiler=True)
x = ti.field(ti.f32, shape=1024*1024)

@ti.kernel
def fill():
    for i in x:
        x[i] = i

for i in range(8):
    fill()
ti.profiler.print_kernel_profiler_info('trace')
ti.profiler.clear_kernel_profiler_info()  # Clears all records

for i in range(100):
    fill()
ti.profiler.print_kernel_profiler_info()  # The default mode: 'count'