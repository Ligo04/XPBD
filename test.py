import taichi as ti

ti.init(ti.cpu, kernel_profiler=True)
n = 128
var = ti.field(ti.f32, shape=n)

@ti.kernel
def fill():
    for i in range(n):
        var[i] = 0.1

window = ti.ui.Window("XPBD with rigid body simulation", (720, 720))
while window.running:
    fill()
    ti.profiler.clear_kernel_profiler_info() #[1]
    for i in range(20):
        fill()
    query_result = ti.profiler.query_kernel_profiler_info(fill.__name__) #[2]
    print("kernel executed times =",query_result.counter)
    print("kernel elapsed time(min_in_ms) =",query_result.min)
    print("kernel elapsed time(max_in_ms) =",query_result.max)
    print("kernel elapsed time(avg_in_ms) =",query_result.avg)
    window.show()