
[toc]
# 目录

## 3 Position Based Rigid body simulation

### 3.1 Particale simulation loop 

下图算法给出了基本的PBD模拟粒子的的循环过程。

![image-20221122172532019](https://img2022.cnblogs.com/blog/1656870/202211/1656870-20221122172532829-1457369361.png)

### 3.2 Rigid Body Simulation Loop

粒子一般单独用位置$x$,速度$v$以及质量$m$描述，但是对于有限体积的刚体也包括一些角量，如：

- 单位四元数描述它的方向（$q\in R^4,|q|=1$）
- 角速度$\omega \in R^3$
- 惯性张量$I \in R^{3\times3}$

其中角速度向量可以分为单位选择轴和标量角速度：
$$
\omega=\omega\cdot n_{rot}
$$
惯性张量$I$是与转动量中质量相对应的量。

对于盒子以及球体一些基本的形状，可以用简单的公式给出。如下：

- 球体：
  $$
  I=\left[\begin{array}{ccc}
  \frac{2}{5}mr^2 & 0 & 0 \\
  0 & \frac{2}{5}mr^2 & 0 \\
  0 & 0 & \frac{2}{5}mr^2
  \end{array}\right]
  $$

- 盒子：width$\ w$ ,height$\ h$，depth$\ d$
  $$
  I=\left[\begin{array}{ccc}
  \frac{1}{12}m(h^2+d^2) & 0 & 0 \\
  0 & \frac{1}{12}m(w^2+h^2) & 0 \\
  0 & 0 & \frac{1}{12}m(w^2+d^2)
  \end{array}\right]
  $$
  其中可在[wiki百科](https://zh.wikipedia.org/zh-hans/%E8%BD%89%E5%8B%95%E6%85%A3%E9%87%8F%E5%88%97%E8%A1%A8)上找到公式

但是用闭合三角形网格描述的物体的一般情况，Blow和Binstock [BB04]设计了一种优雅而简短的算法。[具体笔记在这](https://www.cnblogs.com/Ligo-Z/p/16921551.html)。

下面算法2是算法1的扩展版本，它考虑了这些额外的量。

![image-20221122173403559](https://img2022.cnblogs.com/blog/1656870/202211/1656870-20221122173404406-327785877.png)

积分角速度的公式（包括力矩以及陀螺效应）可以从牛顿-欧拉方程推导出来(Brian Mirtich’s excellent thesis [Mir96] f)。

- 推荐看下王华民教授的Games103关于刚体模拟的部分

  ![image-20221122173913643](https://img2022.cnblogs.com/blog/1656870/202211/1656870-20221122173914516-1411810870.png)

这种基于位置的求解器允许同时和耦合模拟刚体和可变形物体。为了实现其我们可以迭代所有物体以及粒子，然后对于粒子，我们可以简单地省略转动量的更新。

当然，该论文所采用的基础算法是substep XPBD[MSL*19]

![image-20221123152824939](https://img2022.cnblogs.com/blog/1656870/202211/1656870-20221123152828185-1459408985.png)

### 3.2 Core Projection Operations

本文提出了在位置框架上的两个基本操作。求解任意关节，处理接触或者耦合刚体和软体的任务都可以独立建立在这两个基本操作上。

![image-20221122191951213](https://img2022.cnblogs.com/blog/1656870/202211/1656870-20221122191952121-132200455.png)

上图表示了基本的修正操作（传统的基于粒子的PBD）：一对粒子上进行位置修正向量$\Delta x$，这将会产生位置修正$\Delta x_1$和$\Delta x_2$,其与粒子的质量的逆成比例，从而使线动量和角动量守恒。、

**位置约束（position constraintss）**

![image-20221122192416787](https://img2022.cnblogs.com/blog/1656870/202211/1656870-20221122192417718-985621428.png)

上图表示的是第一个核心的投影操作：一对刚体上的点$r_1$以及$r_2$进行位置修正$\Delta x$，这将会产生对质心位置应用位置修正$\Delta x_1$和$\Delta x_2$以及对刚体应用旋转修正$\Delta q_1$和$\Delta q_2$，其与他们的质量的逆以及转动惯量的逆的组合成比例。

为了在点$r_1$以及$r_2$进行位置修正$\Delta x$,我们首先将其分为方向$n$以及大小$c$。大小$c$与PBD中的约束函数的值有关。然后我们计算两个广义质量逆：
$$
\begin{array}{l}
w_{1} \leftarrow \frac{1}{m_{1}}+\left(\mathbf{r}_{1} \times \mathbf{n}\right)^{T} \mathbf{I}_{1}^{-1}\left(\mathbf{r}_{1} \times \mathbf{n}\right) \\
w_{2} \leftarrow \frac{1}{m_{2}}+\left(\mathbf{r}_{2} \times \mathbf{n}\right)^{T} \mathbf{I}_{2}^{-1}\left(\mathbf{r}_{2} \times \mathbf{n}\right)
\end{array}
$$
在XPBD之后，我们可以更新拉格郎日乘子：
$$
\begin{aligned}
\Delta \lambda & \leftarrow \frac{-c-\tilde{\alpha} \lambda}{w_{1}+w_{2}+\tilde{\alpha}} \\
\lambda & \leftarrow \lambda+\Delta \lambda
\end{aligned}
$$
其中$\tilde{\alpha}=\alpha/h^2$和$\alpha$是柔度约束。每个符合要求的约束都会存储一个拉格朗日乘子$\lambda$，在迭代求解开始之前设置为零。

柔性对应的是刚性的逆并且单位是米/牛顿（meters/Newtow）。我们允许设置柔性$\alpha=0$来模拟无限刚性约束。

设置位置冲量$p=\Delta \lambda n$,我们在每个约束求解后立即更新物体的状态：
$$
\begin{array}{l}
\mathbf{x}_{1} \leftarrow \mathbf{x}_{1}+\mathbf{p} / m_{1} \\
\mathbf{x}_{2} \leftarrow \mathbf{x}_{2}-\mathbf{p} / m_{2} \\
\mathbf{q}_{1} \leftarrow \mathbf{q}_{1}+\frac{1}{2}\left[\mathbf{I}_{1}^{-1}\left(\mathbf{r}_{1} \times \mathbf{p}\right), 0\right] \mathbf{q}_{1} \\
\mathbf{q}_{2} \leftarrow \mathbf{q}_{2}-\frac{1}{2}\left[\mathbf{I}_{2}^{-1}\left(\mathbf{r}_{2} \times \mathbf{p}\right), 0\right] \mathbf{q}_{2}
\end{array}
$$
**注意在第二主体上的更新中我们采用负号。**

在处理每个约束之后我们立即更新可以防止overshooting，这也是PBD健壮性的原因之一。同时这就产生了一个非线性的高斯-赛德尔投影解。当然我们可以使用雅可比（Jacobi）解用于并行实现或消除对约束投影顺序的依赖，但是代价是收敛速度比较慢。

求解后，沿约束作用的力可以由下面的公式得到：
$$
f=\lambda n/h^2
$$
对于连接一个定义为粒子系统的软体以及一个刚体是非常简单的，因为每个物体能够用一个粒子表示，通过设置$w\larr m^{-1}$并且忽略方向的更新。

**角度约束（angular constraintss）**

![image-20221122193203505](https://img2022.cnblogs.com/blog/1656870/202211/1656870-20221122193204391-1959089986.png)

上图表示的第二个核心的投影操作：一对刚体进行旋转修正——使他们的方向对齐，这将会产生旋转修正$\Delta q_1$以及$\Delta q_2$,其与他们的的转动惯量的逆成比例，而质心不受这旋转修正的影响。

对于关节，我们需要约束两个物体相互方向的能力，因为我们需要上述的旋转修正。为了应用该旋转旋转修正$\Delta q \in R^3$,我们将其分为方向$n$，表示为旋转轴，以及它的大小$\theta$，表示为旋转角度。所以广义质量逆是：
$$
\begin{array}{l}
w_{1} \leftarrow \mathbf{n}^{T} \mathbf{I}_{1}^{-1}\mathbf{n}\\
w_{2} \leftarrow \mathbf{n}^{T} \mathbf{I}_{2}^{-1}\mathbf{n}
\end{array}
$$
XPBD的更新和上面的位置约束一样，只不过用角度代替了距离：
$$
\begin{aligned}
\Delta \lambda & \leftarrow \frac{-\theta-\tilde{\alpha} \lambda}{w_{1}+w_{2}+\tilde{\alpha}} \\
\lambda & \leftarrow \lambda+\Delta \lambda
\end{aligned}
$$
这里，修正只影响方向：
$$
\begin{array}{l}
\mathbf{q}_{1} \leftarrow \mathbf{q}_{1}+\frac{1}{2}\left[\mathbf{I}_{1}^{-1}\left(\mathbf{r}_{1} \times \mathbf{p}\right), 0\right] \mathbf{q}_{1} \\
\mathbf{q}_{2} \leftarrow \mathbf{q}_{2}-\frac{1}{2}\left[\mathbf{I}_{2}^{-1}\left(\mathbf{r}_{2} \times \mathbf{p}\right), 0\right] \mathbf{q}_{2}
\end{array}
$$
**惯性张量$I$取决于物体的实际方向**。因此必须在每个约束投影之后更新。相反，我们在计算上述表达式之前，我们将$n,r$以及$p$投射到物体的静息状态上（the rest state of bodies）。

对于关节来说，接触点$r$通常定义为静止状态（静息）。此外，我们在静止（静息）状态下旋转物体，使惯性张量变为对角线，这简化了上面的表达式，并允许将张量存储为一个向量。

与上面类似，我们可以推导出施加的力矩：
$$
\tau=\lambda n/h^2
$$

### 3.4 Joints

我们现在描述如何使用上述两个修正操作处理各种关节。关节连接成对的物体，冰限制物体的相对位置和旋转程度。

#### 3.4.1 Rotation Degrees of Freedom

对于使两个物体相互对齐的关节，我们计算角度修正如下：
$$
\begin{array}{l}
q=q_1q_2^{-1} \\
\Delta q_{fixed}=2(q_x,q_y,q_z)
\end{array}
$$
为了建立的更一般的关节，我们必须分别在物体上都定义一个接触点$\overline {r}$以及一组相互垂直的单位轴$[\overline{a},\overline{b},\overline{c}]$，他们首先被转化到世界向量$r$和$[a,b,c]$

对于链接关节，我们希望轴$a_1$和$a_2$对齐，因此我们应用：
$$
\Delta q_{hinge}=a_1 \times a_2
$$
为了驱动链接关节到一个特定角度$\alpha$，我们绕$a_1$旋转$b_1$一个角度$\alpha$得到$b_{target}$，然后应用：
$$
\Delta q_{target}=b_{target} \times b_2
$$
相关的柔度$\alpha$控制约束的刚性。有一个目标角度约束，我们可以通过更新目标角度$\alpha \larr \alpha +h$在每一个子步（substep）创建一个速度驱动的马达，其中$v$是马达的目标速度以及反应了柔性的大小。

处理关节限制是刚体引擎中的关键一部分。对于旋转自由度来说，这相当于限制了关节的角度。

我们使用算法3定义的通用过程，它将两个物体的轴$n_1$和$n_2$限制在使用一个共同的旋转轴$n$的角度范围$[\alpha,\beta]$之间。

![image-20221123093153072](https://img2022.cnblogs.com/blog/1656870/202211/1656870-20221123093156089-100891590.png)

对于使用共同旋转轴$a_1=a_2$的铰链关节，我们使用$[n，n_1,n_2]=[a_1,b_1,b_2]$。

对于球状关节（也叫做球窝接头），我们必须能够对轴$a_2$相对于$a_1$的运动之中区分出摆动（swing）和扭转（Twist）限制。为了限制摆动，我们使用$[n,n_1,n_2]=[a_1 \times a_2,a_1,a_2]$。同时扭转必须与摆动解耦，我们通过以下轴进行实现：
$$
\begin{array}{l}
n \larr (a_1+a_2)/(|a_1+a_2|)\\
n_1 \larr b_1 - (n\cdot b_1)n;\ n_1 \larr |n_1|\\
n_2 \larr b_2 - (n\cdot b_2)n;\ n_2 \larr |n_2|
\end{array}
$$
所以限制可以通过设置$\alpha > 0$变得柔软。

#### 3.4.2 Position Degrees of Freedom

处理位置自由度更简单，我们首先计算位置偏移$\Delta r=r_2-r_1$。设置$\Delta x=\Delta r$从而不分离的情况下连接物体，这是关节的典型情况。使用$\alpha >0$允许一个零静息长度的弹簧。我们能够定义分离距离的上限$d_{max}$使其变得更灵活。在这种情况下，我们只有当$|\Delta r|>d_{max}$才应用修正：
$$
\Delta x=\frac{\Delta r}{|\Delta r|}(|\Delta r|-d_{max})
$$
我们也可以允许物体在边界内沿轴的一个子集移动来放松固定的附着。为此，我们从$\Delta x$开始。对于第一个轴$a_1$,我们计算在其上的投影位移$a=\Delta r \cdot a_1$。如果$a<a_{min}$,我们添加$a_1(a-a_{min})$到修正向量上，如果$a>a_{max}$,我们添加的是$a_1(a-a_{max})$。在应用最后的修正向量之前，我们对所有轴和极限都这样做。这样，所有限制都用了一个单一的约束投影来处理。

将除了第一个轴外的所有限制设为零，这将模拟了一个移动关节（prismatic joints）。

对于一个机器人，我们可能想要驱动关节到一个特定的偏移量。通过将$d_{max}$替换成$d_{target}$，兵无条件地应用校正实现了这一点。

选择柔性$\alpha=\frac{1}{f}(|\Delta r|-d_{target})$来应用一个力$f$.

关节处理显示了非线性高斯-赛德尔方法在位置层上的优势。单边约束只需在某些条件成立时应用修正即可。而且，校正总是与当前的偏移和误差保持一致。此外，在线性化求解器中，附着是用一个约束来处理的，而不是三个约束。

### 3.5 Handing contacts and frrction

为了减少计算花费，我们使用AABB树型结构在每一个时间步中（time step）收集潜在的碰撞对，而不是每一个子步（sub-step）。我们通过一个距离$k\Delta t v_{body}$扩展AABB，其中$k\ge 1$是一个安全的乘数，考虑了时间步中的潜在加速度。在我们的例子中，使用$k=2$。

在每个子步中，我们迭代所有碰撞对检查实际的碰撞。如果碰撞发生，我们计算当前的接触法线以及两个物体的局部的接触点$r_1$和$r_2$。我们同时初始化两个拉格朗日乘数用于法向以及切线的力$\lambda _n$和$\lambda _t$为零。为了在位置上处理接触求解，我们计算两个物体基于当前状态以及在子步积分之前的接触位置：
$$
\begin{array}{l}
p_1=x_1+q_1r_1 \\
p_2=x_2+q_2r_2 \\
\overline {p}_1=x_{1,prve}+q_{1,prev}r_1 \\
\overline {p}_2=x_{2,prve}+q_{2,prev}r_2 
\end{array}
$$
其中四元数与向量的乘积指的是使用四元数旋转向量。

当前计算出的穿透为$d=(p_1-p_2)\cdot n$,如果$d\le 0$，我们跳过该接触。

非线性高斯-塞德尔解算器让我们通过简单地检查每个约束的基础来处理互补性条件。如果两个物体正在穿透，我们使用$\alpha=0$和$\lambda _n$并应用$\Delta x=dn$。

为了处理静摩擦，我们计算在接触点的相对运动以及它的切向分量：
$$
\begin{array}{l}
\Delta p=(p_1-\overline{p}_1)-(p_2-\overline{p}_2) \\
\Delta p_t=\Delta p-(\Delta p \cdot n)n
\end{array}
$$
静摩擦力阻止了在接触点的切向运动，如果$\Delta p_t=0$就会出现这种情况。因此，为了强制执行静摩擦，我们在接触点上应用$\Delta x=\Delta p_t$，其中$\alpha =0$  ，但只有在$\lambda _t <\mu _s \lambda _n$，其中$\mu_s$是静摩擦系数。如果两个物体有不同的系数，我们使用$\mu=(\mu_1+\mu_1)/2$。另一个选择是取最大值或最小值。

### 3.5 Velocity level

PBD在位置求解之后更新速度并且立即进行下一子步。然而，处理动摩擦以及恢复，我们附加了一个速度解，如算法2所示。在这里，我们对所有的接触点进行一次迭代，并更新新的速度。

对于每个接触对，我们在接触点上计算对应的法线和切线速度：
$$
\begin{array}{l}
v \larr (v_1+\omega_1 \times r_1)-(v_2+\omega_2 \times r_2) \\
v_n \larr n \cdot v\\
v_t \larr v-nv_n
\end{array}
$$
通过计算速度对摩擦力进行显式积分：
$$
\Delta v \larr -\frac{v_t}{|v_t|}min(h\mu_d|f_n|,|v_t|)
$$
其中$\mu_d$是动摩擦系数，$f_n=\lambda_b /h^2$是法向力。这种更新对应与动态库伦摩擦力的明确应用。

与高斯-赛德尔更新相联系的显式形式使我们能够使这一步无条件地稳定! 这个最小值保证了速度修正的大小永远不会超过速度本身的大小。

我们还可以利用速度通道来施加关节阻尼：
$$
\begin{array}{l}
\Delta v \larr (v_2-v_1)min(\mu_{lin}h,1) \\
\Delta \omega \larr (\omega_2-\omega_1)min(\mu_{ang}h,1)
\end{array}
$$
根据附录的推导，在位置$r_1$和$r_2$应用速度更新$\Delta v$是通过以下步骤实现的：
$$
\begin{aligned}
\mathbf{p} &=\frac{\Delta \mathbf{v}}{w_{1}+w_{2}} \\
\mathbf{v}_{1} & \leftarrow \mathbf{v}_{1}+\mathbf{p} / m_{1} \\
\mathbf{v}_{2} & \leftarrow \mathbf{v}_{2}-\mathbf{p} / m_{2} \\
\omega_{1} & \leftarrow \omega_{1}+\mathbf{I}_{1}^{-1}\left(\mathbf{r}_{1} \times \mathbf{p}\right) \\
\omega_{2} & \leftarrow \omega_{2}-\mathbf{I}_{2}^{-1}\left(\mathbf{r}_{2} \times \mathbf{p}\right)
\end{aligned}
$$
为了处理恢复，我们需要$\overline {v}_n$，即PBD速度更新之前的法线速度。我们通过对更新前的速度应用之前的公式计算出法线速度。考虑到恢复系数$e$，我们希望在接触出的法线速度是$-e\overline {v}_n$,通过应用：
$$
\Delta v \larr n(-v_n+min(-e \overline{v}_n,0))
$$
我们减去当前速度$v_n$并将将其换成反射速度$-e\overline {v}_n$，这样确保了产生的速度指向碰撞法线方向。

为了避免颤抖，如果$|v_n|$非常小，我们设$e=0$.我们使用一个阈值$|v_n|\le 2|g|h$，其中$g$是重力。这个值相当于预测步骤因重力加速度而增加的速度的两倍。

这一步骤也缓解了PBD的一个重要问题。PBD的常规速度更新步骤产生的速度，只有在最后一个时间步骤中没有发生碰撞时才有意义。否则，他们只是反映了穿透深度，而穿透深度又取决于轨迹的离散性。另外，如果物体是在重叠状态下产生的，PBD会产生较大的分离速度。上述公式消除了除碰撞外的派生速度，并考虑到了恢复系数的前一个时间步长的速度来代替它。在物体最初重叠的情况下，这个速度为零。

# 参考文献

[MSL19] MACKLIN M., STOREY K., LU M., TERDIMAN P., CHENTANEZN., JESCHKE S., MÜLLER M.: Small steps in physics simulation.In Proceedings of the 18th Annual ACM SIGGRAPH Eurographics Symposium on Computer Animation (New York, NY, USA, 2019), SCA’19, Association for Computing Machinery.