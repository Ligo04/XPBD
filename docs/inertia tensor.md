[toc]

# 算法流程

1. 任意选择一个参考点
2. 对于在mesh上的每个三角形：
   - 形成由参考点与该三角形所围成的四面体
   - 计算四面体的协方差（通过仿射变换映射到一个已知的正则四面体
   - 将这个协方差加到累加器中
3. 将累加器从协方差矩阵转换为惯性张量

固定的惯性张量可以表示为：
$$
I=1_3\ tr\ C\ -C
$$
其中$1_3$是$3 \times 3$的单位矩阵，以及$C$是物体的质量权重的协方差矩阵。
$$
C=\sum_{n} m_n X_n X_n^T
$$
C是一个封闭、简单的几何体。

## 正则四面体的协方差

正则四面体有四个顶点$v_0=(0,0,0),v_1=(1,0,0),v_2=(0,1,0),v_3=(0,0,1)$,我们首先得到协方差矩阵的连续形式：
$$

\mathbf{C}=\int_{V} \rho \mathbf{x} \mathbf{x}^{\mathrm{T}} d V=\rho \int_{V} \mathbf{x} \mathbf{x}^{\mathrm{T}} d V
$$
其中$\rho$是物体的密度（常量）.其中$C$是一个$3 \times 3$的对称矩阵，只包含两个分量。

因此我们只需要求解两个分量$C_{xx},C_{xy}$即可，考虑正则四面体的密度为1,我们有：
$$
\begin{array}{c}
C_{x x}=\int_{0}^{1} \int_{0}^{z} \int_{0}^{y} x^{2} d x d y d z=\frac{1}{60} \\
C_{x y}=\int_{0}^{1} \int_{0}^{z-y} \int_{0}^{z-y} x y d x d y d z=\frac{1}{120}
\end{array}
$$
因此，
$$
\mathbf{C}_{\text {canonical }}=\left[\begin{array}{ccc}
\frac{1}{60} & \frac{1}{120} & \frac{1}{120} \\
\frac{1}{120} & \frac{1}{60} & \frac{1}{120} \\
\frac{1}{120} & \frac{1}{120} & \frac{1}{60}
\end{array}\right]
$$

## 变换协方差矩阵

为了映射${C}_{\text {canonical }}$到四面体，我们需要用两个方式操作协方差矩阵。

- 我们通过对每个输入点应该线性变换A,从而得到变换后的矩阵:
  $$
  \begin{aligned}
  \mathbf{C}^{\prime} &=\rho \int_{V}(\mathbf{A} \mathbf{x})(\mathbf{A} \mathbf{x})^{\mathrm{T}} d V_{(\mathbf{A})} \\
  &=\rho \int_{V} \mathbf{A x} \mathbf{x}^{\mathrm{T}} \mathbf{A}^{\mathrm{T}} d V \operatorname{det} \mathbf{A} \\
  &=(\operatorname{det} \mathbf{A}) \mathbf{A} \mathbf{C A}^{\mathrm{T}}
  \end{aligned}
  $$
  $det A$因子描述：变换A扭曲了积分的体积元（$d V_{(A)}=dV\ det A$）

- 我们通过对每个输入点应用偏移$\Delta x$,从而得到偏移后的矩阵：
  $$
  \begin{aligned}
  \mathbf{C}^{\prime} &=\operatorname{Translate}(\mathbf{C}, \boldsymbol{\Delta} \mathbf{x}) \\
  &=\rho \int_{V}(\mathbf{x}+\boldsymbol{\Delta} \mathbf{x})(\mathbf{x}+\boldsymbol{\Delta} \mathbf{x})^{\mathrm{T}} d V \\
  &=\rho \int_{V}\left(\mathbf{x x}^{\mathrm{T}}+\boldsymbol{\Delta} \mathbf{x} \mathbf{x}^{\mathrm{T}}+\mathbf{x} \boldsymbol{\Delta} \mathbf{x}^{\mathrm{T}}+\Delta \mathbf{x} \boldsymbol{\Delta} \mathbf{x}^{\mathrm{T}}\right) d V \\
  &=\rho \int_{V} \mathbf{x x}^{\mathrm{T}} d V+\rho \boldsymbol{\Delta} \mathbf{x} \int_{V} \mathbf{x}^{\mathrm{T}} d V+\rho \int_{V} \mathbf{x} d V \Delta \mathbf{x}^{\mathrm{T}}+\rho \Delta \mathbf{x} \Delta \mathbf{x}^{\mathrm{T}} \int_{V} d V \\
  &=\mathbf{C}+m\left(\boldsymbol{\Delta} \overline{\mathbf{x}}^{\mathrm{T}}+\overline{\mathbf{x}} \boldsymbol{\Delta} \mathbf{x}^{\mathrm{T}}+\Delta \mathbf{x} \Delta \mathbf{\Delta} \mathbf{x}^{\mathrm{T}}\right)
  \end{aligned}
  $$
  其中$\overline{\mathbf{x}}$是输入点的质心用于构造C，以及$m=\rho V$是这些点的总质量。

## 正则四面体映射到目标四面体

我们首先开始于一个正则四面体，通过线性变换使其与目标形状相同，然后平移它使其在相同的位置。

正则四面体的顶点表示为：$\{v_0,v_1,v_2,v_3\}$，目标四面体的顶点表示为$\{w_0,w_1,w_2,w_3\}$

为了使四面体的形状相同，我们定义以下线性变换：
$$
\mathbf{A}\left[\mathbf{v}_{1}-\mathbf{v}_{0}\left|\mathbf{v}_{2}-\mathbf{v}_{0}\right| \mathbf{v}_{3}-\mathbf{v}_{0}\right]=\left[\mathbf{w}_{1}-\mathbf{w}_{0}\left|\mathbf{w}_{2}-\mathbf{w}_{0}\right| \mathbf{w}_{3}-\mathbf{w}_{0}\right]
$$
因为我们使用了正四面体有：
$$
\mathbf{A}\left[\mathbf{v}_{1}-\mathbf{v}_{0}\left|\mathbf{v}_{2}-\mathbf{v}_{0}\right| \mathbf{v}_{3}-\mathbf{v}_{0}\right]=
\left[\begin{array}{ccc}
 1 & 0 & 0 \\
0 & 1 & 0 \\
0 & 0 & 1
\end{array}\right]
$$
因此我们可以得到：
$$
\mathbf{A}=\left[\mathbf{w}_{1}-\mathbf{w}_{0}\left|\mathbf{w}_{2}-\mathbf{w}_{0}\right| \mathbf{w}_{3}-\mathbf{w}_{0}\right]
$$
因此有$C’=（det A）AC_{canonical}A^T$，现在我们发现$C''=Translate(C',w_0-v_0)$，又因为$v_0=0$,因此我们可以得到$C''=Translate(C',w_o)$,$w_o$是我们选择的参考点。

## 其他质量属性（四面体）

质量：$m=\rho V$

体积：$V=\frac{1}{6}detA$,因为$V=\frac{|c\cdot(a\times b)|}{6}$

质心：四个顶点的平均值

## 累加结果

我们定义如下一组质量属性：$B_n=\{C_n,x_n,m_n\}$

我们想要找到$B_3=B_1+B_2$:

- 协方差：设想$C_1,C_2$在同一原点上计算，有$C_3=C_1+C_2$
- 质心：$\overline{\mathbf{x}}_3=（\overline{\mathbf{x}}_1m_1+\overline{\mathbf{x}}_2m_2）/(m_1+m_2)$
- 总质量：$m3=m_1+m_2$

因为$C_{total}$是围绕我们的选择的参考点计算的，我们希望将它移动到质心上（物理模拟器最希望它移动的地方）：
$$
C_{total}’=Translate(C_{total},w_0-x_{total})
$$
最后我们便可以根据一开始的公式从$C_{total}’$计算出$I_{total}'$。

# 参考资料

[BB04] BLOW J., BINSTOCK A.: How to find the inertia tensor (or othermass properties) of a 3d solid body represented by a triangle mesh
