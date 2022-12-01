## 团队名：Abandon team（"常看常新"队）

## 项目名：rigid body solver with XPBD

## 项目介绍：

论文《Detailed Rigid Body Simulation with Extended Position Based Dynamics》的复现（使用taichi）

论文阅读笔记在[这里](https://www.cnblogs.com/Ligo-Z/p/16921559.html)

**想要实现的功能（期望达到的效果）：**使用taichi实现一个通用的基于XPBD的刚体求解器，能够处理刚体之间的接触，各种关节以及与软物体的相互作用。

- 功能一：实现论文中提到的两个核心的投影操作
- 功能二：能够自然处理刚体间的接触与摩擦
- 功能三：处理铰链关节，球状关节以及移动关节，并限制其自由度
- 功能四：完成刚体碰撞检测

使用这个刚体求解器实现一些简单的场景（场景在设想，目前采取论文中的实验：

- 简单场景：很多刚体盒子（或者兔子）叠在一起

- 摆动场景：实现双摆和三摆，甚至闭环摆，类似下图

  ![image-20221123212748953](https://img2022.cnblogs.com/blog/1656870/202211/1656870-20221123212750831-1757344658.png)

- 链条悬挂：100只兔子组成的链条悬挂在天花板上的模拟，类似下图

  ![image-20221123212800311](https://img2022.cnblogs.com/blog/1656870/202211/1656870-20221123212802123-1618061579.png)

- 弹珠场景：在电线上的弹珠滑下后与其他弹珠的碰撞的情况，类似下图

  ![image-20221123212724738](https://img2022.cnblogs.com/blog/1656870/202211/1656870-20221123212726614-1777701870.png)

**实现的类：**

- Transfrom类：用于表示刚体与关节的欧式变换，
  - 位置，缩放（局部，世界）：3D vector
  - 旋转（局部，世界）：四元数
- Shape类：用于表示刚体的形状
- RigiBody类：用于表示刚体，支持创建一些简单的类型以及导入3D模型
  - 成员：质量，转动惯量（当前，上一帧），transfrom（当前，上一帧），速度（当前，上一帧）
  - 函数：set,get各种属性，应用位置修正，应用旋转修正
- Joint类：用于表示各种关节将两个物体连接起来，如铰链，球状，固定，移动关节
  - 成员：连接的两个物体
- XPBD Solver类：用于对刚体模拟进行求解

### 碰撞检测过程(目前打算采取开源代码，具体实现似情况而定)

1. 对需要进行碰撞检测的物体生成一个AABB（世界坐标）（用于碰撞检测
2. **Broad-Phase:**动态生成潜在的碰撞对（potentail collision pair）
3. **Narrow-Phase:**碰撞检测，若检测到物体发生碰撞，生成对应的接触的法线以及在两个物体上的接触点

![image-20221124213042012](https://img2022.cnblogs.com/blog/1656870/202211/1656870-20221124213043033-1530754736.png)

目前打算使用的TAICHIGAME的开源库进行碰撞检测，但是他有个问题计算太慢了（没有采用taichi加速）

# 有个大胆的想法（houdini with taichi）

场景在Houdini中制作，获取场景中刚体的属性信息，然后通过写tiachi求解器进行实现

这样，我便可以通过houdini快速制作场景并且获取得到想要的刚体属性信息，以及直接使用Houdini进行碰撞检测，然后获得碰撞后的法线和接触点。

（还没测试过，也没怎么说过houdini,后面几天看看能不能这样做）

## 开源库

- [TAICHIGAME](https://github.com/maksyuki/TaichiGAME)：Collision算法部分
- Tina：A real-time soft renderer based on the [Taichi](https://github.com/taichi-dev/taichi) programming language.（用于加载模型和渲染）
- 四元数函数：https://github.com/Yihao-Shi/TaichiDEM/blob/version-updated/Quaternion.py

