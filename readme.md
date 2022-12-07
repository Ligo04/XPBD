## 团队名：Abandon team（"常看常新"队）

## 项目名：rigid body solver with XPBD

## 项目介绍：

论文《Detailed Rigid Body Simulation with Extended Position Based Dynamics》的复现（使用taichi）

论文阅读笔记在[这里](https://www.cnblogs.com/Ligo-Z/p/16921559.html)

**想要实现的功能（期望达到的效果）：**  使用taichi实现一个通用的基于XPBD的刚体求解器，能够处理刚体之间的接触，各种关节以及与软物体的相互作用。

- 功能一：实现论文中提到的两个核心的投影操作
- 功能二：处理铰链关节，球状关节以及移动关节，并限制其自由度(关节约束，距离约束)(**关节约束还有些问题卡着，待后续完善**)
- 功能三：完成刚体碰撞检测（做不了啦。abandon）
- 功能四：能够自然处理刚体间的接触与摩擦（碰撞约束）

使用这个刚体求解器实现一些简单的场景（场景在设想，目前采取论文中的实验：

**场景应该是最后实现了单，双，三摆场景**

- 简单场景：很多刚体盒子（或者兔子）叠在一起(需要碰撞检测，adandon)

- 摆动场景：实现双摆和三摆，甚至闭环摆，类似下图(场景就是这个)

  ![image-20221123212748953](https://img2022.cnblogs.com/blog/1656870/202211/1656870-20221123212750831-1757344658.png)

- 链条悬挂：100只兔子组成的链条悬挂在天花板上的模拟，类似下图（abandon）

  ![image-20221123212800311](https://img2022.cnblogs.com/blog/1656870/202211/1656870-20221123212802123-1618061579.png)

- 弹珠场景：在电线上的弹珠滑下后与其他弹珠的碰撞的情况，类似下图(abandon)

  ![image-20221123212724738](https://img2022.cnblogs.com/blog/1656870/202211/1656870-20221123212726614-1777701870.png)

**实现的类：**
- Quaternion：四元数
  - 四元数函数

- Transfrom类（ti.dataclass）：用于表示刚体与关节的欧式变换，
  - 位置，缩放（世界）：3D vector
  - 旋转（世界）：四元数
  
- Entity类（ti.dataclass）：用于表示实体（表示用于solver的数据）
  - 成员：逆质量，逆转动惯量，transfrom（当前，上一帧），速度，角速度，静摩擦系数，动摩擦系数，修复系数，
  - 函数：应用位置修正，应用旋转修正，应用速度修正，获取广义逆质量等
  
- Model类型（ti.data_oriented）：加载3D模型(目前加载obj模型)
  
- Rigiboody类（ti.data_oriented）: 包含一个Model 以及 一个Enitiy
  
- Constarint类（ti.dataclass）：用于表示需要进行约束：

  - 位置约束：最大位置
  - 碰撞约束：碰撞点（两个刚体上），碰撞法线
  - 关节约束：固定关节，铰链关节，球状关节

  ​    最后求解器是分别对他们进行求解

- XPBD Solver类（ti.data_oriented）：用于对刚体模拟进行求解

  - Entity struct Field,Constraint struct  Filed,mat4 field(采用dynamic snode)

  - 函数：添加刚体以及约束（有最大限制

  - 求解过程具体是采用Small Steps in Physics Simulation的思路（对刚体进行更新：

    ![image-20221203154934490](https://img2023.cnblogs.com/blog/1656870/202212/1656870-20221203155140530-631716489.png)

    对于$\Delta x$ 这篇论文提到了两个核心的投影操作，一个对位置修正，一个是对旋转修正（使两个刚体某个轴对齐）

### 碰撞检测过程(目前打算采取开源代码，具体实现似情况而定)：abandon

1. 对需要进行碰撞检测的物体生成一个AABB（世界坐标）（用于碰撞检测
2. **Broad-Phase:**  动态生成潜在的碰撞对（potentail collision pair）
3. **Narrow-Phase:**  碰撞检测，若检测到物体发生碰撞，生成对应的接触的法线以及在两个物体上的接触点

![image-20221124213042012](https://img2022.cnblogs.com/blog/1656870/202211/1656870-20221124213043033-1530754736.png)

目前打算使用的TAICHIGAME的开源库进行碰撞检测，但是他有个问题计算太慢了（没有采用taichi加速）


## Run 

```
pip install -r requirements.txt
```

按照目前的复现的，目前提供了两个场景：

1. 三摆场景：

   ```
   python demo_threePendulum.py
   ```

   ![image-20221207195244207](https://img2023.cnblogs.com/blog/1656870/202212/1656870-20221207195244537-1257811328.png)

2. 盒子：

   ```
   python demo_box.py
   ```

   ![image-20221207195430151](https://img2023.cnblogs.com/blog/1656870/202212/1656870-20221207195430462-1631538396.png)

由于笔者采用GS迭代，但是又没有实现并行化版本的GS,在多个约束或者多次迭代时候很多概率约束求解不正确，目前场景提供的标准的numsubStep=5时是效果最好的。（CPU并行）

### Benchmark(估摸)

CPU：i7-12700H

GPU：NVIDIA GeForce RTX 3070 Laptop GPU

|                     | numSubStep=5 | numSubStep=10 |
| ------------------- | ------------ | ------------- |
| Boxes(CPU)          | 80FPS        | 60FPS         |
| Boxes(GPU)          | 43FPS        | 30FPS         |
| Three pendulum(CPU) | 60FPS        | 48FPS         |
| Three pendulum(GPU) | 35FPS        | 24FPS         |
