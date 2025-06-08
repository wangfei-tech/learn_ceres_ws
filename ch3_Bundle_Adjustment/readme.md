“**Bundle Adjustment problems**”（捆绑调整问题）是计算机视觉和机器人领域中的一个**非线性优化问题**，主要用于**同时优化相机姿态（位置和方向）以及三维点的位置**，以最小化所有图像观测误差。

---

###  一句话解释：

> **Bundle Adjustment** 是在多个图像中联合优化相机位姿和三维点坐标，以让所有图像中的投影尽可能贴近真实观测。

---

###  应用场景：

* **SLAM（Simultaneous Localization and Mapping）**
* **Structure from Motion（SfM）**
* **三维重建**
* **视觉里程计**

---

###  数学表达（简化）：

设：

* $\mathbf{P}_i$：第 $i$ 个相机的姿态参数
* $\mathbf{X}_j$：第 $j$ 个三维点
* $\mathbf{x}_{ij}$：第 $i$ 个相机中看到的第 $j$ 个点的图像坐标

目标是优化所有 $\mathbf{P}_i$、$\mathbf{X}_j$，使得：

$$
\min_{\{\mathbf{P}_i, \mathbf{X}_j\}} \sum_{i,j} \left\| \mathbf{x}_{ij} - \pi(\mathbf{P}_i, \mathbf{X}_j) \right\|^2
$$

其中 $\pi(\cdot)$ 表示从3D点到图像的投影函数。

---

###  通常涉及：

* 相机模型（内参/外参）
* 投影误差
* 非线性最小二乘优化（如用 Ceres Solver、g2o、GTSAM）

---

###  举个例子：

假设你拍了一系列照片用于 3D 重建，每张照片中都有重叠的物体。你知道每个图像里某个点的2D位置，但不太确定相机具体在哪，也不确定3D点确切在哪。**Bundle Adjustment 就是让你把“所有相机和点的位置”都调到最合理的状态，使得投影误差最小。**

---
