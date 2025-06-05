“残差”（**residual**）是一个在优化、SLAM、曲线拟合等领域中非常核心的概念，它表示 **模型的预测值与真实观测值之间的误差**。

---

## ✅ 简单理解

你可以把 **残差看成“没对上差了多少”**。

### 举个例子：

假设你要找一个变量 `x`，希望满足这个目标：

> $x = 10$

但你一开始猜的是 $x = 0.5$，那就“差了 9.5”，这 9.5 就是残差：

$$
\text{残差} = 10 - x = 9.5
$$

---

## ✅ 残差在优化中的意义

优化算法（比如 Ceres Solver）不直接最小化“误差”，而是最小化 **残差的平方和（sum of squared residuals）**：

$$
\text{cost} = \frac{1}{2} \sum (\text{残差})^2
$$

所以：

* 残差越小，表示模型越准确；
* 最小化残差的平方，就是让我们的模型尽量“拟合”真实数据。

---

## ✅ 在 SLAM 中的残差

在 SLAM 或 Bundle Adjustment（BA）中，残差一般是：

* 位置误差（里程计和实际位置之间的差）；
* 投影误差（地图点投影到相机图像上的点，与观测像素位置的差）；
* 角度误差（姿态估计和观测之间的差）；

例如，Bundle Adjustment 的一个常见残差：

$$
\text{残差} = \text{观测图像坐标} - \text{投影的三维点图像坐标}
$$

---

## ✅ 图示帮助理解（用拟合为例）

假设你有一堆点，想用一条直线去拟合它们：

```
点:         o
                o
      o
           o
拟合线:    ----------------

每个点和线之间垂直的距离，就是残差。
```

---

## ✅ 总结

| 名称        | 意义                                     |
| --------- | -------------------------------------- |
| 残差        | 模型输出值与真实观测值之间的误差                       |
| 目标        | 最小化所有残差的平方和（非线性最小二乘）                   |
| 在 Ceres 中 | 用 `residual[0] = ...` 定义残差即可，其他交给优化器处理 |

---

这句代码：

```cpp
options.linear_solver_type = ceres::DENSE_QR;
```

是 **Ceres Solver** 中设置 **线性求解器（linear solver）类型** 的一部分，它定义了在每一次迭代中，Ceres 用哪种方法来求解线性方程组（通常是雅可比矩阵相关的正规方程）。

------------------


##  背景：非线性最小二乘是怎么求的？

Ceres 是求解非线性最小二乘问题的，它通过迭代，每次都要解一个线性子问题：

$$
J^T J \Delta x = -J^T r
$$

其中：

* $J$：雅可比矩阵（Jacobian）
* $r$：残差向量
* $\Delta x$：当前变量的更新量

这个线性方程组的解法就叫 **线性求解器（linear solver）**。

---

##  `DENSE_QR` 是什么？

`DENSE_QR` 表示使用 **稠密矩阵的 QR 分解** 来解线性方程：

* **DENSE**：矩阵是小而密集的（不是稀疏矩阵），通常适用于小规模问题；
* **QR 分解**：比 `Cholesky` 更稳定，适用于条件不太好的问题（比如方程组接近奇异）；

> 所以，如果你只是做一些简单的学习测试（比如一个变量），用 `DENSE_QR` 是很合适的。

---

##  可选的线性求解器类型（常见）

| 类型                       | 说明                          | 适合场景            |
| ------------------------ | --------------------------- | --------------- |
| `DENSE_QR`               | 稠密QR分解，稳定性好，但计算量大           | 小规模问题，教学演示      |
| `DENSE_NORMAL_CHOLESKY`  | 稠密Cholesky分解，速度快但数值不太稳定     | 小规模但数值条件好的问题    |
| `SPARSE_NORMAL_CHOLESKY` | 稀疏Cholesky分解                | SLAM/BA类大规模稀疏问题 |
| `ITERATIVE_SCHUR`        | 用于 Bundle Adjustment 的高效迭代法 | BA、大规模优化        |
| `CGNR`                   | 共轭梯度法                       | 特别大的稀疏问题        |

---

##  小结

```cpp
options.linear_solver_type = ceres::DENSE_QR;
```

的意思是：

> 在每次非线性优化迭代中，Ceres 用稠密 QR 分解法来解线性子问题。

适用于你这个小型非线性问题（比如残差维度是1，变量也是1），简单稳定，计算负担小。

---

##  Ceres 中常见 `linear_solver_type` 对比

| 类型                       | 原理说明                       | 适用场景             | 优缺点       |
| ------------------------ | -------------------------- | ---------------- | --------- |
| `DENSE_QR`               | QR 分解：更稳定，但计算量大            | 小规模问题，如学习例子      | ✅稳定 ❌慢    |
| `DENSE_NORMAL_CHOLESKY`  | 构建 $J^T J$ 然后用 Cholesky 分解 | 小规模，条件好的问题       | ✅快 ❌不稳定   |
| `SPARSE_NORMAL_CHOLESKY` | 稀疏矩阵Cholesky               | 中等规模稀疏问题，如小型SLAM | ✅节省内存 ✅中速 |
| `ITERATIVE_SCHUR`        | 分块变量、迭代解法（主要用于 BA）         | 大型 BA（地图点多）      | ✅效率极高     |
| `CGNR`                   | 共轭梯度法，适合超大问题               | 极大稀疏图优化问题        | ✅内存省 ❌收敛慢 |

---

##  具体 SLAM 应用场景比较

### 场景：你在做视觉 SLAM，使用 Bundle Adjustment 优化如下内容：

* 有 3000 个地图点；
* 有 150 帧关键帧（位姿）；
* 每帧看到大约 100\~200 个地图点；
* 每个观测残差是 2 维的（图像投影误差）；

这时我们构建的非线性最小二乘问题变量数量可能是：

* 位姿变量维度：150 × 6 = 900；
* 地图点变量维度：3000 × 3 = 9000；
* 总变量数接近 10000 维。

###  哪种求解器好？

| 求解器                      | 是否合适？  | 原因                              |
| ------------------------ | ------ | ------------------------------- |
| `DENSE_QR`               | ❌完全不合适 | 会构建出 10000×10000 的稠密矩阵，占内存上百 GB |
| `DENSE_NORMAL_CHOLESKY`  | ❌同样不合适 | 数值不稳定、内存爆炸                      |
| `SPARSE_NORMAL_CHOLESKY` | ✅可用    | 如果使用稀疏性，可以高效求解                  |
| `ITERATIVE_SCHUR`        | ✅非常合适  | 专门为 BA 优化设计，拆分变量结构，高效计算         |
| `CGNR`                   | 可能 ✅   | 更适合内存极为紧张的超大地图优化                |

### Ceres 推荐：使用 `ITERATIVE_SCHUR`！

```cpp
options.linear_solver_type = ceres::ITERATIVE_SCHUR;
```

此外，为提升稀疏效率：

```cpp
options.preconditioner_type = ceres::JACOBI;
options.use_explicit_schur_complement = false;
```

---

##  如果你是做 2D/3D 激光 SLAM？

使用 `SPARSE_NORMAL_CHOLESKY` 可能更合适：

```cpp
options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
```

前提是你系统装了支持稀疏线性代数库（比如 `SuiteSparse`, `CXSparse`），否则可能 Ceres 会自动降级。

---

##  总结一句话：

> ❗选对 `linear_solver_type` 可以显著减少求解时间、内存占用，并提高收敛稳定性！

| 规模/目标        | 推荐 Solver 类型             |
| ------------ | ------------------------ |
| 学习、例子        | `DENSE_QR`               |
| 小型 BA        | `SPARSE_NORMAL_CHOLESKY` |
| 大型 BA / SLAM | `ITERATIVE_SCHUR`        |
| 极大规模优化       | `CGNR`                   |

---

在 Ceres Solver 中，**Numeric Derivatives（数值导数）** 是在无法提供解析导数（Analytic Derivatives）或自动微分（Automatic Differentiation）时，用\*\*有限差分（finite difference）\*\*方法近似计算导数的一种方式。

---

## 📘 一、什么是 Numeric Derivatives？

Ceres 提供三种导数计算方式：

1. **Analytic Derivatives（解析导数）**：手动写出雅可比矩阵（Jacobian）。
2. **Automatic Differentiation（自动微分）**：Ceres 内置的 `AutoDiffCostFunction`。
3. **Numeric Derivatives（数值导数）**：Ceres 用 `NumericDiffCostFunction` 通过有限差分计算导数。

使用 Numeric Derivatives 的成本最高，因为它：

* 需要多次调用用户定义的 cost 函数；
* 导数精度较差，可能引入数值不稳定；
* 效率低，但在一些难以微分的场景中是可行的选择。

---

## 🛠️ 二、使用方法：`NumericDiffCostFunction`

### 示例：

```cpp
struct MyCostFunctor {
  template <typename T>
  bool operator()(const T* const x, T* residual) const {
    residual[0] = T(10.0) - x[0];
    return true;
  }
};

ceres::CostFunction* cost_function =
    new ceres::NumericDiffCostFunction<MyCostFunctor, ceres::CENTRAL, 1, 1>(
        new MyCostFunctor);
problem.AddResidualBlock(cost_function, nullptr, &x);
```

### 模板参数说明：

```cpp
NumericDiffCostFunction<FunctorType, DifferenceMethodType, num_residuals, parameter_block_size...>
```

* `FunctorType`：代价函数结构体；
* `DifferenceMethodType`：差分类型：

  * `ceres::CENTRAL`（中央差分，精度更高）；
  * `ceres::FORWARD`（前向差分，速度更快）；
* `num_residuals`：残差维度；
* `parameter_block_size...`：参数块维度，可以多个。

---

## 🧮 三、差分原理（中央差分举例）：

对某参数 $x_i$，残差函数为 $f(x)$，其数值导数估算为：

$$
\frac{\partial f}{\partial x_i} \approx \frac{f(x + h \cdot e_i) - f(x - h \cdot e_i)}{2h}
$$

其中 $h$ 是一个小扰动值（默认为 $\sqrt{\epsilon}$，可以配置），$e_i$ 是单位向量。

---

## ⚠️ 四、使用建议

* 优先考虑使用 **AutoDiff** 或 **Analytic**；
* 只有在无法手动/自动求导时才使用 NumericDiff；
* 注意配置差分步长，避免数值精度误差；
* NumericDerivatives 更容易引入局部最小值或震荡现象。

---



