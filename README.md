# High-Performance RTS Simulation & Incremental Predictive Contact Pipeline
# 大规模 RTS 群体模拟 & 增量预测接触管线

![Unity](https://img.shields.io/badge/Unity-2022.3%2B-black)
![DOTS](https://img.shields.io/badge/Tech-ECS%20%7C%20Jobs%20%7C%20Burst-blue)
![NetCode](https://img.shields.io/badge/Network-NetCode%20for%20Entities-green)
![License](https://img.shields.io/badge/License-MIT-yellow)

> **核心亮点**：200+ 高密度单位同屏物理模拟 | 跨帧增量拓扑复用 | 并行 Jacobi XPBD 求解 | 基于指令的确定性回放

---

## 📺 [[Click to Watch Demo Video](https://www.bilibili.com/video/BV1moUhBsE3K/) (Bilibili )]

---

## 📖 项目简介 (Introduction)

基于 **Unity DOTS**（ECS + Jobs + Burst）的大规模 RTS 群体模拟系统，以及面向**动态接触**的**增量预测碰撞**与**并行 XPBD 约束求解**框架。

项目以 RTS 高密度单位移动作为实际应用与压力测试场景，自下而上实现**玩法驱动 → 运动预测 → 碰撞检测 → 接触调度 → 约束求解 → 状态回写**的完整管线。围绕动态接触在连续时间中的相干性，分别在**跨帧**（Persistent Topology）、**跨 substep**（Timestep Contact Set）和**单次 iteration**（CSR Incident Index）三个尺度上减少重复计算。

---

## ✨ 核心特性 (Key Features)

### 1. 海量单位流场寻路 (Flow Field Pathfinding)
摒弃传统 A* 逐单位寻路，实现基于 **Eikonal 方程**的向量场寻路。
* **性能：** 支持 **500+** 单位同时寻路，寻路逻辑完全并行化（Job System + Burst）。
* **机制：** Cost Field（静态障碍代价）→ Integration Field（Eikonal BFS）→ Vector Field（8 方向梯度下降），实现 $O(1)$ 复杂度单位方向查询。
* **调度：** 全局目标变更时按需重烘焙，双缓冲 `Grid` / `PendingGrid` 保证读取无锁。

<img src="https://cdn.jsdelivr.net/gh/chu123122/Image-hosting-service/img/FlowField.gif"/>

### 2. 增量预测接触管线 (Incremental Predictive Contact Pipeline)
针对高密度 RTS 场景中相邻帧接触拓扑高度相干的特点，维护**跨帧持久代理视图**，只对发生变化的"脏 body"进行增量修补。
* **持久拓扑：** `StableEntityPairKey` 驱动的实体对生命周期（Dormant → Approaching → Predictive → Actual → Separating → Expired），`PersistentSweptProxy` 以 Guard Bounds 证明拓扑完备性。
* **增量决策：** 每帧验证代理有效 → 标记脏体（拓扑脏/几何脏/逃逸）→ dirtyRatio > 35% 全量回退，否则增量修补。
* **安全钳位：** Soft/RVO 输出强制钳位在已证明的 Interaction Envelope 内，二分搜索寻找最大安全缩放；逃逸时 fallback 到全量重建。
* **独立验证：** O(N²) Oracle 持续检测增量管线的漏报（False Negative），诊断模式下漏报数必须为零。

### 3. XPBD 约束求解与软避让 (XPBD Contact Solver & Soft Avoidance)
为解决大量单位挤过窄口（沙漏场景）时的死锁与穿模，实现分层接触求解：
* **软分离 (Soft Avoidance)：** 支持 **Surface Velocity Buffer** 与 **Reciprocal Velocity Obstacle (RVO)** 两种模式，消耗紧凑 Soft 视图（非全量邻居），维持队形分离。
* **硬约束投影 (Hard Constraint)：** XPBD 位置投影修正穿透，含 Compliance 柔度控制，支持 Regular 与 Predictive 两种接触模式。
* **墙壁约束：** 基于流场格阻挡的静态墙壁投影，防止单位穿入障碍格。

<img src="https://cdn.jsdelivr.net/gh/chu123122/Image-hosting-service/img/PBD.gif"/>

### 4. 并行 Jacobi P1-P6 求解器 (Parallel Jacobi Solver)
在串行 Gauss-Seidel 参考路径之外，实现 **6 阶段并行接触投影管线**，突破高接触密度下的求解瓶颈：
* **P1-P2：** 管线初始化 + Substep 准备（位置预测、速度积分，body-parallel）
* **P3-P4：** 并行 Pair 评估 + 局部修正累积（pair-parallel，读取不可变位置快照确保确定性）
* **P5：** 持久拓扑修复 + Spatial Membership 缓存 + Pair 分类（serial prepare → parallel eval → serial commit）
* **P6：** 确定性 Body Gather + 位置应用 + 墙壁投影（body-parallel，CSR Incident Index 消除原子操作）
* **Gauss-Seidel** 保留为串行参考路径，调试模式下自动切换。

### 5. 事件溯源回放系统 (Event Sourcing Replay)
在服务端权威（Server-Authoritative）架构下，实现基于指令流的回放系统。
* **零快照：** 不记录每帧 Transform，仅记录关键输入指令 (Command Buffer)。
* **瞬间重置：** 利用 ECS 的结构特性，毫秒级状态回滚与指令重演。
* **操作：** 按 L 开始录制，按 R 开始回放。

<img src="https://cdn.jsdelivr.net/gh/chu123122/Image-hosting-service/img/Replay.gif"/>

### 6. 混合式网络架构 (Hybrid Network Architecture)
* **框架：** 基于 **Unity NetCode for Entities**。
* **策略：** 服务端权威 (Server-Auth) + 客户端预测 (Client-Side Prediction)，结合本地模拟层，支持断线后的本地平滑回放。
* **双模式：** `NetCodeUnitFlowMovementSystem`（联网）与 `LocalUnitFlowMovementSystem`（本地）共享同一 `BaseFlowMovementSystem` 调度逻辑。

---

## 🛠️ 技术架构 (Architecture)

### ECS Systems Overview

**Simulation Group（核心模拟）：**
* `FlowFieldBakeSystem` — 按需烘焙 Cost / Integration / Vector Field（Parallel Jobs）。
* `BaseFlowMovementSystem` — 完整移动管线：独立力 → 接触对构建 → 软避让 → XPBD求解 → 位置写回。
  * `LocalUnitFlowMovementSystem` / `NetCodeUnitFlowMovementSystem` — 本地 / 联网模式具体化。
* `RtsCommandSystem` — MoveOrder 消费，编队槽位分配，触发流场重烘焙。
* `UnitSpatialPartitionSystem` — 空间哈希网格（可选，当前禁用）。

**Contact Pipeline 六层模块（`Jobs/ContactPipeline/`）：**

| 模块 | 职责 | 生命周期 |
|------|------|---------|
| **BroadPhase** | Swept Disc 空间哈希候选对生成 | 帧内 |
| **Persistent** | 跨帧实体对拓扑、脏体分类、稳定键/法线 | 跨帧 |
| **Prediction** | Timestep 包络预测 + Envelope Guard 安全钳位 | Timestep |
| **SoftAvoidance** | RVO / 速度缓冲软避让 | 子步 |
| **Solver** | XPBD (Gauss-Seidel / Jacobi P1-P6) + 墙壁投影 | 子步迭代 |
| **Motion** | 速度准备、位置预测、速度重建 | 子步 |

**Replay Group（回放）：**
* `CommandReplayingSystem` — 指令录制与时间轴管理 (Event Sourcing)。
* `RequestCommandRpcSystem` — RPC 指令采集与网络同步。

### Pipeline 数据流

```
鼠标输入 (UnitMoveInputSystem)
    │  框选单位 → MoveOrder (RPC)
    ▼
RtsCommandSystem
    │  编队槽位 → UnitMoveDestination
    ▼
FlowFieldBakeSystem
    │  Cost → Integration → Vector → 双缓冲发布
    ▼
BaseFlowMovementSystem (每帧)
    │
    ├─ [1] CalculateIndependentFlowForceJob  流场驱动力 + 状态初始化
    ├─ [2] IncrementalPredictiveContactPipeline  增量拓扑验证/修补
    ├─ [3] SoftAvoidance (子步迭代)  RVO / 速度缓冲
    ├─ [4] XPBD Solver (子步迭代)  Gauss-Seidel / Jacobi P1-P6
    ├─ [5] Wall Projection (子步迭代)  静态墙壁投影
    └─ [6] ApplyFlowMovementJob  Transform + Velocity 写回 ECS
```

### Performance (Profiler Data)

在 Ryzen 7 上实测：
* **逻辑帧耗时:** < 0.3ms (200 Agents)
* **Burst 优化:** 核心 Job 利用 SIMD 指令集加速，零 GC。
* **内存：** NativeContainer 持久分配，无每帧堆分配。

<img src="https://cdn.jsdelivr.net/gh/chu123122/Image-hosting-service/img/20251126133028955.png"/>

---

## 📊 诊断与调优 (Diagnostics & Tuning)

| 工具 | 快捷键 | 用途 |
|------|--------|------|
| **SimulationDebuggerPanel** | `F8` | 四窗口 IMGUI（整体概况 / 跨帧AABB / 跨子步Contact / 运行时设置） |
| **AdaptiveParameterTuner** | 挂场景运行 | 自动参数搜索，CSV 输出 |
| **IncrementalContactPipelineBenchmarkWindow** | Editor 菜单 | 增量管线指标实时对比 |
| **IncrementalPredictiveContactValidation** | `Ctrl+Shift+F12` | 预测接触 Stage 3 验证 |
| **LocalGameplayModeValidation** | Editor 菜单 | 本地模式功能验证 |
| **IncrementalContactOracle** | 诊断模式自动 | O(N²) 独立验证，检测增量漏报 |

诊断统计 `PredictiveDiscContactStatistics` 覆盖：Contact Set 构建/逃逸/回退计数、Active/Predictive/Dormant pair 分类、穿透深度统计、各阶段纳秒级耗时。

---

## ⚠️ 已知限制 (Known Limitations)

* 未实现 Contact-island 休眠——持续活跃接触始终求解
* Gauss-Seidel 无并行路径（需图着色或冲突无关批次）
* 确定性列表压缩、排序、持久视图发布仍为串行协调点
* 流场当前为全局烘焙，未支持局部动态障碍增量更新
* 持久 Spatial Membership 依赖容量上限，超限回退全量扫描
