# 大规模 RTS 群体模拟 & 增量预测接触管线

![Unity](https://img.shields.io/badge/Unity-2022.3%2B-black)
![DOTS](https://img.shields.io/badge/Tech-ECS_|_Jobs_|_Burst-blue)
![NetCode](https://img.shields.io/badge/Network-NetCode_for_Entities-green)

> 200+ 高密度单位同屏物理模拟 | 跨帧增量拓扑复用 | 并行 Jacobi XPBD 求解 | 指令回放

---

## 项目定位

基于 **Unity DOTS**（ECS + Jobs + Burst）的大规模 RTS 群体模拟系统，以及面向**动态接触**的**增量预测碰撞**与**并行 XPBD 约束求解**框架。

项目以 RTS 高密度单位移动作为实际应用与压力测试场景，自下而上实现**玩法驱动 → 运动预测 → 碰撞检测 → 接触调度 → 约束求解 → 状态回写**的完整管线。围绕动态接触在连续时间中的相干性，分别在**跨帧**、**跨 substep** 和**单次 iteration** 三个尺度上减少重复计算。

---

## 整体架构

```
PlayerInput (鼠标指令)
    │
    ▼
RtsCommandSystem ──→ MoveOrder ──→ FlowFieldBakeSystem (Cost → Integration → Vector)
    │                                    │
    ▼                                    ▼
BaseFlowMovementSystem ◄── FlowFieldGrid (每帧稳定快照)
    │
    ├─ [A] CalculateIndependentFlowForceJob    ← 流场驱动力 + 单位状态初始化
    │
    ├─ [B] SolveXpbdUnitContactsJob (主求解器)  ← ┐
    │   ├─ Persistent: 跨帧增量拓扑验证/修复     │  跨帧复用
    │   ├─ BroadPhase: 帧内 Swept Disc 候选对    │
    │   ├─ Prediction: Timestep 包络预测          │
    │   ├─ SoftAvoidance: RVO / 速度缓冲 (子步)   │  子步迭代
    │   ├─ XPBD Solver: Gauss-Seidel / Jacobi    │
    │   └─ Motion: 位置预测 + 速度重建 (子步)     │
    │
    └─ [C] ApplyFlowMovementJob               ← Transform 写回 ECS
```

### 核心模块

| 模块 | 路径 | 职责 |
|------|------|------|
| **FlowField** | `Entities/Unit/Systems/FlowField/` | Eikonal 流场寻路 + 移动调度 |
| **Contact Pipeline** | `…/Jobs/ContactPipeline/` | 增量预测碰撞 + XPBD 约束求解 |
| **Unit Components** | `Entities/Unit/Components/` | ECS 组件定义 |
| **Replay** | `Entities/_RePlay/` | 基于指令的事件溯源回放 |
| **Diagnostics** | `…/FlowField/Diagnostics/` | 调试面板、参数调优、Benchmark |
| **Editor** | `…/FlowField/Editor/` | 编辑器验证与基准窗口 |
| **Common** | `Entities/_Common/` | 攻击、伤害、技能等通用系统 |
| **PlayerInput** | `_PlayerInput/` | 鼠标框选与移动指令 |
| **Camera** | `Entities/Camera/` | RTS 摄像机控制 |

---

## 一、流场寻路（Flow Field）

基于 Eikonal 方程的向量场寻路，替代传统逐单位 A*。

```
Cost Field → Integration Field (Eikonal) → Vector Field (梯度下降)
```

- 单次全局烘焙，O(1) 每单位方向查询
- `FlowFieldBakeSystem` 负责 Cost → Integration 计算
- `GenerateVectorFieldJob` 生成 8 方向向量场
- `CalculateIndependentFlowForceJob` 从向量场采样，计算独立于邻居的驱动力

### 关键类型

| 类型 | 文件 | 说明 |
|------|------|------|
| `FlowFieldGrid` | `GridComponent.cs` | 网格数据（原点、尺寸、Cell 数组） |
| `FlowFieldCell` | `GridComponent.cs` | 单格 Cost + Integration + 最优方向 |
| `FlowFieldSettings` | `GridComponent.cs` | 网格 + 软避让参数 |
| `FlowFieldRuntimeState` | `GridComponent.cs` | 版本号驱动的稳定快照 |
| `MoveOrder` / `MoveOrderSelectionElement` | `GridComponent.cs` | 移动指令 + 选中单位快照 |

---

## 二、增量预测接触管线（Contact Pipeline）

### 设计思想

高密度 RTS 场景中，单位位置在连续帧间变化缓慢——相邻两帧的接触拓扑高度相干。与其每帧从零构建所有接触对，不如维护一份**跨帧持久代理视图**，只对移动超过包络边界的"脏 body"进行增量修复。

三条核心正确性保障：

1. **Envelope Guard**：所有可能接触都在当前 Interaction 视图内或被脏 body 覆盖——Soft/RVO 输出不得逃逸
2. **Fallback**：增量证明失败时统一回退到全量 Swept Disc 重建
3. **Oracle**：O(N²) 独立验证器持续检测增量管线的漏报

### 六层子模块

| 层 | 路径 | 职责 | 生命周期 |
|----|------|------|---------|
| **BroadPhase** | `…/BroadPhase/` | Swept Disc 空间哈希候选对生成 | 帧内 |
| **Persistent** | `…/Persistent/` | 跨帧实体对拓扑、脏体分类、稳定键 | 跨帧 |
| **Prediction** | `…/Prediction/` | Timestep 包络预测 + 安全钳位 | Timestep |
| **SoftAvoidance** | `…/SoftAvoidance/` | RVO / 速度缓冲软避让 | 子步 |
| **Solver** | `…/Solver/` | XPBD 约束求解 + 墙壁投影 | 子步迭代 |
| **Motion** | `…/Motion/` | 速度准备、位置预测、速度重建 | 子步 |

### 关键类型

| 类型 | 说明 |
|------|------|
| `StableEntityPairKey` | 跨帧稳定的实体对标识（按 Index/Version 排序） |
| `PersistentSweptProxy` | 持久代理：Guard Bounds 证明拓扑完备性，Tight Bounds 描述预测视野 |
| `PersistentNeighborPair` | 跨帧邻居对缓存 |
| `PersistentPredictiveContact` | 跨帧预测接触（含法线） |
| `UnitCollisionPair` | 帧内 XPBD 约束（BodyA/B、Lambda、法线、激活状态） |
| `FlowMovementFrameState` | 单帧临时状态（位置、速度、包络、力累积），不写 ECS |
| `ContactPipelineConfiguration` | 归一化求解器配置 |

### 增量拓扑流程

```
当前帧 Body States
    │
    ├─ ValidateAndClassifyIncrementalDirtyBodies()
    │   └─ 标记：拓扑脏（entity 增删）、几何脏（位置超包络）、稳定
    │
    ├─ dirtyRatio > 35%？
    │   ├─ YES → FullRebuildPersistentNeighborTopology()  [全线重建]
    │   └─ NO  → UpdatePersistentProxyMetadata()          [增量修补]
    │            PatchDirtyBodyNeighborTopology()          [仅修补脏体邻居]
    │
    └─ 输出：当前帧有效 Contact Pair Set
```

---

## 三、XPBD 约束求解器

### Gauss-Seidel（串行参考路径）

串行迭代，每个 contact 求解后立即更新双方位置——下个 contact 可见最新结果。收敛快但无并行性。

### Parallel Jacobi P1-P6（并行路径）

6 阶段并行接触投影管线，含 CSR 格式主动约束索引。

| 阶段 | 内容 | 并行度 |
|------|------|--------|
| **P1** | 初始化管线状态 | — |
| **P2** | 准备 substep：位置预测、速度积分 | body-parallel |
| **P3** | 并行 pair 评估：计算法线 + 约束值 | pair-parallel |
| **P4** | 局部修正累积 (scatter) | pair-parallel |
| **P5A** | 持久拓扑修复 | 串行协调 |
| **P5B** | Spatial Membership 缓存（cell→proxy 映射） | — |
| **P5C** | 持久 pair 分类（serial prepare → parallel eval → serial commit） | pair-parallel |
| **P6** | 确定性 body gather + 位置应用 + 墙壁投影 | body-parallel |

**关键设计**：
- Jacobi 并行对评估读取不可变位置快照，确保确定性
- 集合并行修正按 incident-pair 顺序完成
- `ActiveConstraintIncidentIndex`：CSR 格式的 body → contact 索引，Gauss-Seidel 和 Jacobi 共享

### 求解器类型

| 类型 | 说明 |
|------|------|
| `XpbdContactConstraintMath` | XPBD 约束评估（法线、约束值、Lambda 更新） |
| `SoftAvoidanceMath` | 软避让速度计算（RVO / Surface Velocity Buffer） |
| `WallConstraintSolver` | 静态墙壁投影（基于流场格阻挡） |
| `ContactEnvelopeGuard` | 软避让输出钳位 + Timestep 逃逸检测 |

---

## 四、运动集成

每子步执行：

```
1. PredictUnconstrainedPositions()
   └─ positionₜ₊₁ = positionₜ + (独立力 + 软避让) × Δt

2. XPBD Contact Solve (Gauss-Seidel / Jacobi)
   └─ 约束投影：positionₜ₊₁ → 满足无穿透的 positionₜ₊₁'

3. ReconstructVelocities()
   └─ velocity = (positionₜ₊₁' − positionₜ) / Δt
```

---

## 五、事件溯源回放系统

| 文件 | 说明 |
|------|------|
| `CommandReplayingSystem.cs` | L 键录制 / R 键回放 |
| `ReplaySchema.cs` | 回放状态 + 指令 Buffer 定义 |
| `RequestCommandRpcSystem.cs` | RPC 指令采集 |
| `RTSUnitSpawner.cs` | 录制时的单位快照重建 |

**设计**：不记录每帧 Transform，仅记录输入指令 Buffer + 时间戳。回放时从头快进执行，利用确定性模拟重现结果。

---

## 六、诊断与调优

| 工具 | 文件 | 用途 |
|------|------|------|
| **SimulationDebuggerPanel** | `Diagnostics/SimulationDebuggerPanel.cs` | F8 四窗口 IMGUI（整体/跨帧AABB/跨子步Contact/设置） |
| **AdaptiveParameterTuner** | `Diagnostics/AdaptiveParameterTuner.cs` | 自动参数搜索，输出 CSV |
| **IncrementalContactPipelineCsvRecorder** | `Diagnostics/IncrementalContactPipelineCsvRecorder.cs` | 实验数据 CSV 导出 |
| **IncrementalContactPipelineExperimentRuntime** | `Diagnostics/IncrementalContactPipelineExperimentRuntime.cs` | 运行时实验参数覆盖 |
| **IncrementalContactPipelineBenchmarkWindow** | `Editor/IncrementalContactPipelineBenchmarkWindow.cs` | Editor 基准窗口 |
| **IncrementalPredictiveContactValidation** | `Editor/IncrementalPredictiveContactValidation.cs` | Editor 预测接触验证 |
| **LocalGameplayModeValidation** | `Editor/LocalGameplayModeValidation.cs` | 本地模式功能验证 |
| **SimulationDebuggerWorldOverlay** | `Diagnostics/SimulationDebuggerWorldOverlay.cs` | 场景覆盖层可视化 |
| **IncrementalContactOracle** | `Jobs/IncrementalContactOracle.cs` | O(N²) 独立验证器，检测增量管线漏报 |

### 诊断统计

`PredictiveDiscContactStatistics` 提供完整帧级指标：
- Timestep Contact Set 构建/分类/复用/逃逸/回退计数
- Candidate / Active / Predictive / Dormant pair 分类
- 穿透统计（最大/平均 穿透深度）
- 各阶段耗时（ns）：Pair Generation / Soft Avoidance / Iteration / Solver

---

## 七、其他游戏系统

| 系统 | 路径 | 说明 |
|------|------|------|
| **Attack** | `_Common/Systems/Attack/` | 攻击技能 + 触发器伤害 |
| **HealPoint** | `_Common/Systems/HealPoint/` | 生命值 + 帧伤害累积 + 应用 |
| **AbilityMove** | `_Common/Systems/AbilityMove/` | 技能驱动位移 |
| **Track** | `_Common/Systems/Track/` | 目标跟踪 |
| **Destroy** | `_Common/Systems/Destroy/` | 定时销毁 + 生命归零销毁 |
| **Building** | `Entities/Building/` | 建筑放置（兵营等） |
| **Selection** | `Unit/Systems/Selection/` | 框选 + 选中状态同步 |
| **HealthBar** | `Unit/Systems/HealthBar/` | 血条创建 |
| **Camera** | `Entities/Camera/` | 摄像机初始化 |
| **Input** | `_PlayerInput/UnitControl/` | 鼠标移动指令 |

---

## 配置参数

### 核心求解器 (`UnitContactSolverSettings`)

| 参数 | 默认 | 说明 |
|------|------|------|
| `SubstepCount` | 4 | 每帧子步数 |
| `IterationCount` | 4 | 每子步 XPBD 迭代数 |
| `ContactPositionSolver` | GaussSeidel | 约束求解器（GaussSeidel / Jacobi） |
| `Compliance` | — | XPBD 柔度 |
| `PredictiveSkin` | 0.05 | 预测接触膨胀厚度 |
| `EnablePredictiveContacts` | true | 启用预测接触 |
| `EnableFatAabbCache` | true | 启用跨帧持久拓扑缓存 |
| `FatAabbCacheMargin` | 0.5 | Guard Envelope 膨胀余量 |
| `TimestepContactMargin` | 0.02 | Timestep 接触检测余量 |

### 流场避让 (`FlowFieldSettings`)

| 参数 | 说明 |
|------|------|
| `SoftAvoidanceResponseRate` | 软避让响应强度 |
| `SoftAvoidanceShell` | 软避让壳体半径 |
| `SettledSoftAvoidanceMultiplier` | 已到达单位的避让衰减 |
| `SoftAvoidanceVelocitySolver` | 避让算法（SurfaceBuffer / RVO） |
| `RvoTimeHorizon` | RVO 时间视野 |

---

## 已知限制

- 未实现 Contact-island 休眠——持续活跃接触始终求解
- Gauss-Seidel 无并行路径（需图着色或冲突无关批次）
- 确定性列表压缩、排序、持久视图发布仍为串行协调点
- 流场当前为全局烘焙，未支持局部动态障碍增量更新

---

## 性能参考

| 指标 | 数值 |
|------|------|
| 逻辑帧耗时 | < 0.3ms（200 Agents, Ryzen 7） |
| Burst 优化 | 核心 Job 利用 SIMD，零 GC |
| 内存 | NativeContainer 持久分配，无每帧堆分配 |
