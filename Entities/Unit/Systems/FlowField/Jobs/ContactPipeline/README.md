# 增量接触管线

该目录实现 RTS 单位移动中的增量接触发现、预测接触生命周期、软避让与 XPBD 位置约束。

当前管线只有一条权威数据流：

```text
Guarded Swept Proxy
        ↓
Persistent Neighbor Topology
        ↓ versioned classification
Persistent Predictive Contact Lifecycle
        ↓ derived views
Timestep Interaction / Soft / Active / Dormant Views
        ↓
Motion + Soft Avoidance + Wall Projection + Unit Contact Solver
```

旧 Fat AABB / Adaptive 实现不再拥有运行时状态。仍保留的旧字段或类型只用于场景序列化兼容，生产管线内部统一使用新的配置语义。

更完整的所有权与生命周期说明见 [ARCHITECTURE.md](./ARCHITECTURE.md)，当前已知债务见 [DEBT.md](./DEBT.md)。

---

## 当前状态

单位—单位接触位置约束支持两种求解模式：

```text
Gauss-Seidel
Averaged Jacobi-style XPBD
```

运行时可在原有“仿真诊断 → 设置”窗口中切换。诊断视图、四窗口布局和场景 Overlay 暂时沿用原实现，本轮不重做界面。

当前 Jacobi 路径已经建立真正的两阶段数据边界：

```text
所有 Pair 从同一位置快照求值
        ↓ barrier
每条 Pair 写入独立 correction 槽位
        ↓
Body 通过 CSR incident index 汇总关联 Pair
        ↓
每个 Body 统一写回一次位置
```

但整个碰撞管线并没有全部迁移为并行 Jacobi。准确结构是：

```text
串行拓扑 / 分类 / 激活 / 墙体约束
        ↓
并行 Jacobi Pair Evaluate
        ↓
并行 Body Gather / Apply
        ↓
串行统计 / 包络验证 / Repair / Fallback / 速度重建
```

因此 `ParallelJacobi=True` 只表示单位接触投影进入并行路径，不代表整套移动与碰撞求解能够占满全部 Job Worker。

---

## 模块职责

```text
ContactPipeline/
├─ Core/           配置、公共类型、求解器编排入口
├─ BroadPhase/     当前帧 Swept candidate 发现
├─ Persistent/     跨 timestep 拓扑、分类版本和接触生命周期
├─ Prediction/     timestep/substep view、包络验证和统一回退边界
├─ SoftAvoidance/  只消费紧凑 soft view
├─ Motion/         速度准备、位置预测和速度重建
├─ Solver/         Wall、Gauss-Seidel、Jacobi 与并行调度
├─ ARCHITECTURE.md 权威数据流、生命周期和正确性不变量
└─ DEBT.md         已知性能债务和工程债务
```

### Core

只负责配置、公共数据类型和阶段编排。不得重新承担 BroadPhase 缓存、Persistent 生命周期或 Editor 诊断职责。

### BroadPhase

生成帧内候选交互 Pair，不拥有跨帧接触状态。

### Persistent

唯一的跨 timestep 接触权威，拥有：

- Stable Entity Pair；
- Persistent Proxy / Neighbor Pair；
- Predictive Contact Lifecycle；
- Classification Version；
- Stable Normal；
- 用于派生各类 View 的 Stable Key。

### Prediction

负责：

- timestep/substep envelope；
- Interaction / Soft / Active / Dormant View；
- Base motion、Soft/RVO、position integration 和 solver correction 后的安全验证；
- 单一的 incremental repair → full rebuild 回退入口。

### Solver

只消费当前帧紧凑约束，不允许直接修改 Persistent topology。

---

## Gauss-Seidel 模式

Gauss-Seidel 是串行参考实现：

```text
读取 Pair 两端 Body
→ 计算 XPBD Δλ
→ 立即写回两个 Body
→ 下一条 Pair 读取更新后的状态
```

特点：

- 单轮收敛通常更强；
- 数据流简单；
- 当前实现为单 Job 串行热循环；
- 高密度、约束数较多时容易成为主要 CPU 热点。

---

## Averaged Jacobi-style XPBD 模式

Jacobi 模式的 Pair 求值使用标准 XPBD 形式计算候选乘子：

```text
Δλ = -(C + αλ) / (wA + wB + α)
λ' = max(0, λ + Δλ)
```

每条 Pair 保存完整的 `λ'`，并生成两端候选位置贡献：

```text
ΔxA =  n · wA · AppliedLambda
ΔxB = -n · wB · AppliedLambda
```

Body Gather 阶段不会直接把所有贡献完整相加，而是按该 Body 本轮产生非零贡献的约束数量进行平均：

```text
AppliedBodyCorrection = Sum(Incident Contributions) / ActiveContributionCount
```

因此当前实现具有严格的 Jacobi 调度语义，但不是数学上完全一致的原始 XPBD Jacobi：

- Lambda 按完整 `AppliedLambda` 更新；
- 位置修正按 Body 的有效贡献数量衰减；
- 同一 Pair 两端可能使用不同的平均分母；
- 高 degree 单位更稳定，但通常需要更多 iteration；
- Compliance 的实际表现不能直接与 Gauss-Seidel 按相同 iteration 比较。

准确名称是：

> 按 Body 有效约束贡献数归一化的 Averaged Jacobi-style XPBD 接触求解器。

---

## Active Constraint Incident Index

Jacobi Body Gather 使用帧内 CSR 索引：

```text
BodyIndex → 当前 TimestepContactPairs 中关联的 PairIndex
```

对应容器：

```text
ActiveIncidentOffsets
ActiveIncidentPairIndices
ActiveIncidentWriteCursors
```

构建过程：

```text
统计每个 Body 的 active degree
→ Prefix Sum 生成 offsets
→ 将每条 Pair 写入 BodyA、BodyB 的 CSR range
```

该索引只服务当前 Active Constraint View，并不等同于仍未实现的跨 timestep 索引：

```text
Stable Entity → Persistent Neighbor Pair Handle
```

后者仍属于 Persistent topology 的性能债务。

---

## 并行 Job Graph

当前并行 Jacobi 每个 iteration 的依赖链为：

```text
PrepareParallelJacobiIterationJob        IJob，串行
        ↓
EvaluateParallelJacobiPairsJob           IJobParallelForDefer，Pair 并行
        ↓
ReduceParallelJacobiBlocksJob            IJobParallelForDefer，统计 block 并行
        ↓
GatherAndApplyParallelJacobiBodiesJob     IJobParallelFor，Body 并行
        ↓
FinalizeParallelJacobiIterationJob       IJob，串行
```

Substep 外围还有：

```text
InitializeParallelJacobiPipelineJob
PrepareParallelJacobiSubstepJob
FinalizeParallelJacobiSubstepJob
FinalizeParallelJacobiPipelineJob
```

### 当前仍然串行的主要工作

- timestep ContactSet 构建；
- Persistent topology / lifecycle 更新；
- Soft Avoidance；
- Wall Constraint；
- residual 测量；
- envelope validation；
- repair / full rebuild；
- corrected-body flag 汇总；
- statistics publication；
- velocity reconstruction。

因此当前实现的目标是验证 Pair Evaluate 与 Body Gather 的无冲突并行数据流，而不是宣称整个求解器已经充分多核化。

### 当前 batch 设置

```text
Pair batch size = 64
Body batch size = 64
```

实际 Worker 利用率取决于：

- `TimestepContactPairs.Length`；
- Pair batch 数量；
- Active Body 数量；
- 每个并行阶段的工作粒度；
- Prepare/Finalize 等串行阶段占比；
- Unity Job Worker 数量和当帧其他 Job 负载。

即使进入 `ParallelJacobi=True`，也可能只看到 1～2 个 Worker 短暂工作。需要结合 Pair 数、batch 数和阶段墙钟时间判断，而不能只根据“是否使用了并行 API”认定优化有效。

---

## 运行时模式选择与调试回退

并行路径选择条件为：

```text
ContactPositionSolver == Jacobi
&& SelectedPairs capture 未开启
```

当前原有诊断视图中，只要“跨子步接触缓存”窗口可见，就会采集 `SelectedPairs`，Jacobi 会自动回退到串行 Jacobi reference，以保留逐 Pair 诊断数据。

```text
Jacobi + 普通汇总诊断
    → Parallel Jacobi

Jacobi + SelectedPairs capture
    → Serial Jacobi reference

Gauss-Seidel
    → Serial Gauss-Seidel
```

该行为目前沿用原视图，不新增开关。做性能采样时应关闭“跨子步接触缓存”窗口，并确认运行时日志或后续执行路径指标显示：

```text
ParallelJacobi=True
```

注意：`ParallelJacobi=True` 只确认调度分支，不代表 Worker 利用率良好。

---

## Unity Profiler 验证

在 Unity Profiler 中使用：

```text
CPU Usage → Timeline
```

展开：

```text
Job
├─ Worker 0
├─ Worker 1
├─ Worker 2
└─ ...
```

理想情况下，真正并行的两个阶段应在多个 Worker 上存在时间重叠：

```text
EvaluateParallelJacobiPairsJob
GatherAndApplyParallelJacobiBodiesJob
```

不要把所有 Worker 样本时间相加当成帧延迟。并行阶段的实际 wall time 是：

```text
第一个 batch 开始
→ 最后一个 batch 结束
```

当前 Profiler 可能仍把部分嵌套 Job 或内联逻辑显示为外层 `SolveXpbdUnitContactsJob (Burst)`。后续应增加显式阶段 Marker，至少拆出：

```text
Contact.Jacobi.Initialize
Contact.Jacobi.PrepareSubstep
Contact.Jacobi.PrepareIteration
Contact.Jacobi.EvaluatePairs
Contact.Jacobi.ReduceBlocks
Contact.Jacobi.GatherBodies
Contact.Jacobi.FinalizeIteration
Contact.Jacobi.FinalizeSubstep
Contact.Jacobi.FinalizePipeline
```

同时建议补充运行时指标：

```text
Parallel Jacobi Pair Count
Parallel Jacobi Pair Batch Count
Parallel Jacobi Active Body Count
Unity Job Worker Count
```

在这些数据补齐前，当前只能确认并行路径被调度，尚不能证明并行效率或性能收益达到预期。

---

## 统计口径限制

当前 `TotalContactPositionCorrection` 与 `MaxContactPositionCorrection` 来源于平均前的 Pair candidate correction：

```text
PairCorrection = (wA + wB) × abs(AppliedLambda)
```

但 Jacobi 实际施加给 Body 的 correction 已经除以 `ActiveContributionCount`。

所以现有 correction 指标不能公平比较 Gauss-Seidel 与 Averaged Jacobi。后续应区分：

```text
RawPairCorrectionTotal / Max
AppliedBodyCorrectionTotal / Max
ResidualBeforeIteration
ResidualAfterIteration
FinalResidual
```

正式性能对比应以“达到相近残差或穿透误差所需的总时间”为准，而不是只比较相同 iteration 数下的 Solver 毫秒数。

---

## 正确性不变量

1. 每个潜在接触必须位于当前 Interaction View，或关联一个待修复 Dirty Body。
2. Persistent normal 只能在映射到当前 BodyIndex 顺序时完成一次朝向转换。
3. Soft/RVO 输出不能静默逃出已证明安全的 Interaction Envelope。
4. Position integration 与 Solver correction 后必须分别验证对应 envelope。
5. Incremental proof 失败时必须收敛到唯一的 full-sweep fallback。
6. Jacobi Pair Evaluate 必须读取同一轮不可变位置快照。
7. Body Gather 必须按确定性 incident-pair 顺序归约，不使用浮点原子累加。
8. Diagnostics 开启时 Oracle missing-pair count 必须保持为零。

---

## 当前验证方式

至少保留以下 A/B 组合：

```text
Gauss-Seidel：4 iterations
Averaged Jacobi：4 iterations
Averaged Jacobi：8 iterations
Averaged Jacobi：12 iterations
```

同时记录：

```text
Solver wall time
Pair Evaluate wall time
Body Gather wall time
Active Pair / batch count
Worker utilization
Max / Average Penetration
Residual reduction
Envelope Escape Count
Repair / Full Rebuild Count
Oracle Missing Pair Count
```

当前静态 workflow 只能验证目录边界、旧执行路径没有回流，以及 Jacobi 数据结构契约。它不能替代：

```text
Unity C# 编译
Burst 编译
Play Mode
真实 5k 高密度场景性能测试
GS / Jacobi 收敛质量比较
```

---

## 当前已知重点债务

- MotionDirty Body 仍可能触发 O(K) Persistent Pair 扫描；
- Local topology repair 的保守路径仍可能达到 O(K + D×N)；
- 缺少跨 timestep 的 Entity → Persistent Pair incident index；
- Wall、Soft Avoidance、Residual、Envelope Validation 和部分 Finalize 仍串行；
- Jacobi Job Graph 含有较多 iteration barrier；
- batch size 仍需依据 Active Pair 数做实测；
- Solver 热循环仍读写较大的 `FlowMovementFrameState`；
- correction 统计尚未区分 raw Pair candidate 与实际 Body apply；
- Sleeping / Contact Island 尚未实现，本轮也不引入。

README 只描述当前真实边界。任何新的性能结论必须由 Unity Profiler、Profile Analyzer 和固定场景 A/B 数据支持。