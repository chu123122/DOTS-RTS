# Crowd 物理管线架构迁移交接

> 更新时间：2026-07-30  
> 工作区：`/mnt/e/unity/3d/RTS/Assets/Scripts`  
> 分支：`codex/gs-shared-parallel-contact-pipeline`  
> 基线 HEAD：`6b63e13`（`refactor: parallelize certification stage assembly`）  
> 证据口径：本文基于当前含未提交改动的工作树、静态合约脚本和普通 C# 构建；不代表 Unity Editor、Burst、Collections Safety、Play Mode 或 Profiler 已通过。

## 1. 本轮目标与锁定决策

本轮目标不是修改碰撞算法，而是先整理 Crowd locomotion 的数据所有权、抽象层级和调度边界，使上层 Unit/Navigation 只通过一个输入口调用物理管线。

已锁定的架构决策：

| 维度 | 决策 |
|---|---|
| ECS World | 保持同一个 World，不创建第二个 ECS World |
| Unity PhysicsWorld | 保留现有世界查询、Collider、Trigger/Collision Event 和普通刚体职责 |
| Crowd 调度 | 使用独立 `CrowdPhysicsSystemGroup` |
| Crowd 分层 | `BroadPhase → NarrowPhase → CrowdMotionSolver` 三层 |
| Solver 内部顺序 | `SoftAvoidance → Integrate → XPBD → Velocity reconstruction` |
| 跨层数据 | 四个逻辑产品，内部允许 SoA |
| 缓存 | 一个跨帧 `CrossFrameCache`，一个跨 substep 的 `TimestepCache` |
| 写入规则 | 单一写者，可有多个只读消费者；下游不修改上游产品 |
| Unity Physics 交互 | Crowd 权威控制单位 locomotion；Unity Physics 中保留 query proxy |

不创建第二个 World 的原因是当前单位仍需参与攻击、追踪和 Trigger 查询。拆成两个 World 会额外引入 Entity 身份、Transform、Collider、生命周期和 JobHandle 的同步问题，但不能消除这些查询依赖。

## 2. 目标数据流

```text
Unit / FlowField Adapter
        │
        ▼
CrowdPhysicsStepInput                 唯一上层输入产品
        │
        ▼
BroadPhase
        │
        ▼
BroadPhaseCandidateBatch              BroadPhase 唯一写者
        │
        ▼
NarrowPhase                           精确分类、缓存证明/修复、约束定义
        │
        ▼
NarrowPhaseConstraintBatch            NarrowPhase 唯一写者
        │
        ▼
CrowdMotionSolver
  SoftAvoidance → Integrate → XPBD → Velocity reconstruction
        │
        ▼
CrowdPhysicsStepOutput                Solver 唯一写者
        │
        ▼
Unit Writeback
```

缓存只属于 Physics Runtime：

```text
CrossFrameCache
  ├─ BroadPhase proxy/topology
  └─ NarrowPhase predictive/history data

TimestepCache
  ├─ obstacle snapshot copy
  ├─ narrow-phase definitions and schedules
  ├─ soft-avoidance scratch
  ├─ solver runtime / Jacobi CSR
  └─ execution state
```

`Certification` 是 NarrowPhase 内部的缓存证明、增量修复和权威回退机制，不再作为第四个公开抽象层。`XpbdContactKernel` 是 `CrowdMotionSolver` 内部的硬约束数学模块，不应读取 FlowField、Entity Mapping、BroadPhase Cache 或诊断容器。

## 3. 四个跨层产品

### 3.1 `CrowdPhysicsStepInput`

- 来源：Gameplay/Navigation Adapter。
- 消费者：Crowd Physics。
- 当前逐 Body 输入为 `CrowdPhysicsBodyInput`，包含稳定 Entity ID、位置、旋转、速度、期望速度、转向速度误差、移动速度、最大加速度、逆质量、半径和运动标志。
- 不携带 `FlowFieldCell`、IntegrationValue、Arrival 细节或目标点。
- `BuildCrowdMotionIntentJob` 仍读取 FlowField/Arrival，但只负责在 Adapter 边界生成该纯输入；`AdaptCrowdPhysicsStepInputJob` 再将其展开成当前内部 SoA。

### 3.2 `BroadPhaseCandidateBatch`

- 写者：BroadPhase。
- 消费者：NarrowPhase。
- 保存规范化 BodyIndex Pair 及候选来源信息。
- 不负责 Actual/Predictive 精确分类，不拥有 XPBD Lambda。

### 3.3 `NarrowPhaseConstraintBatch`

- 写者：NarrowPhase。
- 消费者：CrowdMotionSolver。
- 提供 Soft interaction view 和硬接触定义 view。
- 当前 `ContactConstraint` 已拆为：
  - `ContactConstraintDefinition`：端点、模式、法线等不可变定义；
  - `ContactConstraintRuntime`：Lambda、激活状态和法线方向历史。
- Runtime 归 Solver/Timestep 生命周期所有，不再由持久候选定义直接携带。

### 3.4 `CrowdPhysicsStepOutput`

- 写者：CrowdMotionSolver。
- 消费者：Unit Writeback。
- 当前输出最终位置、速度和运动结果，由 `BuildCrowdBodyResultsJob` 生成，再由 `ApplyFlowMovementJob` 写回 ECS。
- 朝向、到达状态和动画表现仍应由 Unit 层根据最终速度处理。

四个产品是逻辑产品，不要求每个产品只对应一个 `NativeArray`。具体 Job 继续直接持有 `NativeArray`/`NativeList` 字段，避免隐藏 Unity Collections Safety 的真实读写关系。

## 4. 当前实际调度路径

当前 `BaseFlowMovementSystem` 的主要调度顺序是：

```text
BuildCrowdObstacleSnapshotJob
        └─ FlowField cost → CrowdObstacleCell（过渡实现）

BuildCrowdMotionIntentJob
        └─ FlowField / Arrival / ECS components → CrowdPhysicsBodyInput

AdaptCrowdPhysicsStepInputJob
        └─ StepInput → internal Body / NavigationState / MotionIntent SoA

InitializeCrowdStepStateJob
        ↓
CrowdPhysicsPipelineComposition.ScheduleStep
        ↓
BuildCrowdBodyResultsJob
        ↓
ApplyFlowMovementJob
```

`CrowdPhysicsPipelineComposition.ScheduleStep` 是当前唯一的托管装配入口。它负责从两个缓存展开能力受限的资源切片，并连接现有 Lifecycle、Broad/Narrow certification、SoftAvoidance、Wall、XPBD、repair、velocity reconstruction 和 diagnostics publication。

`BaseFlowMovementSystem` 仍负责输入采集、临时资源分配、配置冻结、诊断发布和 ECS 写回，因此它现在是 Gameplay Adapter，而不是算法实现层。

## 5. 已落地内容

### 5.1 调度与数据边界

- 增加四个公开逻辑产品：
  - `CrowdPhysicsStepInput`
  - `BroadPhaseCandidateBatch`
  - `NarrowPhaseConstraintBatch`
  - `CrowdPhysicsStepOutput`
- 增加单一 `CrowdPhysicsPipelineComposition.ScheduleStep` 门面。
- `BaseFlowMovementSystem` 不再手工拼装整套 Certification/Solver 资源袋。
- `CrowdPhysicsSettings` 在 step 开始时冻结物理设置；`ContactPipelineConfiguration.Create` 不再直接接收完整 `FlowFieldSettings`。
- `CrowdPhysicsSystemGroup` 已加入同一 ECS World，并位于 `FlowFieldBakeSystem` 之后。

### 5.2 缓存所有权

- 原 `InteractionCandidateStore` 已收敛为 World 生命周期 `CrossFrameCache`。
- 跨帧容器为私有字段；只有 Physics 内部 Scheduler 取得精确容器字段，
  Gameplay 通过 runtime lease 访问输入/输出。
- Solver 无法直接取得整个跨帧缓存。
- `TimestepCache` 统一拥有本 timestep 内可跨 substep 的 NarrowPhase、SoftAvoidance、Solver、Execution 和障碍数组。
- 缓存仍只是优化：验证失败必须走增量修复或权威全量构建。

### 5.3 Solver 边界

- `ContactConstraintDefinition` 与 `ContactConstraintRuntime` 已拆分。
- XPBD/Jacobi Solver 主路径已移除不必要的 FlowField 读取。
- GS/Jacobi 共用 Broad/Narrow 生命周期和约束数据，只在 XPBD 投影实现处分叉。
- Release 路径不再无条件在 step 末尾执行 `Dependency.Complete()`；容量增长、销毁和显式诊断探针仍可合法同步。

### 5.4 环境与 Unity Physics 适配

- 增加 `CrowdObstacleCell` 和带版本的 `CrowdObstacleSnapshot`。
- 接触管线下游不再直接解释 `FlowFieldCell.Cost`。
- 当前仍由 `BuildCrowdObstacleSnapshotJob` 将 FlowField `Cost == 0` 翻译为物理占据，这是过渡发布源。
- 增加独立 `CrowdDiscShape`；单位半径在初始化阶段从 `PhysicsCollider` 推导一次，物理 step 不再每帧推导。
- 增加 `CrowdQueryProxy` 标记，保留单位 Collider 供 Unity Physics 查询使用。

### 5.5 文档和静态合约

- `Physics/ContactPipeline/ARCHITECTURE.md` 已记录目标层级、证书边界、生命周期和正确性不变量。
- `Physics/ContactPipeline/DEBT.md` 已区分已完成结构迁移与运行证据债务。
- 四个静态脚本已扩展，用于防止旧目录、聚合资源袋、诊断反向控制和跨层访问回归。

## 6. 当前结构状态

### 6.1 已闭合的 API 和程序集边界

- `Physics/RTS.Physics.asmdef` 不引用 Gameplay；
- `InternalsVisibleTo("RTS.Gameplay")` 已删除；
- Gameplay 只通过 `CrowdPhysicsRuntime`、`CrowdPhysicsStep` 和
  `CrowdPhysicsDiagnosticsStep` 提交输入、调度、读取输出和发布诊断；
- `CrossFrameCache`、`TimestepCache`、帧资源、证书 Job 和 Solver 数组均不再
  出现在 Gameplay 源码；
- UI/Input/场景组合保留在终端 `Assembly-CSharp`，因为 `QFramework` 和生成的
  `PlayerAction` 同属默认程序集。此边界是明确约束，不伪造不可编译的
  `RTS.UI.asmdef`。

### 6.2 已删除的旧结构

- `InteractionCertificationAlgorithms`、`CertificationStageKernel`；
- 全部 `Certification*Resources` 和转发属性；
- `CreateAlgorithms()`、`CreateNarrowPhaseResources()`；
- `IncrementalPredictiveContactKernel.cs`、`ContactEnvelopeGuardKernel.cs`、
  `SweptDiscBroadPhaseKernel.cs`；
- `CrowdBodyStepState`、`SortJobDefer` 和串行
  `PrepareSubstepRepairTopologyJob`。

持久 Contact 现在只有权威
`NativeList<PersistentPredictiveContact>` 和派生 key→index。具体 Stage 不再
调用另一个 Stage 的 DataFlow，共享操作迁移到中性 Kernel。

### 6.3 当前单向数据流

Gameplay 只写 `CrowdPhysicsStep.InputBodies`；Physics 内部一次性展开为 SoA。
避免、Solver 和运动证明分别由 `CrowdAvoidanceState`、
`CrowdSolverBodyState`、`CrowdMotionEvidence` 承载。Solver 可写自己的 runtime；
认证修复只能按 Scheduler 依赖读取已完成的 Solver 状态并发布新的认证产品，不能
直接回写 Solver 或由 Solver 修改持久缓存。

### 6.4 当前并行链

- Dirty Body：`RefreshDirtyBodiesJob(IJobParallelForDefer)` → Reduce；
- Dirty Contact/Schedule：Count → Prefix → Scatter；
- Full Sweep：Body Cell Count/Prefix/Scatter → Cell Pair
  Count/Prefix/Scatter → block sort → merge passes → copy-if-needed →
  Deduplicate；
- Substep Repair：串行 prepare 只检查/扩容，两个大列表复制由独立
  `IJobParallelForDefer` 完成；
- GS/Jacobi 只在 XPBD contact projection 分叉，其余前后阶段共享。

当前仍缺的是这些新链在最新源码上的 Unity/Burst/Collections Safety 和代表性
Profiler 复验，不能只凭静态结构声称性能已经提升。

## 7. Unity PhysicsWorld 职责边界

### Unity Physics 保留

- 静态和普通动态刚体；
- Collider Authoring；
- Raycast、Overlap、攻击和追踪查询；
- Trigger/Collision Events；
- 障碍几何快照的发布来源。

### Crowd Physics 权威负责

- 接收单位期望速度；
- 单位间 SoftAvoidance；
- 预测 Disc Contact；
- 单位—单位和单位—障碍 XPBD；
- 提交单位最终位置和速度。

### 目标同步协议

```text
NavigationIntent
    ↓
CrowdPhysics step N
    ↓
提交单位 Transform / Velocity，标记 CrowdStepVersion = N
    ↓
同步 Unity query proxy
    ↓
Unity PhysicsWorld build，记录 ProxyVersion = N
    ↓
攻击 / 追踪 / Trigger 查询声明其使用的版本
```

在该协议和过滤规则完成前，不得宣称 Unity Physics query proxy 已经与 Crowd locomotion 完全同步。

## 8. 验证证据

### 8.1 当前已取得

| 验证项 | 当前结果 | 证据等级 |
|---|---|---|
| 四个静态合约脚本 | 已通过当前代码的定向执行 | 静态检查 |
| `git diff --check` | 已通过文档创建前的当前代码检查 | 文本差异检查 |
| `RTS.Gameplay.csproj` 普通构建 | 42 warnings，0 errors，12.31 秒 | 普通 C# 构建 |
| `RTS_CONTACT_DIAGNOSTICS` 构建 | 最新缓存封装修改后未重新执行 | 未验证 |
| Unity Editor 编译 | 未执行 | 未验证 |
| Burst 编译/Inspector | 未执行 | 未验证 |
| Collections Safety | 未执行 | 未验证 |
| Play Mode 行为 | 未执行 | 未验证 |
| Profiler 分配/同步点 | 未执行 | 未验证 |

普通 `.csproj` 构建中的主要警告包括 Unity API obsolete 警告、`System.ValueTuple` 引用版本冲突和两个未使用的诊断 trace signature。它只能证明当前生成的 C# 工程没有编译错误，不能替代 Unity Editor/Burst 证据。

### 8.2 必须补做的运行验收矩阵

| 场景 | 必须证明 |
|---|---|
| Cache OFF | 作为权威 Pair、Constraint 和输出基线 |
| CrossFrame ON | 与 Cache OFF 得到等价的约束集合和物理结果 |
| Timestep ON | 与 Cache OFF 得到等价的约束集合和物理结果 |
| Body 重排/创建/销毁 | Entity→BodyIndex 映射和拓扑正确失效 |
| 半径/Shape 变化 | ShapeVersion 触发缓存失效 |
| Settings 变化 | configuration fingerprint 触发失效 |
| ObstacleVersion 变化 | 不复用旧障碍约束 |
| 守护包络逃逸 | 只产生 violation/rebuild request，并在消费前完成修复 |
| Settled 单位 | 停止主动移动，但继续接受硬接触修正 |
| GS/Jacobi | 共用相同 Broad/Narrow 数据，轨迹和 Pair 集合按容差对比 |
| Query proxy | 攻击、追踪和 Trigger 读取已声明的 Crowd step 版本 |
| Diagnostics OFF | 不创建额外 gameplay 容器/Job/Profiler 读取，不强制末尾 Complete |
| 多 World/重播 | WorldId 隔离、确定性 hash 和缓存状态互不污染 |

## 9. 后续验收顺序

1. 重新运行四个静态脚本、普通和 Diagnostics 构建；
2. 让已启动 Editor 重新导入最新门面与 Job 拆分，检查 Console；
3. 分别验证 Cache OFF、Timestep、CrossFrame，以及 GS/Jacobi；
4. 开启 Collections Safety 覆盖 full sweep、repair、异常释放；
5. 用 Profiler 对比 `SortBroadPhasePairBlocksJob`/merge passes 和两个 repair copy
   Job，记录单位数、pair 数、线程数和采样帧；
6. 最后执行完整攻击、追踪、Trigger 和 Diagnostics OFF 回归。

## 10. 关键文件

| 文件 | 当前职责 |
|---|---|
| `Gameplay/Entities/Unit/Systems/FlowField/BaseFlowMovementSystem.cs` | Gameplay Adapter、配置冻结、Physics lease 调度、结果写回 |
| `Gameplay/Entities/Unit/Systems/FlowField/LocalUnitFlowMovementSystem.cs` | `CrowdPhysicsSystemGroup` 和本地移动系统调度 |
| `Gameplay/Entities/Unit/Systems/FlowField/Runtime/BuildCrowdMotionIntentJob.cs` | FlowField/Arrival → `CrowdPhysicsBodyInput` Adapter |
| `Physics/ContactPipeline/Scheduling/CrowdPhysicsRuntime.cs` | 唯一公开 Physics runtime/step/diagnostics lease |
| `Physics/ContactPipeline/Scheduling/CrowdContactPipelineScheduler.cs` | Physics 内部托管管线装配 |
| `Physics/ContactPipeline/State/Frame/CrowdStepBodyResources.cs` | StepInput/StepOutput 逻辑产品及内部 Body SoA |
| `Physics/ContactPipeline/State/Frame/BroadPhaseFrameResources.cs` | Full Sweep、持久复用和分块排序工作集 |
| `Physics/ContactPipeline/State/Frame/ContactProductFrameResources.cs` | BroadPhase/NarrowPhase 对外消费产品 |
| `Physics/ContactPipeline/State/Frame/ContactClassificationFrameResources.cs` | 分类结果和 publication block/workset |
| `Physics/ContactPipeline/State/Frame/ContactRepairFrameResources.cs` | dirty、repair 和 persistent incident 重建工作集 |
| `Physics/ContactPipeline/State/Frame/ContactCertificateFrameResources.cs` | contact scratch、schedule 和 certificate |
| `Physics/ContactPipeline/State/Frame/ContactPipelineExecutionResources.cs` | `TimestepCache` |
| `Physics/ContactPipeline/State/Persistent/InteractionCandidateStore.cs` | `CrossFrameCache` |
| `Physics/ContactPipeline/Contracts/Interaction/ContactConstraint.cs` | Constraint Definition/Runtime |
| `Physics/ContactPipeline/Contracts/Interaction/CrowdEnvironmentViews.cs` | versioned obstacle snapshot |
| `Physics/ContactPipeline/ARCHITECTURE.md` | 目标架构和不变量 |
| `Physics/ContactPipeline/DEBT.md` | 已完成迁移、运行债务和明确排除项 |

## 11. 建议复验命令

从 `/mnt/e/unity/3d/RTS/Assets/Scripts` 执行：

```bash
python3 .github/scripts/validate_contact_architecture.py
python3 .github/scripts/validate_contact_diagnostics.py
python3 .github/scripts/validate_contact_pipeline_audit.py
python3 .github/scripts/validate_contact_static_contracts.py
git diff --check
```

普通 C# 构建应从项目根目录执行，并把完整日志单独保存：

```bash
cd /mnt/e/unity/3d/RTS
dotnet build RTS.Gameplay.csproj --no-restore
```

Diagnostics define 的具体构建参数应沿用项目当前验证脚本或上一次已确认命令；不要为绕过失败临时修改项目环境、生成工程或全局编译常量。即使两个 `.csproj` 构建均通过，仍必须在 Unity Editor 中单独验证 Burst 和 Collections Safety。

## 12. Git 与工作树警告

当前工作树在本交接文档创建前已经包含大量已修改、已删除和未跟踪文件。这些变更不能仅凭 `git status` 归属给单一任务或单一执行者。

交接后操作必须遵守：

- 禁止 `git add .` 或 `git add -A`；
- 禁止全局 `git stash`、`git reset --hard`、`git clean`；
- 不覆盖现有 `FAT_AABB_CACHE_TASK_HANDOFF.md`；
- 提交前按精确路径审查和暂存；
- 先用 `git diff -- <path>` 区分本轮文档与继承改动；
- Unity 生成物、缓存和 `.csproj` 变化不得混入架构提交；
- 若需拆分提交，优先在隔离 worktree 中按所有权边界重放，而不是整理当前混杂工作树。

本交接文件只负责记录当前架构迁移状态，不代表上述未提交源码已经完成归属审计、提交或运行验收。

## 13. 2026-07-30 续作结果

本节覆盖前文中关于步骤 7、步骤 8 和 Query Proxy “尚未证明”的旧状态。

### 13.1 Query Proxy 协议已接入

- Crowd 写回同时提交 `CrowdQueryProxy.CrowdStepVersion`。
- `CrowdQueryProxyPublicationSystem` 位于 `PhysicsInitializeGroup` 之后、
  `PhysicsSimulationGroup` 之前；它只在 PhysicsWorld build 已消费 Transform
  后把 `CrowdStepVersion` 发布为 `ProxyVersion`。
- 攻击和追踪查询移入 `CrowdQuerySystemGroup`，该组位于
  `FixedStepSimulationSystemGroup` 之后、`CrowdPhysicsSystemGroup` 之前。
- 攻击、追踪和 Trigger 产物携带实际消费的 `QueryProxyVersion`；攻击消费端会
  再次验证 source/target 版本没有变化。
- `CrowdQueryCollisionFilters` 统一定义 Ground、Unit、Obstacle 和 Unit overlap
  查询过滤器。三个 Unit prefab 的 Unit body 已关闭 Unit—Unit Unity Physics
  响应；Crowd XPBD 仍是单位 locomotion 的唯一权威。
- 障碍发布已从 FlowField cost 翻译切到 `PhysicsWorldSingleton.CollisionWorld`
  距离查询，并以 `ObstacleVersion` 发布不可变快照。

### 13.2 运行验证结果

实际使用已启动的 Unity `6000.0.27f1c1` Editor 验证，而非只使用生成的
`.csproj`：

| 验证项 | 结果 | 边界 |
|---|---|---|
| Unity Editor 编译 | 通过 | 当前源码重新导入并完成 domain reload |
| Burst/Entities/Jobs ILPP | 通过 | 当前运行程序集均完成 IL 后处理 |
| Collections Safety | 通过定向场景 | 验证过程中发现并修复 3 组缺失容器绑定，最终无异常 |
| Cache OFF / Timestep / CrossFrame | 输出等价 | 两个重叠单位、四步、Transform/Velocity 容差 `0.0005` |
| GS / Jacobi | 输出近似等价 | 共用场景，Transform/Velocity 容差 `0.02` |
| Jacobi incident CSR | 多接触通过 | 四单位接触链完成 Count/Prefix/Scatter/确定性 range sort 并保持分离 |
| Query proxy | 通过 | Crowd commit `9`，Physics publish `9` |
| 攻击 / 追踪 | 通过 | 同版本命中，旧版本 target 被拒绝 |
| Trigger 版本消费 | 通过 | version `7` 被接受，stale version `8` 被伤害汇总拒绝 |
| Diagnostics OFF | 通过 | 无诊断 singleton 时移动和接触管线可完成 |
| Profiler | 基础接入通过 | `RTS.Simulation.Update` recorder 可绑定；未形成 5k 性能结论 |
| Play Mode | 短时烟测通过 | `ConnectionScene` 约 14 秒，无新的脚本/Collections 异常 |

本轮运行验证还暴露并修复：

- Cache OFF 的 `PrepareTimestepPredictionBodiesJob` 未绑定持久容器；
- 两个 deferred Scatter Job 未持有其迭代计数容器；
- persistent classification 首条调度未绑定 `MotionEvidence`。
- 新增的 full-sweep/active-incident deferred workset 必须同时作为 Job 字段绑定；
  consumer validation 直接读取既有 `Persistent` slice，不能再别名绑定同一 dirty
  list。

这些绑定现在由 `.github/scripts/validate_contact_static_contracts.py`
追加检查，防止同类安全错误回归。

### 13.3 最终构建和静态证据

- 四个静态合约脚本全部通过；
- `git diff --check` 通过；
- 普通构建：`RTS.Shared`、`RTS.Gameplay.Core`、`RTS.Physics`、
  `RTS.Gameplay`、`RTS.Network`、`RTS.Physics.Editor` 均为 `0 errors`；
- `RTS_CONTACT_DIAGNOSTICS` 构建：上述六个程序集均为 `0 errors`。

仍未取得的证据是代表性 5k 单位 Profiler 捕获及完整玩法场景中的真实 Trigger
碰撞事件回放；当前 Trigger 证据覆盖事件进入 damage 分支后的版本数据流，不把它
表述为完整关卡回归。

### 13.4 Finalize/Repair 并行收口

- 初始分类不再调用 `RefreshPersistentPairSourceForClassification`；该 dead
  legacy 已删除。
- 持久拓扑由 `PreparePersistentTopologyPublicationJob` →
  `BuildPersistentProxiesJob` →
  `BuildPersistentProxyIndexJob` → `PublishPersistentNeighborPairsJob` →
  `FinalizePersistentTopologyPublicationJob` 发布；代理和邻居顺序直接继承上游
  已排序、去重的权威 Pair 流，不再重复串行排序。
- `FinalizePreparedSubstep`、Wall iteration finalize 和 Contact iteration
  finalize 只产生 dirty/recovery 状态；权威修复统一回到
  `SchedulePersistentRepairStages`，不再在 Finalize `IJob` 内嵌套串行
  repair/full rebuild。
- Cache OFF、Timestep-only 和 CrossFrame 三种模式现在都走
  Full Sweep → staged topology publication → parallel classification →
  deterministic commit。`PersistentRepairKernel` 中原先转发到
  `BuildSubstepInteractionAndSoftViews`、`BuildOrRefreshTimestepContactViews`
  和 `BuildSubstepContactView` 的活动串行入口已删除。
- CrossFrame 帧首会先检查 dirty body 和持久 contact-view 证书：有 dirty
  或完整结构/配置指纹失配时展开 Full Sweep。指纹覆盖 proxy/index 容量、
  `ObstacleVersion`、guard/predictive/contact margin、soft/RVO 参数、
  substep/predictive 开关和 solver mode；无 dirty 且指纹有效时，Full Sweep/Topology
  workset 保持为 0，改走 `PreparePersistentReusePublicationJob` →
  `MapPersistentReusePairsJob` → 共享分块排序/归并/去重 →
  `FinalizePersistentReusePublicationJob`，把持久 entity pair 并行映射到当前
  body index 后复用同一 classification/publication。
- Full Sweep 以 deferred body workset 表示 repair request；持久缓存有效且
  无 dirty 时
  body/cell/pair 工作集长度为 0，topology proxy/index/pair job 同样不遍历旧
  容器。Dirty Contact/Schedule 的 prepare 在 dirty list 为空时发布 0 个
  block，commit 保留既有 contact/schedule。
- Jacobi active-incident CSR 改为证书指纹 Prepare → Clear → Count →
  Prefix → Scatter → per-body deterministic sort。证书和 contact view
  未变化时，body/pair deferred workset 均为空，不重复重建 CSR。GS 与
  Jacobi 复用上述修复和索引前后阶段，求解分叉只留在 XPBD contact solve。
- 所有会继续调度子链的 scheduler helper 使用 `ref JobHandle`，确保 helper
  内部调度异常时，外层 catch 仍能完成最后一个成功入队的 Job，再释放帧资源。

验证边界：上述 early-out 已由源码契约和普通构建覆盖，但仍不等于 5k 单位性能
结论；真正发生 dirty 的 frame 仍采用全量 staged sweep/publication，尚未用
代表性 Profiler 捕获证明其耗时优于旧增量 repair。

### 13.5 删除优先最终收口与公开边界

- 全部 Certification 资源袋、旧转发属性和旧 Algorithms 文件已删除；
- `IncrementalPredictiveContactKernel.cs` 已按持久视图、预测激活和 timestep
  repair view 拆分，旧文件删除；
- Full Sweep 的 `SortJobDefer` 已替换为 prepare → parallel block sort →
  显式 merge pass → parity copy → deduplicate；
- Substep repair 的 prepare 只检查/扩容，两个大列表复制由独立
  `IJobParallelForDefer` 完成；
- `CommitPersistentClassificationJob`、`CommitSubstepRepairJob` 已删除。分类
  结果统一走 `Prepare → Materialize → Count → Prefix → Scatter`；初始路径
  再执行状态发布、Oracle、Certificate 三个明确 Job，修复路径再执行状态发布、
  线性 contact merge、并行清除 escape、并行 incident lookup、Certificate。
- 修复 contact view 不再对每个新 Pair 二分查旧列表后全量 sort/deduplicate，
  而是合并两个已排序流，复杂度从 `O(n log n)`（且带重复二分）收敛为 `O(n)`。
- `CrowdBodyStepState` 改为正式的 `CrowdSolverBodyState`，软避让字段迁到
  `CrowdAvoidanceState`；
- `InternalsVisibleTo("RTS.Gameplay")` 已删除。Gameplay 只使用
  `CrowdPhysicsRuntime`、`CrowdPhysicsStep`、
  `CrowdPhysicsDiagnosticsStep`，不再取得任何缓存、帧资源或 Solver scratch；
- 诊断代理按索引返回值，不再把持久 `NativeList` 可变别名交给 Gameplay。

当前验证：

- 六个普通程序集 `.csproj` 均 `0 errors`；
- Diagnostics：Shared、Gameplay.Core、Physics、Gameplay、Network 均
  `0 errors`；Physics.Editor 目标程序集在
  `BuildProjectReferences=false` 下 `0 errors`。递归构建 Physics.Editor 时，
  Unity 的 `com.unity.scriptablebuildpipeline` 工程会因其生成工程
  `CalculateAssetDependencyData.Version` 报 `CS0120`，未修改包或环境掩盖；
- 四项静态契约和 `git diff --check` 通过；
- 已启动的 Unity `6000.0.27f1c1` 完成最新 C# 编译、Entities/Jobs/Burst ILPP
  和 domain reload；
- 编辑器内置 `LOCAL_GAMEPLAY_VALIDATION_OK` 通过：Cache OFF/Timestep/
  CrossFrame 输出等价、GS/Jacobi 近似等价、Jacobi 多接触 incident、Query
  Proxy、攻击/追踪、Trigger 版本链及 Diagnostics 可选路径均通过；
- 本轮没有新的代表性 Profiler 捕获，因此不声称 20.29 ms / 8.27 ms 已达到某个
  性能数值，只确认对应串行实现已不在调度图。

本轮最新 Editor 验证先后暴露并修复了三项仅靠 `.csproj` 编译无法发现的
Collections/Burst 问题：

- `PrepareClassificationPublicationJob.ContactIndex` 未绑定；
- publication block 同时作为 deferred iteration list 和可写 block 数组产生
  alias，现已拆成独立 `NativeList<byte>` workset；
- repair 线性 merge 在“旧流剩余项全部 dirty、而新流已耗尽”时会越界，现已
  改为仅在两个流都有当前项时比较，再分别消费尾部；匹配到 dirty 旧 Pair 时仍
  保留旧 timestep runtime，并加入静态回归约束。

修复后重新取得 `LOCAL_GAMEPLAY_VALIDATION_OK`，并在 `ConnectionScene`
连续 Play Mode 约 20 秒；新增日志中无脚本异常、Collections 异常或 Native
Collection leak。该结果仍不是 5k 单位性能结论。
