# 固定拓扑约束与动态接触约束统一框架：设计与实验计划

## 1. 选题定义

### 1.1 项目名称

**固定拓扑约束与动态接触约束的统一求解框架，以及基于时间相干性的动态碰撞对优化**

英文工作名：

> Unified Constraint Solver for Fixed-Topology Constraints and Dynamic Contacts with Temporal-Coherent Pair Optimization

### 1.2 核心问题

本项目不试图证明“布料模拟与群体寻路是同一种模型”，而是研究两者能否共享同一套约束求解生命周期：

1. 自由运动或外部驱动产生预测位置；
2. 固定拓扑直接提供约束，动态接触通过宽相和窄相产生活动约束；
3. 求解器迭代投影约束；
4. 修正位置并回写速度；
5. 动态接触利用时间相干性减少碰撞对生成和求解成本。

两类模型的差异保留在驱动和约束来源中：

| 模型 | 驱动 | 固定拓扑约束 | 动态接触约束 |
| --- | --- | --- | --- |
| 布料 | 重力、风、惯性 | 拉伸、剪切、弯曲、固定点 | 自碰撞、外部碰撞 |
| RTS 群体 | Flow Field、期望速度、软避让 | 通常没有 | 单位接触、墙壁接触 |

### 1.3 术语边界

- **固定拓扑约束（Fixed-Topology Constraint）**：约束关系由初始化拓扑决定，例如布料边和弯曲约束。这里不使用“静态碰撞”称呼它，因为静态碰撞通常指动态物体与静态几何体发生接触。
- **动态接触约束（Dynamic Contact Constraint）**：运行时根据空间关系生成和失效的单边非穿透约束。
- **候选碰撞对（Candidate Pair）**：宽相认为可能接近，但尚未确认发生接触的对象对。
- **活动接触对（Active Contact Pair）**：窄相确认违反非穿透约束、需要进入求解器的对象对。
- **邻域候选对（Neighbor Pair）**：位于接触半径加 skin 范围内、可用于时间相干缓存的候选对。
- **接触缓存（Contact Cache）**：保存持续接触的法线、lambda、生命周期等求解状态；它与邻域候选缓存不是同一层结构。

## 2. 当前 RTS 基线

### 2.1 当前运动流水线

当前 `BaseFlowMovementSystem` 按以下顺序调度：

1. 计算不依赖邻居的 Flow Field 和到达控制力；
2. 基于当前位置和 `UnitSpatialMap` 计算软避让力；
3. 半隐式欧拉积分，生成所有单位的预测位置快照；
4. 基于当前位置构建的 Spatial Hash 筛选邻居，在预测位置上进行硬约束判断；
5. 累计单体位置修正并写回位置，约束修正暂不反推速度。

关键文件：

- `Entities/Unit/Systems/FlowField/BaseFlowMovementSystem.cs`
- `Entities/Unit/Systems/FlowField/UnitSpatialPartitionSystem.cs`
- `Entities/Unit/Systems/FlowField/Jobs/CalculateFlowConstraintsJob.cs`
- `Entities/Unit/Systems/FlowField/Jobs/IntegrateFlowForcesJob.cs`
- `Entities/Unit/Systems/FlowField/Jobs/ApplyFlowMovementJob.cs`

### 2.2 当前已经存在什么

- 存在 `cell -> Entity` 的 Spatial Hash；
- 每个单位会实时枚举周围九宫格内的邻居；
- 存在单位之间的成对距离判断；
- 窄相使用同一帧预测位置快照；
- 存在硬接触位置修正。

### 2.3 当前缺少什么

当前并不是“完全没有碰撞对概念”，而是碰撞对只存在于循环执行过程里，没有成为显式数据：

- 没有唯一 `PairKey`；
- `(A,B)` 与 `(B,A)` 会从双方视角分别处理；
- 没有帧级 Candidate Pair / Active Contact 快照；
- 没有可供多轮求解复用的接触约束数组；
- 没有跨帧生命周期和相干性统计；
- 没有跨帧 Contact Cache 或 lambda；
- 软避让与硬约束分别枚举邻居；
- 当前位置宽相与预测位置窄相之间仍可能出现跨格漏对；
- 完全重合时当前逻辑会因距离过小而跳过，不能产生稳定分离法线；
- 当前约束是每个单位独立累计修正，不是唯一 Pair 上同时按质量权重修正双方；
- 当前只有单轮位置修正，尚不是完整的迭代 PBD/XPBD 接触求解器。

## 3. 总体实验路线

| 阶段 | 变量 | 目的 |
| --- | --- | --- |
| 实验一 | 旁路生成唯一 Pair 和统计，不改变运动 | 验证碰撞图是否具有可利用的时间相干性 |
| 实验二 | 每帧建立唯一 Pair 快照并供多轮求解复用 | 测量显式 Pair 的成本与多轮复用收益 |
| 实验三 | 带 skin 的 Neighbor/Verlet Cache | 减少 Spatial Hash 和候选 Pair 的重建频率 |
| 实验四 | Hot Contact 与 XPBD Warm Start | 降低达到相同残差所需的迭代次数 |
| 实验五 | 接入布料固定拓扑和自碰撞 | 验证统一求解调度器，而不是只实现两个相邻 Demo |

所有优化版本必须在相同单位数、随机种子、时间步和误差标准下，与基线比较。

### 3.1 时间相干性必须按层级测量

“动态系统具有时间相干性”不等于“精确Primitive Pair适合跨帧缓存”。时间相干性可能分别存在于：

1. **Broad Phase区域层**：相邻帧仍是相同Spatial Hash Cell、相邻Cell或BVH节点发生重叠；
2. **邻域候选层**：扩大skin后的近邻关系持续存在；
3. **精确Primitive Pair层**：具体单位Pair、Vertex-Face Pair或Edge-Edge Pair持续存在；
4. **接触区域层**：具体Primitive Pair发生替换，但仍属于同一局部接触区域或接触簇；
5. **求解状态层**：法线、lambda或接触方向能够被安全Warm Start。

项目不能预设相干性一定集中在第3层。尤其在布料滑动、折叠和多层接触中，具体Vertex-Face/Edge-Edge Pair可能快速更替，而BVH拓扑、重叠节点和接触区域更稳定。缓存精确Pair还可能因Pair数量膨胀、哈希身份、失效维护和随机内存访问导致空间成本高于节省的宽相时间。

因此后续缓存实验必须同时报告：

- 缓存位于哪个层级；
- 该层级的命中率、失效率和生命周期；
- 缓存字节数及峰值容量；
- 每帧维护、验证和失效成本；
- 相对完全重建实际节省的时间；
- 是否减少了最终窄相或求解迭代，而不只减少某个局部步骤。

### 3.2 统一的是求解接口，不强求统一碰撞生成后端

RTS圆盘单位与三角形布料可以共享预测、约束表示、求解调度和速度回写，但不要求共享同一个Broad Phase实现：

```text
CrowdSpatialHashContactGenerator
    -> DynamicContactConstraintBuffer

ClothBVHContactGenerator
    -> DynamicContactConstraintBuffer

FixedClothConstraintProvider
    -> FixedConstraintBuffer

DynamicContactConstraintBuffer + FixedConstraintBuffer
    -> UnifiedConstraintSolver
```

该边界避免为了“统一”而把Spatial Hash强行用于所有三角形自碰撞，或把BVH引入当前只需要圆盘邻域查询的RTS系统。

## 4. 实验一：旁路碰撞对观察模型

### 4.1 实验目的

实验一不让新生成的 Pair 参与运动，不改变当前软避让和位置修正结果。它只回答：

1. 当前沙漏瓶颈场景中，活动接触对跨帧延续程度有多高；
2. 活动接触和扩大邻域候选，哪一种具有更稳定的时间相干性；
3. 当前宽相和窄相是否存在漏对、重复或退化接触；
4. 显式 Pair 快照本身需要多少生成时间和内存；
5. 后续应优先尝试 Active Contact Cache、Neighbor Cache，还是继续每帧重建。

### 4.2 场景

第一场景使用现有沙漏形瓶颈：单位从宽区域进入狭窄通道，再离开通道。该场景同时包含：

- 大量候选邻居；
- 瓶颈处持续接触；
- 进入和离开瓶颈时的 Pair 更替；
- Flow Field 主动驱动与接触约束冲突；
- 高度不均匀的动态接触图。

该场景适合作为压力测试，但不能作为时间相干性结论的唯一场景。后续至少补充：

1. 同向密集流动：预期高相干；
2. 双向对冲：预期高碰撞、高 Pair 更替；
3. 开阔稀疏移动：预期低接触；
4. 高速或传送：缓存失效边界。

### 4.3 观察集合

实验必须分开记录以下集合，不能把宽相候选等同于真实接触。

#### CandidatePairs

由当前 `UnitSpatialMap` 九宫格查询得到的唯一单位对。它反映当前宽相产生的数据规模。

#### SoftInteractionPairs

基于当前位置，满足软避让作用距离的唯一单位对。它对应当前速度空间软避让邻域。

#### ActiveContactPairs

从当前Spatial Hash候选中，基于双方预测位置并按现有求解器判定规则得到的唯一单位对。它表示当前实现实际能够看到的活动动态约束。

#### DegeneratePairs

距离平方低于当前法线计算阈值的Pair。当前求解代码会跳过这些Pair，但统计必须单独记录，否则“没有活动接触”可能只是零距离退化被静默忽略。

#### GeometricOverlapPairs（Oracle模式）

不依赖当前Spatial Hash，直接基于所有单位预测位置计算的几何重叠真值集合。它包含近零距离重叠，并与ActiveContactPairs分开，专门用于暴露“当前位置宽相、预测位置窄相”造成的漏对以及退化Pair。

#### NeighborPairs（实验一可选，实验三必需）

满足以下距离的Pair：

```text
NeighborRadius = MaxInteractionRadius + Skin
```

它用于验证扩大邻域是否比精确活动接触更稳定。实验一可以只采集若干固定 skin 配置，不让它参与运动。

#### SpatialRegionPairs

记录发生候选邻域或活动接触的Spatial Hash区域关系，例如规范化后的Cell Pair、活跃Cell集合和每个Cell内Pair churn。它不替代具体单位Pair，而是用于比较：

```text
区域关系延续率 vs 邻域Pair延续率 vs 精确Active Pair延续率
```

如果精确Pair快速更替但区域关系长期稳定，后续优化应优先考虑区域/Cell级增量更新，而不是持久保存全部精确Pair。

### 4.4 第一阶段范围

实验一首先只统计单位—单位Pair：

- 不把墙壁格子混入单位Pair集合；
- 墙壁接触作为后续独立的 `Body-StaticGeometry` Pair 类型；
- 不在本阶段修改单位碰撞半径、软避让半径或墙壁近似；
- 不在本阶段加入多轮投影、lambda、Warm Start或速度回写。

这样可以保证所有统计变化来自观察模型，而不是求解行为变化。

## 5. 实验一需要实现的数据结构

以下为职责定义，不预先锁死具体 Unity Collections 类型。

### 5.1 PairKey

```text
PairKey
    BodyA
    BodyB
```

要求：

- 始终规范化为 `BodyA < BodyB`；
- 同一Pair在一帧内只能出现一次；
- 身份必须包含 Entity 版本或使用独立稳定 BodyId，避免 Entity Index 复用造成错误续接；
- PairKey的排序和哈希必须确定性一致。

实验一可以先使用规范化的完整 `Entity` 身份。若后续进入统一框架，再引入连续 `BodyIndex` 与稳定BodyId映射。

### 5.2 FramePairSnapshot

每个已完成观察帧保存：

```text
FrameIndex
UnitCount
CandidatePairs
SoftInteractionPairs
ActiveContactPairs
DegeneratePairCount
Optional GeometricOverlapPairs in oracle mode
Optional NeighborPairs by skin
SpatialRegionPairs or active-cell summary
```

Pair快照应使用 Current/Previous 双缓冲，使统计系统能够比较相邻帧，同时避免覆盖仍在被Job读取的容器。

### 5.3 ContactLifetimeState

对活动接触Pair维护：

```text
PairKey
FirstActiveFrame
LastActiveFrame
ConsecutiveActiveFrames
TotalActiveFrames
```

实验一的“接触寿命”定义为连续活动帧数。Pair中断一帧后再次出现，应开始新的连续生命周期。带迟滞的接触生命周期可以作为后续实验，不能在基线统计中混用。

### 5.4 FrameStatistics

每帧至少保存：

```text
FrameIndex
UnitCount
CandidatePairCount
SoftPairCount
ActiveContactCount
DegeneratePairCount
PreviousRetainedCount
NewPairCount
EndedPairCount
RetentionRate
NewPairRate
JaccardSimilarity
AverageDegree
MaxDegree
ActiveVertexCount
PairGenerationTime
StatisticsTime
EstimatedPairStorageBytes
RegionRetentionRate
```

性能时间优先通过 Unity Profiler Marker 采集，不在 Burst Job 内使用不稳定的墙钟计时。

## 6. 指标定义

令本帧活动接触集合为 `A_t`，上一帧集合为 `A_(t-1)`。

### 6.1 Pair延续率

```text
RetentionRate = |A_t ∩ A_(t-1)| / |A_(t-1)|
```

含义：上一帧活动接触中，有多少延续到本帧。

### 6.2 新Pair比例

```text
NewPairRate = |A_t - A_(t-1)| / |A_t|
```

含义：本帧活动接触中，有多少是新出现的。

### 6.3 Jaccard相似度

```text
Jaccard = |A_t ∩ A_(t-1)| / |A_t ∪ A_(t-1)|
```

含义：整个活动碰撞图相邻帧之间的相似程度。

当公式分母为0时，结果必须记为 `N/A` 并单独计数，不能静默写成0或1。

上述三项必须分别对以下集合统计：

- CandidatePairs；
- ActiveContactPairs；
- 可选的NeighborPairs。
- SpatialRegionPairs或活跃Cell集合。

只统计Active Contact会把边界附近的频繁进入/退出误判为整个邻域关系不稳定。

### 6.4 接触寿命

至少输出以下区间：

```text
1 frame
2-4 frames
5-16 frames
17-64 frames
65+ frames
```

尚未结束的持续接触属于右删失数据，应同时记录 `OngoingContactCount`，不能把它们当成已经结束的短接触。

### 6.5 动态碰撞图指标

第一阶段的高ROI指标：

- 总顶点数：场景单位数；
- 活动顶点数：至少参与一个Active Contact的单位数；
- 活动边数：唯一Active Contact数量；
- 平均度：`2 * ActiveEdgeCount / TotalVertexCount`；
- 活动顶点平均度：`2 * ActiveEdgeCount / ActiveVertexCount`；
- 最大度；
- 度数分布或P50/P95/P99；
- Pair churn：新增、延续、结束Pair数量；
- Candidate/Active比值：宽相膨胀程度；
- Degenerate Pair数量。
- 活跃Cell数量、Cell Pair churn和区域级Jaccard；
- 每种缓存粒度的估算字节数及峰值容量。

最大连通分量、连通分量数量和图着色冲突数对后续并行求解很有价值，但实现成本更高，放在实验一第二阶段，不作为首个旁路版本的阻塞条件。

## 7. 系统与Job职责

### 7.1 ContactPairObservationOwner

负责：

- 创建和释放Persistent观察容器；
- 管理Current/Previous双缓冲；
- 管理容量增长；
- 暴露只写的本帧快照和只读的已完成快照；
- 记录快照版本和对应帧号；
- 串联生产、消费和清理JobHandle。

它是观察数据的生命周期Owner，不是通用内存分配器。

### 7.2 BuildObservedPairSnapshotJob

输入：

- 当前Spatial Hash；
- 当前帧位置；
- 当前帧预测位置；
- 当前硬接触半径、软避让半径和可选skin。

输出：

- 唯一CandidatePairs；
- SoftInteractionPairs；
- ActiveContactPairs；
- DegeneratePairs计数。

约束：

- 只观察，不写 `FlowMovementFrameState.PositionCorrection`；
- 使用与当前硬约束相同的预测位置输入；
- `A < B` 后再写入，避免双方重复Pair；
- 首个版本不改变现有 `CalculateFlowConstraintsJob`。

### 7.3 ContactPairStatisticsJob

输入：

- Current Pair集合；
- Previous Pair集合；
- 上一帧生命周期状态。

输出：

- 交集、新增、结束和并集计数；
- Retention/NewRate/Jaccard；
- 更新后的接触生命周期；
- 动态图度数统计；
- 帧级摘要。

### 7.4 ContactPairStatisticsSystem

负责：

- 调度统计Job；
- 将完成的帧摘要写入固定容量环形缓冲；
- 提供UI或测试工具读取；
- 可选地在一次测试结束后导出CSV；
- 不在每帧输出 `Debug.Log`。

### 7.5 GroundTruthValidationJob

仅在小规模验证模式启用：

- 对所有单位执行 `O(N^2)` 预测位置距离检查；
- 生成Geometric Ground Truth Pair集合；
- 验证旁路观察Job对“当前候选集合+当前窄相规则”的复现没有内部错误；
- 将当前实现可见的ActiveContactPairs与几何真值比较；
- 输出由当前宽相、退化法线规则或观察实现分别造成的False Negative、False Positive和重复Pair数量。

大规模性能测试必须禁用Oracle。

## 8. 旁路观察并非完全零侵入

目前预测位置和 `FlowMovementFrameState` 都是在 `BaseFlowMovementSystem` 内创建的 `TempJob`数据，独立统计系统无法在系统外可靠读取它们。

因此实验一至少需要在现有调度链中增加一个观察挂点：

```text
IntegrateFlowForcesJob
    -> BuildObservedPairSnapshotJob（只读预测位置）
    -> 原CalculateFlowConstraintsJob
```

或者让观察Job与原约束Job在积分完成后并行读取同一份预测快照，再正确合并依赖。

这属于数据观测接入，不属于运动行为修改。禁止采用以下替代方案：

- 在独立系统里重新计算一遍预测位置；
- 从最终Transform反推原始预测位置；
- 为了读取统计数据每帧调用 `Complete()`；
- 把临时预测位置无条件改成长期ECS组件。

这些方案会分别造成重复计算、输入失真、主线程同步或过度侵入。

## 9. 正确性验证

### 9.1 小规模Oracle

建议使用128或256单位、固定随机种子和固定时间步。每帧或每隔固定帧数运行 `O(N^2)` Oracle。

旁路观察模型自身的正确性要求：

```text
ObservedActivePairs == ActivePairsProducedFromCurrentCandidates
DuplicatePairCount == 0
```

随后将其与不经过当前Spatial Hash的几何真值比较：

```text
ObservedActivePairs vs GeometricOverlapPairs
```

此处允许发现基线False Negative；实验一必须准确记录它们，而不是为了让测试通过而改变运动逻辑。若漏对来自当前位置宽相与预测位置窄相不一致，它属于实验结论，不属于旁路观察系统失败。

对于后续Neighbor Cache，正确性条件为：

```text
GeometricOverlapPairs ⊆ CachedNeighborPairs
```

缓存允许产生额外候选，但不允许漏掉真实接触。

### 9.2 行为不变

实验一启用和禁用观察系统时，在相同输入和固定时间步下：

- 单位最终位置和速度应保持一致；
- 原有位置修正结果应保持一致；
- 不应新增结构变化或随机输入；
- 不应因为统计产生每帧主线程同步。

### 9.3 性能开销单独报告

旁路观察本身一定有成本。实验一只要求明确报告：

- Pair生成耗时；
- 统计耗时；
- 容器容量和峰值Pair数；
- 是否发生容量扩张；
- 是否产生同步点。

不能把观察模式的总耗时直接当成未来优化后求解器的耗时。

## 10. 当前理解与实现边界的错位

### 错位一：当前“没有碰撞对”

当前代码已经隐式执行成对碰撞检查，但没有显式、唯一、可复用的Pair数据模型。实验一补的是“碰撞对作为一等数据”，而不是第一次引入成对检测。

### 错位二：旁路系统可以完全独立

统计逻辑可以独立，但Pair生产不能完全脱离移动调度，因为真实预测位置是局部TempJob数据。必须增加最小观察挂点并共享同一输入快照。

### 错位三：Candidate Pair等于碰撞Pair

Spatial Hash输出的只是宽相候选。真正活动接触必须使用预测位置再次窄相判断。两者的规模、延续率和缓存价值必须分别统计。

### 错位四：上一帧Active Pair就是下一帧候选集

上一帧活动接触只能作为预测，不能覆盖新接触。后续优化更可能缓存带skin的Neighbor Pair，再对每帧几何关系重新窄相验证。

### 错位五：缓存自然会优化性能

显式写入、排序、去重和统计都有成本。只有在多轮求解、邻域相干性较高或多个消费者共享Pair时，缓存才可能胜过当前流式查询。实验一的任务正是测量这个前提。

### 错位六：沙漏场景足以证明普遍有效

沙漏场景偏向高密度和长接触，对缓存有利，也是很好的压力测试；但它不能证明开阔、对冲、高速情况下仍有效。项目结论必须报告适用条件和失效场景。

### 错位七：接触寿命是一个即时帧指标

完整生命周期只能在Pair结束时确定；仍在持续的Pair属于未结束样本。统计系统必须区分Completed Lifetime与Ongoing Lifetime。

### 错位八：完全重合Pair不属于接触

当前实现会跳过近零距离Pair，因为无法归一化法线。观察模型应将其记录为Degenerate Pair，而不是从接触统计中静默消失。

### 错位九：墙壁和单位Pair可以立即混合统计

单位—单位接触是动态图的边；单位—静态格子接触具有不同身份、生命周期和数量分布。实验一先研究单位—单位Pair，之后再扩展统一Pair类型。

### 错位十：RTS精确Pair缓存的结论可以直接迁移到布料

RTS当前研究的是圆盘单位和规则网格邻域；布料自碰撞至少涉及三角形拓扑、相邻Primitive过滤、BVH自相交遍历以及Vertex-Face/Edge-Edge窄相。两者能够共享最终动态约束表示，但Broad Phase、Pair粒度、时间相干层级和内存ROI必须分别实验。

## 11. 布料接入前的独立可行性门

布料不能在“还没有Broad Phase和三角形碰撞生成”的情况下直接被当作统一框架的第二个已完成后端。正式接入前需要完成以下可行性调查和最小原型。

### 11.1 几何与拓扑数据

- 稳定的顶点、边和三角形索引；
- 三角形邻接关系；
- 排除共享顶点、共享边及拓扑近邻的自碰撞Pair；
- 布料厚度、双面接触规则和固定点处理；
- 变形后Primitive AABB更新。

### 11.2 Broad Phase

- 初始BVH构建；
- 每帧叶节点AABB更新；
- BVH refit与完全rebuild的触发条件；
- 单布料BVH self traversal；
- 多布料BVH cross traversal；
- 顶层cloth-to-cloth AABB过滤；
- BVH节点重叠、叶候选和内存规模统计。

布料时间相干优化应优先比较“复用BVH拓扑并refit”和“每帧完全rebuild”，而不是先实现持久Primitive Pair Cache。

### 11.3 Narrow Phase

- Vertex-Face接触；
- Edge-Edge接触；
- 法线和最近点退化处理；
- 离散碰撞检测与连续碰撞检测（CCD）的范围选择；
- 重复接触和同一区域多个Primitive Pair的合并策略。

第一版可以明确限定为离散碰撞检测，但必须记录高速运动可能穿透，不得把它描述为完整布料自碰撞。

### 11.4 接触区域与流形假设

刚体中的持久Contact Manifold不能无条件照搬到变形布料。布料三角形会形变，具体Vertex-Face/Edge-Edge身份可能变化。需要分别测量：

- BVH重叠节点生命周期；
- 叶级三角形候选生命周期；
- 精确Primitive Contact生命周期；
- 空间接触区域或接触簇生命周期；
- 每个层级缓存的命中率、维护成本和内存。

只有数据证明具体Primitive Pair具有足够生命周期时，才继续实现其跨帧缓存或lambda Warm Start。否则优化重点应停留在BVH refit、区域级候选和接触簇层。

### 11.5 布料可行性门的通过条件

进入统一求解器集成前，至少满足：

1. BVH候选生成相对小规模 `O(T^2)` 三角形Oracle无漏对；
2. 拓扑相邻过滤规则有独立测试；
3. Vertex-Face与Edge-Edge最小窄相用例通过；
4. 能把布料接触转换成与RTS动态接触兼容的约束输入；
5. 已测量BVH refit、rebuild、候选Pair和精确接触的时间与内存；
6. 已决定时间相干缓存放在BVH、区域、Primitive Pair还是求解状态层，并有数据依据。

## 12. 实验一实施顺序

### Commit 1：观察数据模型

- PairKey和帧级Pair快照；
- Persistent双缓冲Owner；
- 规范化Pair身份；
- 生命周期和释放规则；
- 暂不接入运动调度。

### Commit 2：旁路Pair生产

- 在预测位置生成后调度观察Job；
- 输出Candidate、Soft、Active和Degenerate集合；
- 保证不写运动状态；
- 小规模下验证唯一性。

### Commit 3：统计系统

- 相邻帧集合运算；
- Pair延续率、新Pair比例、Jaccard；
- 接触寿命；
- 度数和Pair churn；
- 环形摘要缓冲，不逐帧打印日志。

### Commit 4：Oracle与实验输出

- 小规模 `O(N^2)` Oracle；
- False Negative/Positive检查；
- 沙漏场景固定配置；
- UI或CSV实验结果；
- Profiler Marker和观察开销报告。

如果希望严格保持“小步验证”，Commit 1和Commit 2之间可以增加纯C#数学测试，验证Pair规范化、集合交并和空集合指标规则。

## 13. 实验一成功判据

实验一完成必须同时满足：

1. 显式生成唯一Candidate Pair和Active Contact Pair；
2. 小规模Oracle下无重复Pair，且能区分观察实现错误与当前宽相造成的几何漏对；
3. 开关观察系统不改变单位运动结果；
4. 没有新增每帧 `Dependency.Complete()` 主线程同步；
5. 能记录活动接触和邻域候选各自的延续率、新Pair比例和Jaccard；
6. 能输出已结束与仍持续的接触寿命；
7. 能记录Candidate/Active比值、平均度、最大度和Pair churn；
8. 沙漏场景能够形成一组可重复的基线数据；
9. 明确记录观察模型自身的时间和内存开销；
10. 数据能够支持“下一步做Neighbor Cache、Contact Cache或保持每帧重建”的决策。
11. 能比较Spatial Region、Neighbor Pair和Active Pair三个粒度的时间相干性与存储成本。

## 14. 暂不进入实验一的内容

- 不让新Pair快照替换当前 `CalculateFlowConstraintsJob`；
- 不实现多轮Jacobi或Gauss-Seidel；
- 不实现XPBD compliance和lambda；
- 不实现跨帧Warm Start；
- 不实现Verlet Neighbor Cache；
- 不改变当前软避让力；
- 不修正位置投影后的速度；
- 不引入通用形状碰撞、GJK/EPA、CCD或接触流形；
- 不把系统宣称为通用物理引擎。

这些内容都依赖实验一先证明动态碰撞图的规模、变化率和相干性。
