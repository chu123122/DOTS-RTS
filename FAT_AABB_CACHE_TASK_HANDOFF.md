# RTS 单位接触与 Fat AABB 缓存任务交接

更新时间：2026-07-20
工作区：`E:\unity\3d\RTS\Assets\Scripts`
当前分支：`Simulation`
当前 HEAD：`cfdec59 feat: add runtime predictive pair generation toggle`

## 1. 当前任务目标

在现有 Predictive Disc Contacts / XPBD 单位圆盘接触求解器上，把原来的 Shadow Fat AABB 旁路实验接成真正参与权威求解的跨子步、跨帧 Broad Phase 缓存。

本阶段只优化单位圆盘接触候选发现，不引入：

- 跨帧 XPBD lambda Warm Start；
- 跨帧接触模式缓存；
- 布料 Point-Triangle 接触；
- 通用多形状碰撞框架。

成功边界是：缓存只跳过重复的 Broad Phase 候选发现，当前子步仍使用最新位置重新执行 Narrow Phase，并在缓存不再安全时自动回退完整 Broad Phase。

## 2. 已有提交基线

关键历史提交按时间顺序为：

| Commit | 内容 |
|---|---|
| `88d6b84` | 生成唯一单位碰撞 Pair |
| `ed66739` | Stage 2：单位接触升级为 XPBD |
| `2e65460` | Stage 3：Predictive Disc Contacts |
| `6b5810c` | 预测碰撞可视化 |
| `1aa02a0` | 墙壁与单位约束统一进入迭代 |
| `8280edd` | Shadow Fat AABB 邻居缓存旁路测试 |
| `af118c0` | 修复诊断数据读取同步 |
| `2041dc2` | 放大诊断面板 |
| `cd8b7ca` | 中文双面板诊断 UI |
| `7b439dd` | 修正残差诊断、软避让按子步计算 |
| `cfdec59` | 运行时开启/关闭 Predictive Pair 生成 |

当前 Fat AABB 权威缓存实现位于 `cfdec59` 之上的**未提交工作区**，还没有独立 commit。

## 3. 当前单帧求解流程

`SolveXpbdUnitContactsJob.Execute()` 当前每帧执行：

1. 初始化所有单位的本帧求解状态。
2. 对每个 substep：
   1. 按当前已求解位置重新计算一次软避让；
   2. 积分得到本 substep 的 `StartPosition` 和 unconstrained `PredictedPosition`；
   3. 获取候选 Pair：
      - 缓存关闭：执行完整 swept Spatial Hash Broad Phase；
      - 缓存有效：把缓存的 `Entity Pair` 映射为当前帧 `BodyIndex Pair`；
      - 缓存无效：从当前 swept path 重建 Fat AABB 邻居表；
   4. 对候选 Pair 使用当前 substep 的起点和预测终点重新执行 Narrow Phase；
   5. 全部 XPBD iterations 复用本 substep 分类后的 Pair；
   6. 每轮依次求解墙壁约束和单位接触约束；
   7. 根据最终位置回写速度。

注意：当前“Predictive Pair”只在同一 substep 的全部 XPBD iterations 内复用。Fat AABB 缓存解决的是候选 Entity Pair 的跨 substep / 跨 frame 时间相干性，两者不是同一层缓存。

## 4. Fat AABB 缓存的数据边界

持久数据：

- 每个单位的 `Entity`、Fat AABB 和有效标记；
- 由两个稳定 `Entity` 组成的候选 Pair；
- 缓存是否有效、年龄、构建时使用的 `PredictiveSkin` 和 `Margin`。

明确不持久保存：

- `BodyIndex`：每帧重新建立 `Entity -> BodyIndex` 映射；
- `UnitContactMode`：每个 substep 重新做 Narrow Phase 后决定；
- `Lambda`：仍按原设计在每个 substep 清零；
- `WasActivated` 等当次求解状态。

因此缓存 Pair 只回答“这两个单位是否值得进入 Narrow Phase”，不回答“它们现在是否接触、使用径向约束还是防换侧约束”。

## 5. 缓存有效性与安全回退

每个 substep 在使用缓存前检查：

- 单位 Entity 集合和 `IsInsideGrid` 状态是否一致；
- `PredictiveSkin` / `FatAabbCacheMargin` 是否变化；
- 当前 `StartPosition -> UnconstrainedPredictedPosition` swept disc 范围是否仍完全包含在旧 Fat AABB 内。

XPBD iteration 内还会在墙壁修正和单位接触修正之后检查最终圆盘是否逃出 Fat AABB。

一旦失效：

1. 标记持久缓存无效；
2. 当前 substep 立即执行完整 swept Broad Phase；
3. 后续 iteration 使用新发现的 Pair；
4. 下一 substep 根据最新状态重建 Fat AABB 缓存。

如果逃逸发生在最后一次 iteration 后，会额外执行一轮单位接触恢复求解，避免新 Pair 已发现但没有求解机会。

## 6. 运行时控制与 Inspector 参数

`FlowFieldManagerAuthoring` 当前默认值：

| 参数 | 默认值 | 含义 |
|---|---:|---|
| `contactSubsteps` | `2` | 每帧接触子步数 |
| `contactIterations` | `4` | 每个子步 XPBD 迭代数 |
| `predictiveContactSkin` | `0.05` | Predictive/Narrow Phase 提前范围 |
| `enablePredictivePairGeneration` | `true` | 是否生成 swept Predictive Pair |
| `enablePredictiveContacts` | `true` | 是否启用防换侧约束 |
| `enableFatAabbCache` | `false` | 是否启用权威 Fat AABB Broad Phase 缓存 |
| `fatAabbCacheMargin` | `0.25` | Fat AABB 在圆盘半径和 skin 之外的额外余量 |
| `diagnosticCaptureDuration` | `10s` | 单次 JSON 采集时长 |
| `diagnosticCaptureInterval` | `0.1s` | JSON 采样间隔 |

Game 运行时快捷键：

| 快捷键 | 功能 |
|---|---|
| `F6` | 开始/提前结束 JSON 诊断采集 |
| `F7` | 开关 Predictive Pair 生成 |
| `F8` | 开关 Stage 3 诊断数据 |
| `F9` | 开关防换侧 Predictive 约束 |
| `F10` | 开关选中单位场景线框 |
| `F11` | 开关 Fat AABB 权威缓存 |
| `PageUp / PageDown` | 调整诊断面板缩放 |
| 鼠标中键 | 选择一个单位查看具体 Pair 和 swept 范围 |

关闭 Fat AABB 缓存时会清空持久代理、Pair 和状态；重新开启后第一次使用必然完整重建，不会复用关闭前的陈旧数据。

## 7. 诊断面板与 JSON

右侧 Fat AABB 面板重点字段：

- `帧首/帧末有效性`：本帧开始和结束时缓存是否有效；
- `年龄`：连续没有重建的帧数；
- `使用/复用/重建`：本帧各 substep 如何取得候选 Pair；
- `完整回退`：缓存失效后执行完整 Broad Phase 的次数；
- `Entity集合/边界/求解后逃逸`：缓存失效原因；
- `候选 Pair / Narrow 检查`：缓存带来的候选规模；
- `构建/验证/映射耗时`：缓存路径各阶段开销。

JSON 格式已升级为 `Stage3ContactDiagnostic/v2`，增加 Fat AABB 的有效性、年龄、复用、重建、失效、回退、候选数量和耗时字段。

输出目录：

`C:\Users\chu\AppData\LocalLow\DefaultCompany\RTS\Stage3ContactDiagnostics`

文件名带毫秒时间戳，不会覆盖上一份：

`stage3-contact-yyyyMMdd-HHmmss-fff.json`

## 8. 已完成验证

Runtime 和 Editor 程序集均已通过 Unity Roslyn 编译。

Unity 菜单验证：

`RTS/Validation/Predictive Disc Contacts Stage 3`

快捷键：`Ctrl + Shift + F12`

最近一次完整结果：

```text
STAGE3_VALIDATION_OK
crossing: predictive=1, activated=1
predictive disabled: potential=1, active predictive=0
generation disabled: contacts=0, predicted=0
tangent: contacts=1, unactivated=1
chain: active=2
soft avoidance: evaluations=4
wall->unit: B.x 2.435000 -> 2.491323
fat cache: reuse=2, fallback=1, post-solve invalidation=1
iterations 1->8: maxPenetration 0.262500 -> 0.002930
```

Fat AABB 自动测试覆盖：

1. 稳定密集接触连续两帧复用缓存；
2. 缓存开关前后最终位置与完整 Broad Phase 基线一致；
3. 小 margin 下墙壁把单位推出 Fat AABB，能够失效并安全回退；
4. 关闭缓存会清空持久状态；
5. 重新开启时会重建而不是使用旧缓存。

## 9. 当前未提交文件边界

与单位接触、诊断和 Fat AABB 任务有关的工作区文件：

- `Entities/Unit/System/FlowFieldSystem/BaseFlowMovementSystem.cs`
- `Entities/Unit/System/FlowFieldSystem/Editor/PredictiveDiscContactStage3Validation.cs`
- `Entities/Unit/System/FlowFieldSystem/FlowFieldManagerAuthoring.cs`
- `Entities/Unit/System/FlowFieldSystem/GridComponent.cs`
- `Entities/Unit/System/FlowFieldSystem/MoveJob/SolveXpbdUnitContactsJob.cs`
- `Entities/Unit/System/FlowFieldSystem/ShadowNeighborCacheTypes.cs`
- `Entities/Unit/System/FlowFieldSystem/Stage3ContactDiagnosticComponents.cs`
- `Entities/Unit/System/FlowFieldSystem/Stage3ContactDiagnosticVisualizationSystem.cs`
- `Entities/Unit/System/FlowFieldSystem/Stage3ContactDiagnosticCapture.cs`
- `Entities/Unit/System/FlowFieldSystem/Stage3ContactDiagnosticCapture.cs.meta`

当前工作区还存在不要混入本任务提交的改动：

- `Entities/Building/System.meta`：删除状态；
- `_QFrameWork/UISystem/CameraController.cs`：相机缩放改动；
- `Entities/Unit/System/FlowFieldSystem/MoveJob/CalculateFlowConstraintsJob.cs`：Git 显示修改，但当前没有文本 diff，疑似行尾变化。

提交前必须逐文件暂存，不要直接执行 `git add .`。

## 10. 已知风险与观察点

1. **验证仍是 O(n)**
   Fat AABB 路径每个 substep 仍需线性验证所有单位是否留在缓存边界中。它跳过的是 Spatial Hash 展开、排序和候选组合，不是让整个 Broad Phase 变成 O(1)。

2. **Margin 存在成本平衡**
   margin 太小会频繁重建/回退；太大会显著膨胀候选 Pair，使 Narrow Phase 检查数量上升。

3. **回退会重建当前 Pair 列表**
   中途逃逸回退时，本 substep 当前 Pair 的 lambda 会随 Pair 重建而清零。正确性优先，但可能改变该 substep 的收敛速度，后续需要用真实密集场景观察回退频率。

4. **回退帧统计可能重复累计**
   当前统计是“本帧执行过的工作量”。如果先走缓存 Narrow Phase、随后回退完整 Broad Phase，`CandidatePairCount` / `ContactPairCount` 可能同时包含回退前后的两次分类，不应把它直接解释为最终唯一 Pair 数。

5. **内部仍保留 Shadow 命名**
   `ShadowFatBodyProxy`、`ShadowNeighborCacheStatistics` 等名字来自先前旁路实验。当前逻辑已参与权威求解，但本阶段没有为改名扩大 diff；不影响功能。

## 11. 下一步实验建议

先不要加入 Warm Start 或其他缓存层，使用同一密集停止场景完成 OFF/ON 对照：

1. 固定单位数、目标、substeps、iterations、skin 和 soft avoidance 参数；
2. `F11` 关闭缓存，稳定后按 `F6` 采集一份 10 秒 JSON；
3. `F11` 开启缓存，等待一次重建后再按 `F6` 采集一份 10 秒 JSON；
4. 至少重复 `OFF -> ON -> OFF` 三轮，避免单次时间波动误判；
5. 对比：
   - `PairGenerationMicroseconds`；
   - `SolverMicroseconds`；
   - `FatAabbCacheReuses / Rebuilds / FullBroadPhaseFallbacks`；
   - `FatAabbCachedCandidatePairs / ContactPairs` 膨胀比例；
   - `MaxPenetration`、最终残差和 `MaxVelocityChange` 是否保持同级；
6. 只有在“复用率高、回退率低、Pair 膨胀可控且最终物理结果不退化”时，才进入 margin 扫描或更复杂的增量更新实验。

建议第一轮固定：

```text
Substeps = 2
Iterations = 8 或 10
Predictive Pair Generation = ON
Predictive Contacts = ON
Fat AABB Margin = 0.25
```

## 12. 推荐收尾方式

在真实场景完成一轮 OFF/ON 数据验证后：

1. 检查 `git diff --check`；
2. 再运行一次 `Ctrl + Shift + F12`；
3. 仅暂存第 9 节列出的相关文件和本交接文档；
4. 单独提交，建议 commit：

```text
feat: integrate fat AABB contact cache
```
