# Simulation Debugger 使用与数据说明

## 打开方式

- Editor 或 Development Build 会自动创建 `Simulation Debugger` GameObject；也可以手动挂载：
  - `SimulationDebuggerPanel`
  - `SimulationDebuggerWorldOverlay`
- 默认快捷键：`F8`。
- 面板关闭后 `CaptureMask=None`，不会继续触发诊断快照同步。
- 世界热力图与线框使用面板右上角 `Overlay` 开关。

如果场景中已存在 `Stage3ContactDiagnosticSelection`，继续使用项目原有的单位点击/选择流程即可。选中的 `Entity` 会成为下一 timestep 的详细捕获目标。诊断系统跨帧保存的是 `Entity`，不会保存不稳定的临时 `BodyIndex`。

## 四个视图

### 整体

默认只显示：

- **求解耗时**：软避让、Pair/Contact 生成和 XPBD 求解的总成本；
- **单位数量**：当前参与这套 Flow Movement 的单位；
- **最大接触修正**：单次最强位置修正，用于发现严重穿透或局部不收敛。

展开详细数据后才显示阶段耗时、Broad 候选、Contact Set 和墙体修正。

### 跨帧 AABB

默认只显示：

- **缓存复用率**：Reuse / (Reuse + Rebuild)；
- **候选膨胀**：缓存候选 Pair / 最终 Contact Pair；
- **重建 / 回退**：缓存重新生成次数和退回完整 Broad Phase 的次数。

这个页面回答的是“跨 timestep 的 Broad Phase 缓存值不值得”，而不是 Contact 是否预测准确。

### 跨子步 Contact

默认只显示：

- **Contact Set**：当前 timestep 跨 substep 复用的约束拓扑数量；
- **接触激活率**：缓存 Contact 中至少真正激活过的比例；
- **补充 / 回退**：初始 Contact Set 没覆盖正常路径时发生的恢复次数。

Predictive Contact 只是 Contact Set 的一个组成部分。展开后显示 Actual/Near、Predictive、未激活和预计避免的重复 Contact Generation。

### 设置

所有修改先进入 Draft；点击 `应用 Override` 后，在下一 timestep 开始、任何 Job 调度之前统一写入。页面同时显示 Draft 与 Solver 当前 Effective Value。

- `读取 Effective`：丢弃未应用修改，重新读取当前有效值；
- `恢复 Authoring`：恢复播放开始后首次捕获的 Authoring baseline；
- Adaptive 设置只有场景存在相应 singleton 时才写回。

## 热力图

三个诊断页面共享同一份空间网格快照，但选择不同字段着色。热力图不依赖 Adaptive Fat AABB 必须开启；Adaptive 关闭时仍会构建只读诊断网格，不进行热点路由。

### 整体热力图

- **综合压力**：单位密度、历史修正和局部 Contact 负载的综合值；
- **密度**：该诊断格子的单位密度；
- **修正量**：单位 Contact/Wall 修正压力。

### AABB 热力图

- **缓存收益**：复用效果减去重建、Fallback 和局部惩罚后的归一化结果；
- **剩余余量**：本 timestep Swept Bounds 距离 Fat Bounds 边界的最小余量；
- **候选膨胀**：无效 Broad Candidate 负担的空间近似；
- **逃逸风险**：速度、修正、边界余量和历史失效的综合风险。

### Contact Set 热力图

- **激活**：该区域 Contact Set 实际被激活的比例；
- **未使用**：Contact 负载乘以未激活比例；
- **漏检风险**：局部 Contact 负载、运动逃逸和完整 Broad Phase 恢复的综合风险。

所有空间值使用历史平滑结果，目的是保持颜色稳定。面板与世界 Overlay 始终读取同一 `FrameId`。

## 选中单位 Overlay

点击单位后会显示：

- 蓝色轨迹：timestep 起点到无约束预测位置；
- 橙色轨迹：无约束位置到最终求解位置；
- 青色框：本 timestep Swept Bounds；
- 紫色框：跨帧 Fat Bounds；
- 蓝色小格：该 Swept Bounds 覆盖的诊断/Broad 网格；
- 灰/黄/橙/红/蓝 Pair 线：缓存未激活、Predictive、Predictive Active、Actual Active、Supplement/Fallback。

默认最多绘制 32 条与选中单位相关的 Pair，可在设置页调整。详细模式最多在面板内列出前 12 条。

## 采样开销

- `Summary interval` 控制纯汇总快照频率；
- `Spatial interval` 控制热力图和选中单位空间快照频率；
- 三个诊断页面只请求自己需要的 Capture Mask；
- 设置页只请求 Summary；
- 整个面板关闭时不采样。

当前快照发布会在请求采样的帧执行一次 `Dependency.Complete()` 以保证 GUI、热力图和 Pair 数据属于同一帧。它是明确的调试成本，不应在 Release Build 中默认开启。

## 迁移说明

旧的 `AdaptiveFatAabbSettings.DrawDebug` 仍然保留用于兼容，但建议关闭，避免与新的 `SimulationDebuggerWorldOverlay` 重复绘制。
