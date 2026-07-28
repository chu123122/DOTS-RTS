# Simulation Debugger 使用与数据说明

## 打开方式

- Editor 或 Development Build 会自动创建 `Simulation Debugger` GameObject；也可以手动挂载：
  - `SimulationDebuggerPanel`
  - `SimulationDebuggerWorldOverlay`
- 默认快捷键：`F8`。
- 面板关闭后 `CaptureMask=None`，不会继续触发诊断快照同步。
- 世界热力图与线框使用面板右上角 `Overlay` 开关。

如果场景中已存在 `ContactDiagnosticSelection`，继续使用项目原有的单位点击/选择流程即可。选中的 `Entity` 会成为下一 timestep 的详细捕获目标。诊断系统跨帧保存的是 `Entity`，不会保存不稳定的临时 `BodyIndex`。

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

## 多窗口与网格热力图

- F8 控制整套诊断界面的显示；顶部“仿真诊断”启动条可分别打开或关闭四个窗口。
- 整体仿真、跨帧 AABB、跨子步接触缓存、运行时设置四个窗口可以同时存在。
- 每个窗口可以拖动，右下角“◢”可以调整大小；布局字段也会暴露在 `SimulationDebuggerPanel` Inspector 中。
- 三个诊断窗口各自保存自己的热力图类型，并在面板内显示二维诊断格子地图。
- “映射到游戏地图”会把当前窗口选择的热力图设为世界空间主热力图；场景中使用填充格子和网格边线显示。
- 白色描边格子表示当前选中单位所在的诊断格子。

## 对比实验 A / B / C

运行时设置窗口顶部提供三项互相独立的实验变量：

| 编号 | 开关或模式 | 关闭 / 0 | 开启 / 1 |
|---|---|---|---|
| A | 跨帧 AABB 候选缓存 | 每时间步使用普通 Broad Phase | 验证并复用跨帧 Fat AABB 候选 |
| B | 跨子步接触集缓存 | 每个子步重新生成接触集 | 每时间步生成一次并跨全部子步复用 |
| C | 软避让求解器 | 预测引导 | RVO 互惠避让 |

三个变量共有八种完整组合。界面使用 `A0-B1-C0` 形式显示当前组合，并分配递增的配置编号。

配置切换发生在下一时间步开始前。诊断快照先发布上一时间步的结果，再应用下一组配置，因此面板中的指标、有效配置和实验编号属于同一批求解结果。

### 跨子步接触集的两条路径

缓存开启：

```text
时间步开始
  -> 按完整时间步运动范围生成 Contact Set
  -> 保存预测接触的参考方向
  -> 所有子步复用同一 Contact Set
  -> 每子步只重置 Lambda 并重新计算当前约束违反量
```

缓存关闭：

```text
每个子步
  -> 根据该子步的起点和预测终点重新生成 Contact Set
  -> 当前子步的全部 XPBD 迭代复用
```

跨子步缓存拥有独立的 timestep 运动包络，并不依赖 Fat AABB：

- 生成整步 Contact Set 时，为每个单位保存本时间步 swept envelope；
- 每个子步积分后检查单位是否离开该 envelope；
- 墙体或接触修正后，只检查本轮被修正的单位；
- 一旦逃逸，本时间步剩余部分安全降级为每子步重新生成；
- `运动包络失效` 和 `本帧生成次数` 会显示在跨子步接触窗口的详细信息中。

### 预热阶段

切换配置后，缓存、速度状态和场景拥堵需要重新稳定。默认前 45 帧标记为预热，不建议纳入正式统计。预热帧数可以在“诊断与显示”中调整。

建议先做三组单变量实验，再跑完整八组合：

1. 固定 B、C，只切换 A；
2. 固定 A、C，只切换 B；
3. 固定 A、B，只切换 C；
4. 最后进行完整 `2 × 2 × 2` 析因实验。

## 四窗口与单位选择

- 四个诊断窗口可以同时显示；拖动标题栏移动，拖动右下角 `◢` 调整大小。
- 顶部启动条的“重置布局”会恢复 2×2 四窗口布局，并重新打开全部窗口。
- 鼠标中键短按单位可选择详细诊断目标；中键拖动超过阈值时不会触发选择，仍可供相机平移使用。
- 选择采用屏幕空间最近单位，因此不依赖地面高度或碰撞射线。详细数据在下一时间步快照中出现。
