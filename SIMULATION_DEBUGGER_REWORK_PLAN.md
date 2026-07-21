# Simulation Debugger 重构行动方案

## 目标

把当前分散的接触统计、Fat AABB 调试线、热力图和运行时参数调整整合成一套统一诊断框架。
诊断界面默认只回答三个问题：

1. 整体仿真是否正常；
2. 跨 timestep 的 Broad Phase 缓存是否真正有净收益；
3. timestep Contact Set 在跨 substep 复用时是否完整且有效。

详细计数只在展开后显示，不再默认平铺全部底层字段。

## 最终四个页面

### 1. Simulation Overview / 整体仿真

默认核心值：

- Solver Cost；
- Unit Count；
- Max Contact Correction。

展开后显示软避让、Pair Generation、XPBD Iteration 等阶段成本和工作量。

### 2. Persistent Broad Phase / 跨帧 AABB

默认核心值：

- Cache Reuse Ratio；
- Candidate Expansion；
- Rebuild / Fallback。

展开后显示 Cache Age、Proxy/Pair 数量、Invalidation 原因和估算净收益。

### 3. Timestep Contact Set / 跨子步接触缓存

默认核心值：

- Contact Set Size；
- Activation Ratio；
- Supplement / Fallback Count。

Predictive Contact 只是 Contact Set 的一种来源；组成、首次激活 substep、未激活 Pair 等放在详细信息中。

### 4. Runtime Settings / 运行时设置

按 Global、Soft Avoidance、AABB、Contact Set、XPBD、Diagnostics 分组。
每个可调整项显示 Authoring 值、Runtime Override 和 Effective 值，并只在 timestep 边界应用。

## 数据流

```text
Authoring Defaults
        ↓
Runtime Override Request
        ↓
Effective Config Snapshot
        ↓
Simulation Solver
        ↓
Diagnostics Capture
        ↓
Immutable Frame Snapshot
        ├─ Four-panel GUI
        ├─ Heatmap Renderer
        └─ Selected Unit Overlay
```

GUI、热力图和世界空间线框只能读取同一份带 FrameId 的不可变快照，不能分别读取正在写入的 NativeContainer。

## 提交拆分

1. `docs/diagnostics contracts`
   - 行动方案；
   - 统一快照与 Capture Mask；
   - 运行时只读桥接。

2. `capture unified diagnostics snapshot`
   - 从现有 Contact/Fat AABB 统计构造统一快照；
   - 发布空间 Cell、Region、Proxy 和选中单位数据；
   - 关闭页面时停止对应采样。

3. `add four-panel runtime debugger`
   - 四页面统一风格；
   - 默认核心值 + 折叠详情；
   - 关闭、冻结、选择主热力图。

4. `add heatmaps and selected-unit drilldown`
   - 三页面独立热力图模式；
   - 选中单位的 Swept/Fat Bounds、Broad Cells 和 Pair 连接线；
   - 图例、固定量纲和绘制数量上限。

5. `add runtime overrides and diagnostics safeguards`
   - 参数覆盖请求在 timestep 边界生效；
   - 配置依赖和恢复默认；
   - 诊断开销等级、采样周期和文档。

## 运行时安全约束

- 面板关闭时 `CaptureMask=None`，不产生诊断同步点；
- Summary 与 Spatial 使用独立采样周期；
- 热力图、GUI 和 Pair Overlay 使用同一 `FrameId`；
- Pair 详细数据只记录选中单位，并受 `MaximumVisualizedPairs` 限制；
- Runtime Override 在 `BaseFlowMovementSystem.OnUpdate` 开始、任何 Job 调度前统一写入；
- Restore Authoring 使用第一次运行时捕获的 baseline，不在播放模式中永久改写 Authoring 资产。
