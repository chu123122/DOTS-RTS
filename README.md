# High-Performance RTS Tech Stack based on Unity DOTS

# 基于 Unity DOTS 的高性能 RTS 同步架构

![Unity](https://img.shields.io/badge/Unity-2022.3%2B-black)
![DOTS](https://img.shields.io/badge/Tech-ECS%20|%20Jobs%20|%20Burst-blue)
![NetCode](https://img.shields.io/badge/Network-NetCode%20for%20Entities-green)
![License](https://img.shields.io/badge/License-MIT-yellow)

> 200+ 高密度单位同屏物理模拟 | < 1ms 逻辑耗时 | 基于指令的确定性回放 | 增量接触管线

---

## 项目简介

基于 Unity DOTS (ECS + Jobs + Burst) 的高性能 RTS 游戏核心架构。解决大规模单位在多人联机环境下的**寻路性能**、**物理避障稳定性**以及**确定性同步**问题。完全 ECS 架构，Burst Compiler 极致 CPU 优化。

---

## 目录结构

```
Scripts/
├── FpsDisplay.cs                        # 实时帧数显示
│
├── Entities/
│   ├── Unit/
│   │   ├── Authoring/FlowField/         # ECS 组件 Authoring
│   │   ├── Components/                  # ECS 组件定义
│   │   │   └── FlowField/               # 流场、接触、缓存组件
│   │   └── Systems/FlowField/
│   │       ├── BaseFlowMovementSystem.cs # 分阶段调度基类（OnUpdate）
│   │       ├── FlowFieldBakeSystem.cs    # Cost Field + Vector Field 烘焙
│   │       ├── RtsCommandSystem.cs       # RTS 指令处理
│   │       ├── AdaptiveFatAabbSettings.cs # 自适应热点参数
│   │       ├── Jobs/
│   │       │   ├── ContactPipeline/      # ★ 增量接触管线（模块化）
│   │       │   │   ├── Core/             # 编排 + 类型 + 数学工具
│   │       │   │   ├── BroadPhase/       # 帧内 Swept Disc 候选生成
│   │       │   │   ├── Persistent/        # 跨帧持久拓扑、分类、并行 P1-P6
│   │       │   │   ├── Prediction/       # Timestep envelope + 视图构建
│   │       │   │   ├── Motion/           # 子步速度、位置积分、重建
│   │       │   │   ├── SoftAvoidance/    # 紧凑软避让视图
│   │       │   │   └── Solver/           # XPBD (Gauss-Seidel / 并行 Jacobi)
│   │       │   ├── CalculateIndependentFlowForceJob.cs
│   │       │   ├── ApplyFlowMovementJob.cs
│   │       │   └── [Legacy] AdaptiveFatAabb*.cs, FatAabbCache*.cs
│   │       ├── Diagnostics/              # 诊断面板、调优器、Benchmark
│   │       │   ├── SimulationDebuggerPanel.cs  # IMGUI 多窗口面板
│   │       │   ├── SimulationDebuggerRuntime.cs
│   │       │   ├── AdaptiveParameterTuner.cs   # 自动参数搜索
│   │       │   ├── IncrementalContactPipelineBenchmarkWindow.cs
│   │       │   └── IncrementalContactPipelineCsvRecorder.cs
│   │       └── Editor/                   # 验证测试
│   │           ├── PredictiveDiscContactStage3Validation.cs
│   │           └── LocalGameplayModeValidation.cs
│   ├── Camera/
│   │   └── InitializeMainCameraSystem.cs
│   └── _RePlay/                          # 事件溯源回放系统
│       └── NewReplay/
│           ├── CommandRecordingSystem.cs
│           └── CommandReplayingSystem.cs
│
├── _QFrameWork/
│   ├── UISystem/
│   │   ├── CameraController.cs           # RTS 摄像机控制
│   │   ├── RTSSelectionManager.cs        # 框选
│   │   └── BasicBuildUIController.cs     # 测试单位生成按钮
│   └── BuildingManagement/
│
├── _PlayerInput/UnitControl/
│   └── UnitMoveInputSystem.cs            # 鼠标移动指令
│
└── NetWorkInitialize/
    ├── Client/ClientConnectManager.cs    # 本地模式入口
    └── Common/                           # RPC 相关类型
```

---

## 核心特性

### 1. 海量单位流场寻路 (Flow Field)
- 基于 Eikonal 方程的向量场寻路，替代传统 A*
- 通过 Cost Field → Integration Field 实现 O(1) 寻路查询
- 寻路逻辑完全并行化 (Job System)

### 2. XPBD 接触求解 (Contact Pipeline)
```
Persistent Proxy → Persistent Neighbor Topology → Classification → Views → Solvers
```

**六层架构**：

| 层 | 职责 | 生命周期 |
|----|------|---------|
| **BroadPhase** | Swept Disc 帧内候选对生成 | 帧内，无跨帧状态 |
| **Persistent** | 跨帧实体对拓扑、分类版本、稳定法线、键 | 跨 timestep |
| **Prediction** | Timestep envelope、视图构建、安全校验 | Timestep 或重建窗口 |
| **SoftAvoidance** | 消耗 Soft 紧凑视图 | 子步 |
| **Solver** | XPBD (Gauss–Seidel / 并行 Jacobi) + 墙壁约束 | 子步迭代 |
| **Motion** | 速度准备、位置预测、速度重建 | 子步 |

**正确性保障**：
1. 所有可能接触都在当前 Interaction 视图内或被 dirty body 覆盖
2. Soft/RVO 输出不能逃出 Interaction envelope（强制钳位）
3. 增量证明失败时统一回退到全量 Sweep
4. Jacobi 并行对评估读取不可变位置快照，确定性集合并行修正在 incident-pair 顺序中完成

### 3. 并行 Jacobi 求解器 (P1-P6)
- 6 阶段并行接触投影管道，含 CSR 格式主动约束索引
- 阶段 1-3：并行对评估 + 局部修正累积
- 阶段 4-6：确定性 body gather + 位置应用 + 墙壁投影
- Gauss–Seidel 保留为串行参考实现

### 4. 事件溯源回放系统
- 零快照：不记录每帧 Transform，仅记录输入指令 Buffer
- 毫秒级状态回滚与指令重演

---

## 已知技术债

- 未实现 Contact-island 休眠，持续活跃接触始终求解
- 并行 Gauss-Seidel 需要图着色或冲突无关批次（目前仅 Jacobi 有并行路径）
- 确定性列表压缩、排序、持久视图发布仍为串行协调点

---

## 诊断与调优

| 工具 | 用途 |
|------|------|
| `SimulationDebuggerPanel` | F8 开关，四窗口（整体/跨帧 AABB/跨子步 Contact/设置） |
| `AdaptiveParameterTuner` | 自动参数搜索，挂场景中跑 → 输出 CSV |
| `IncrementalContactPipelineBenchmarkWindow` | 增量管线指标实时对比 |
| `FpsDisplay` | 屏幕正上方实时帧数 |

## 验证测试

Editor 菜单 `RTS/Validation/Predictive Disc Contacts Stage 3` (`Ctrl+Shift+F12`)

---

## 性能参考

- **逻辑帧耗时:** < 0.3ms (200 Agents, Ryzen 7)
- **Burst 优化:** 核心 Job 利用 SIMD，零 GC
