# High-Performance RTS Tech Stack based on Unity DOTS
# 基于 Unity DOTS 的高性能 RTS 同步架构

![Unity](https://img.shields.io/badge/Unity-2022.3%2B-black)
![DOTS](https://img.shields.io/badge/Tech-ECS%20%7C%20Jobs%20%7C%20Burst-blue)
![NetCode](https://img.shields.io/badge/Network-NetCode%20for%20Entities-green)
![License](https://img.shields.io/badge/License-MIT-yellow)

> **核心亮点**：200+ 高密度单位同屏物理模拟 | < 1ms 逻辑耗时 | 基于指令的确定性回放

---

## 📺 [Click to Watch Demo Video (Bilibili / YouTube)]

---

## 📖 项目简介 (Introduction)

本项目是一个基于 **Unity DOTS (Data-Oriented Technology Stack)** 构建的高性能 RTS 游戏核心架构原型。

旨在解决大规模单位（Massive Entities）在多人联机环境下的**寻路性能**、**物理避障稳定性**以及**确定性同步**问题。项目摒弃了传统的 OOP 开发模式，完全采用 **ECS (Entity Component System)** 架构，利用 Burst Compiler 实现极致的 CPU 性能优化。

## ✨ 核心特性 (Key Features)

### 1. 海量单位流场寻路 (Flow Field Pathfinding)
摒弃了传统的 A* 算法，实现了基于 **Eikonal 方程** 的向量场寻路。
* **性能：** 支持 **500+** 单位同时寻路，寻路逻辑完全并行化（Job System）。
* **机制：** 通过 `Physics.Overlap` 预计算静态障碍物代价场 (Cost Field)，生成全局积分场 (Integration Field)，实现 $O(1)$ 复杂度的单位寻路查询。

![FlowField](docs/images/flowfield_demo.gif) 
*(这里放你那个大方阵移动的 GIF)*

### 2. 基于 PBD 的高密度避障 (PBD Avoidance)
为了解决大量单位挤过窄口（沙漏场景）时的死锁与穿模问题，实现了一套专用的 **基于位置的动力学 (Position Based Dynamics)** 求解器。
* **分层处理：** 结合 **软分离力 (Soft Separation)** 维持队形与 **硬约束投影 (Hard Constraint Projection)** 修正穿模。
* **稳定性：** 引入帧率无关阻尼 (Frame-rate Independent Damping) 和拥堵截断机制，在极度拥挤下依然保持物理收敛，无鬼畜抖动。

![Avoidance](docs/images/avoidance_demo.gif)
*(这里放你那个沙漏挤压的 GIF)*

### 3. 事件溯源回放系统 (Event Sourcing Replay)
在服务端权威（Server-Authoritative）架构下，实现了一套基于指令流的回放系统。
* **零快照：** 不记录每帧 Transform，仅记录关键输入指令 (Command Buffer)。
* **瞬间重置：** 利用 ECS 的结构特性，实现了毫秒级的状态回滚与指令重演。

![Replay](docs/images/replay_demo.gif)
*(这里放你按 R 键回放的 GIF)*

### 4. 混合式网络架构 (Hybrid Network Architecture)
* **框架：** 基于 **Unity NetCode for Entities**。
* **策略：** 采用服务端权威 (Server-Auth) + 客户端预测 (Client-Side Prediction) 方案，结合本地模拟层，实现了断线后的本地平滑回放。

---

## 🛠️ 技术架构 (Architecture)

### ECS Systems Overview
* **Simulation Group:**
    * `FlowFieldBakeSystem`: 负责计算 Cost Field 和 Vector Field (Parallel Jobs)。
    * `UnitSpatialPartitionSystem`: 维护空间哈希网格 (Spatial Hash Map)，加速邻居查询。
    * `UnitFlowMovementSystem`: 核心物理求解器，处理 PBD 约束与速度积分。
* **Replay Group:**
    * `CommandRecordingSystem`: 负责 RPC 指令的序列化与 Buffer 写入。
    * `CommandReplayingSystem`: 负责时间轴管理与状态重置 (Event Sourcing)。

### Performance (Profiler Data)
在 Ryzen 7  上实测：
* **逻辑帧耗时:** < 0.3ms (200 Agents)
* **Burst 优化:** 核心 Job (`MoveAlongFlowFieldJob`) 利用 SIMD 指令集加速，无 GC开销。

<img src="https://cdn.jsdelivr.net/gh/chu123122/Image-hosting-service/img/%E6%98%BE%E7%A4%BA.png"/>
