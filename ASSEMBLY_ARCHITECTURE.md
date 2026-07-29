# RTS 脚本程序集与目录边界

## 拆分前

业务代码全部编入 Unity 默认 `Assembly-CSharp`，目录只表达约定，不形成编译边界：

```text
Assets/Scripts
├── Entities
│   ├── Building
│   ├── Camera
│   ├── Unit
│   │   └── Systems/FlowField/Runtime/ContactPipeline
│   ├── _Common
│   └── _RePlay
├── NetWorkInitialize
├── Utils
├── _PlayerInput
└── _QFrameWork
    ├── BuildingManagement
    ├── UISystem
    └── _CommonUtils
```

主要问题：

- 接触、宽相位、软避让和 XPBD 求解位于 `Unit` 目录内部。
- Gameplay、Network、UI 可以任意访问彼此的内部实现。
- 修改任何业务脚本都会触发同一个默认程序集重新编译。

## 拆分后

```text
Assets/Scripts
├── Shared                         # RTS.Shared
│   ├── Utils
│   ├── ServerObjectSystem.cs
│   └── SystemServiceLocator.cs
├── Gameplay
│   ├── Core                       # RTS.Gameplay.Core
│   │   ├── Domain
│   │   ├── Replay
│   │   └── Unit
│   │       ├── Components
│   │       └── FlowField/Utility
│   └── Entities                   # RTS.Gameplay
│       ├── Building
│       ├── Unit
│       ├── _Common
│       └── _RePlay
├── Physics                        # RTS.Physics
│   ├── Configuration
│   ├── ContactPipeline
│   │   ├── Contracts
│   │   ├── Kernels
│   │   ├── Scheduling
│   │   ├── Stages
│   │   └── State
│   ├── Diagnostics
│   ├── Jobs
│   ├── Spatial
│   └── Editor                     # RTS.Physics.Editor
├── Network                        # RTS.Network
│   ├── Client
│   ├── Contracts
│   ├── FlowField
│   ├── Initialization
│   ├── Replay
│   ├── Spawn
│   └── Units
└── UI                             # Unity Assembly-CSharp
    ├── BuildingManagement
    ├── Camera
    ├── Common
    ├── Diagnostics
    ├── Framework
    ├── HealthBarSystems
    ├── Input
    ├── Network
    ├── Replay
    └── Selection
```

## 引用方向

```text
RTS.Shared
    ↓
RTS.Gameplay.Core
    ↓
RTS.Physics
    ↓
RTS.Gameplay
    ↓
RTS.Network
    ↓
Assembly-CSharp (UI/Input/Composition)
```

规则：

1. `RTS.Gameplay.Core` 只放跨模块数据合同和纯工具，不依赖 Physics、Network 或 UI。
2. `RTS.Physics` 负责碰撞体快照、候选生成、接触认证、软避让、XPBD、墙体约束和物理诊断数据。
3. `RTS.Gameplay` 负责游戏规则与系统编排，可调用 Physics，但 Physics 不反向调用 Gameplay 系统。
4. `RTS.Network` 负责 RPC、客户端/服务器初始化和 NetCode 专用系统。
5. UI、输入和场景组合只允许位于依赖链末端。

## UI 为什么保留在 Assembly-CSharp

当前 UI 直接依赖以下位于本 Git 根目录之外的源码：

- `Assets/Qframework/QFramework.cs`
- `Assets/Resources/PlayerAction.cs`

Unity 自定义 asmdef 不能引用默认 `Assembly-CSharp`。为了避免复制第三方源码或修改未纳入当前仓库的资源，UI 暂时作为默认程序集的唯一业务组合层。它仍与 `RTS.Gameplay`、`RTS.Physics`、`RTS.Network` 形成真实编译隔离。
