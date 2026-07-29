using System;
using System.Collections.Generic;
using Unity.Entities;
using Unity.Mathematics;

namespace RTS.Unit.FlowField.Jobs
{
[Flags]
public enum IncrementalBodyDirtyFlags : byte
{
    None = 0,
    Motion = 1 << 0,
    Topology = 1 << 1,
    EntitySet = 1 << 2,
    CorrectedEscape = 1 << 3
}

public struct IncrementalDirtyBody
{
    public int BodyIndex;
    public IncrementalBodyDirtyFlags Flags;
}

/// <summary>
/// 跨 timestep 的权威运行时状态，用于正确性决策。
///
/// 本类型仅含缓存有效性、版本号、生命周期仪表以及证明复用到当前步安全的配置证书。
/// 展示代码、CSV 录制、定时器、oracle 计数器与热力图样本一律不得存放此处，也不得经由此类型访问。
/// </summary>
public struct IncrementalContactCacheState
{
    public byte IsValid;
    public byte LastUpdateWasFullRebuild;
    public byte ContactViewsValid;
    public byte Reserved;
    public uint Timestep;
    public uint TopologyEpoch;
    public uint ClassificationEpoch;
    public int BodyCount;
    public int NeighborPairCount;
    public int DormantContactCount;
    public int ApproachingContactCount;
    public int PredictiveContactCount;
    public int ActualContactCount;
    public int ExpiredContactCount;
    public float GuardMargin;
    public float PredictiveSkin;
    public float TimestepContactMargin;
    public float SoftAvoidanceShell;
    public float SoftAvoidanceResponseRate;
    public float RvoTimeHorizon;
    public int SubstepCount;
    public byte PredictivePairGenerationEnabled;
    public byte PredictiveContactsEnabled;
    public byte SoftAvoidanceVelocitySolver;
    public byte ConfigurationReserved;
}
}
