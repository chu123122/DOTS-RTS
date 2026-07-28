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
/// Authoritative cross-timestep runtime state used by correctness decisions.
///
/// This type contains only cache validity, versioning, lifecycle gauges needed to
/// rebuild derived views, and the configuration certificate that proves reuse is
/// safe. Presentation code, CSV recorders, timers, oracle counters and heatmap
/// samples must never be stored here or consulted through this type.
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
