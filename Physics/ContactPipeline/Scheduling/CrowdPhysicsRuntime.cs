using System;
using Unity.Burst;
using Unity.Collections;
using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics;
using RTS.Unit.FlowField.Diagnostics;
using RTS.Unit.FlowField.Jobs;

namespace RTS.Unit.FlowField.Systems
{
/// <summary>
/// One World-scoped Crowd Physics runtime. Persistent broad/narrow-phase state
/// never crosses this public boundary.
/// </summary>
public sealed class CrowdPhysicsRuntime : IDisposable
{
    private CrossFrameCache _persistent;
#if RTS_CONTACT_DIAGNOSTICS
    private NativeList<SimulationDebuggerPairSample> _debuggerSelectedPairs;
    private NativeReference<SimulationDebuggerUnitSample> _debuggerSelectedUnit;
    private NativeReference<byte> _debuggerSelectedUnitValid;
#endif
    private bool _isCreated;

    private CrowdPhysicsRuntime()
    {
        try
        {
            _persistent = CrossFrameCache.Create();
#if RTS_CONTACT_DIAGNOSTICS
            _debuggerSelectedPairs =
                new NativeList<SimulationDebuggerPairSample>(
                    64, Allocator.Persistent);
            _debuggerSelectedUnit =
                new NativeReference<SimulationDebuggerUnitSample>(
                    Allocator.Persistent);
            _debuggerSelectedUnitValid =
                new NativeReference<byte>(Allocator.Persistent);
#endif
            _isCreated = true;
        }
        catch
        {
#if RTS_CONTACT_DIAGNOSTICS
            if (_debuggerSelectedPairs.IsCreated)
                _debuggerSelectedPairs.Dispose();
            if (_debuggerSelectedUnit.IsCreated)
                _debuggerSelectedUnit.Dispose();
            if (_debuggerSelectedUnitValid.IsCreated)
                _debuggerSelectedUnitValid.Dispose();
#endif
            _persistent.Dispose();
            throw;
        }
    }

    public static CrowdPhysicsRuntime Create() => new CrowdPhysicsRuntime();

    public void EnsureCapacity(int bodyCount)
    {
        ThrowIfDisposed();
        _persistent.EnsureCapacity(bodyCount);
    }

    public CrowdPhysicsStep CreateStep(int bodyCount)
    {
        ThrowIfDisposed();
        return new CrowdPhysicsStep(bodyCount);
    }

    public CrowdPhysicsDiagnosticsStep CreateDiagnosticsStep(
        int bodyCount,
        int substepCount,
        int iterationCount)
    {
        ThrowIfDisposed();
        return new CrowdPhysicsDiagnosticsStep(
            bodyCount,
            substepCount,
            iterationCount);
    }

    public CrowdPhysicsScheduleHandles ScheduleStep(
        CrowdPhysicsStep step,
        ContactPipelineConfiguration configuration,
        CrowdObstacleSnapshot obstacles,
        CrowdPhysicsDiagnosticsStep diagnostics,
        Entity diagnosticSelectedEntity,
        SimulationDebuggerCaptureMask captureMask,
        int maximumVisualizedPairs,
        float deltaTime,
        JobHandle inputReady)
    {
        ThrowIfDisposed();
        if (step == null)
            throw new ArgumentNullException(nameof(step));
        if (diagnostics == null)
            throw new ArgumentNullException(nameof(diagnostics));

        CrowdStepBodyResources body = step.Body;
        TimestepCache timestep = step.Timestep;
        ContactDiagnosticsFrameResources diagnosticResources =
            diagnostics.Resources;
        JobHandle latestScheduled = inputReady;
        try
        {
            JobHandle adaptInput = new AdaptCrowdPhysicsStepInputJob
            {
                Inputs = body.Input.Bodies,
                Bodies = body.Bodies,
                NavigationStates = body.NavigationStates,
                MotionIntents = body.MotionIntents
            }.Schedule(body.Bodies.Length, 64, inputReady);
            latestScheduled = adaptInput;
            JobHandle initialize = new InitializeCrowdStepStateJob
            {
                Bodies = body.Bodies,
                MotionEvidence = body.MotionEvidence,
                StepStates = body.StepStates
            }.Schedule(body.Bodies.Length, 64, adaptInput);
            latestScheduled = initialize;
            JobHandle solve = CrowdPhysicsPipelineComposition.ScheduleStep(
                configuration,
                obstacles,
                body,
                body.Input,
                timestep.Products.BroadPhaseCandidates,
                timestep.Products.NarrowPhaseConstraints,
                _persistent,
                timestep,
                diagnosticResources,
                diagnosticSelectedEntity,
                captureMask,
                maximumVisualizedPairs,
#if RTS_CONTACT_DIAGNOSTICS
                _debuggerSelectedPairs,
                _debuggerSelectedUnit,
                _debuggerSelectedUnitValid,
#else
                default,
                default,
                default,
#endif
                initialize);
            latestScheduled = solve;
            JobHandle outputReady = new BuildCrowdBodyResultsJob
            {
                DeltaTime = deltaTime,
                Bodies = body.Bodies,
                NavigationStates = body.NavigationStates,
                StepStates = body.StepStates,
                Results = body.Results.AsArray()
            }.Schedule(body.Bodies.Length, 64, solve);
            latestScheduled = outputReady;
            return new CrowdPhysicsScheduleHandles(solve, outputReady);
        }
        catch
        {
            // Complete the latest handle returned by every successful schedule
            // before resetting persistent state or letting the caller release
            // timestep containers.
            latestScheduled.Complete();
            _persistent.Reset();
            throw;
        }
    }

    public void Reset()
    {
        ThrowIfDisposed();
        _persistent.Reset();
    }

    public int DebugSweptProxyCount
    {
        get
        {
            ThrowIfDisposed();
            return _persistent.DebugSweptProxyCount;
        }
    }

    public PersistentSweptProxy ReadDebugSweptProxy(int index)
    {
        ThrowIfDisposed();
        if ((uint)index >= (uint)_persistent.DebugSweptProxyCount)
            throw new ArgumentOutOfRangeException(nameof(index));
        return _persistent.ReadDebugSweptProxy(index);
    }

    public bool TryReadSimulationDebuggerSelectedUnit(
        out SimulationDebuggerUnitSample sample)
    {
        ThrowIfDisposed();
#if RTS_CONTACT_DIAGNOSTICS
        if (_debuggerSelectedUnitValid.Value == 0)
        {
            sample = default;
            return false;
        }

        sample = _debuggerSelectedUnit.Value;
        return true;
#else
        sample = default;
        return false;
#endif
    }

    public int SimulationDebuggerSelectedPairCount
    {
        get
        {
            ThrowIfDisposed();
#if RTS_CONTACT_DIAGNOSTICS
            return _debuggerSelectedPairs.Length;
#else
            return 0;
#endif
        }
    }

    public SimulationDebuggerPairSample ReadSimulationDebuggerSelectedPair(
        int index)
    {
        ThrowIfDisposed();
#if RTS_CONTACT_DIAGNOSTICS
        if ((uint)index >= (uint)_debuggerSelectedPairs.Length)
            throw new ArgumentOutOfRangeException(nameof(index));
        return _debuggerSelectedPairs[index];
#else
        throw new ArgumentOutOfRangeException(nameof(index));
#endif
    }

    public void Dispose()
    {
        if (!_isCreated)
            return;

        _persistent.Dispose();
#if RTS_CONTACT_DIAGNOSTICS
        if (_debuggerSelectedPairs.IsCreated)
            _debuggerSelectedPairs.Dispose();
        if (_debuggerSelectedUnit.IsCreated)
            _debuggerSelectedUnit.Dispose();
        if (_debuggerSelectedUnitValid.IsCreated)
            _debuggerSelectedUnitValid.Dispose();
        _debuggerSelectedPairs = default;
        _debuggerSelectedUnit = default;
        _debuggerSelectedUnitValid = default;
#endif
        _persistent = default;
        _isCreated = false;
    }

    private void ThrowIfDisposed()
    {
        if (!_isCreated)
            throw new ObjectDisposedException(nameof(CrowdPhysicsRuntime));
    }
}

/// <summary>
/// Cohesive one-timestep diagnostics lease. Mutable telemetry containers stay
/// inside Physics; Gameplay can only schedule publication and release them.
/// </summary>
public sealed class CrowdPhysicsDiagnosticsStep
{
    private ContactDiagnosticsFrameResources _resources;
    private bool _isDisposed;

    internal CrowdPhysicsDiagnosticsStep(
        int bodyCount,
        int substepCount,
        int iterationCount)
    {
        if (bodyCount <= 0)
            throw new ArgumentOutOfRangeException(nameof(bodyCount));

#if RTS_CONTACT_DIAGNOSTICS
        try
        {
            _resources.IncrementalOracleContactPairs =
                new NativeList<BodyPair>(
                    math.max(bodyCount * 4, 1),
                    Allocator.TempJob);
            _resources.ParallelPairCandidates =
                new NativeList<ParallelSimulationDebuggerPairCapture>(
                    math.max(bodyCount * 4, 1),
                    Allocator.TempJob);
            _resources.ParallelPairScratch =
                new NativeList<SimulationDebuggerPairSample>(
                    math.max(bodyCount, 1),
                    Allocator.TempJob);
            _resources.Iterations =
                new NativeList<ContactIterationDiagnostic>(
                    math.max(substepCount * iterationCount, 1),
                    Allocator.TempJob);
            _resources.Pairs =
                new NativeList<ContactPairDiagnostic>(
                    math.max(bodyCount * 2, 1),
                    Allocator.TempJob);
            _resources.SelectedBody =
                new NativeReference<SelectedBodyContactDiagnostic>(
                    Allocator.TempJob);
            _resources.HeatSamples =
                new NativeArray<ContactHeatSample>(
                    bodyCount,
                    Allocator.TempJob,
                    NativeArrayOptions.ClearMemory);
            _resources.ContactStatistics =
                new NativeReference<PredictiveDiscContactStatistics>(
                    Allocator.TempJob);
            _resources.IncrementalStatistics =
                new NativeReference<IncrementalContactPipelineStatistics>(
                    Allocator.TempJob);
        }
        catch
        {
            DisposeCreatedDiagnosticsResources();
            throw;
        }
#endif
    }

    internal ContactDiagnosticsFrameResources Resources
    {
        get
        {
            ThrowIfDisposed();
            return _resources;
        }
    }

    public JobHandle ScheduleStatisticsPublication(JobHandle solve)
    {
        ThrowIfDisposed();
#if RTS_CONTACT_DIAGNOSTICS
        return new PublishPredictiveDiscContactStatisticsJob
        {
            Source = _resources.ContactStatistics,
            SelectedBodySource = _resources.SelectedBody,
            IterationSource = _resources.Iterations,
            PairSource = _resources.Pairs,
            HeatSource = _resources.HeatSamples
        }.Schedule(solve);
#else
        return solve;
#endif
    }

    public JobHandle ScheduleIncrementalPublication(
        CompletedSimulationStepMetadata completedStep,
        IncrementalContactPipelineConfiguration configuration,
        Entity target,
        ComponentLookup<IncrementalContactPipelineSnapshot> snapshotLookup,
        JobHandle solve)
    {
        ThrowIfDisposed();
#if RTS_CONTACT_DIAGNOSTICS
        return new PublishIncrementalContactPipelineStatisticsJob
        {
            CompletedStep = completedStep,
            Configuration = configuration,
            SolverSource = _resources.ContactStatistics,
            Source = _resources.IncrementalStatistics,
            Target = target,
            SnapshotLookup = snapshotLookup
        }.Schedule(solve);
#else
        return solve;
#endif
    }

    public JobHandle Dispose(
        JobHandle solve,
        JobHandle statisticsPublication,
        JobHandle incrementalPublication)
    {
        ThrowIfDisposed();
        _isDisposed = true;
#if RTS_CONTACT_DIAGNOSTICS
        JobHandle a = _resources.IncrementalOracleContactPairs.Dispose(solve);
        JobHandle b = _resources.ParallelPairCandidates.Dispose(solve);
        JobHandle c = _resources.ParallelPairScratch.Dispose(solve);
        JobHandle d = _resources.SelectedBody.Dispose(statisticsPublication);
        JobHandle e = _resources.Iterations.Dispose(statisticsPublication);
        JobHandle f = _resources.Pairs.Dispose(statisticsPublication);
        JobHandle g = _resources.HeatSamples.Dispose(statisticsPublication);
        JobHandle published = JobHandle.CombineDependencies(
            statisticsPublication,
            incrementalPublication);
        JobHandle h = _resources.ContactStatistics.Dispose(published);
        JobHandle i = _resources.IncrementalStatistics.Dispose(
            incrementalPublication);
        _resources = default;
        return JobHandle.CombineDependencies(
            JobHandle.CombineDependencies(a, b, c),
            JobHandle.CombineDependencies(d, e, f),
            JobHandle.CombineDependencies(g, h, i));
#else
        _resources = default;
        return default;
#endif
    }

    private void ThrowIfDisposed()
    {
        if (_isDisposed)
            throw new ObjectDisposedException(
                nameof(CrowdPhysicsDiagnosticsStep));
    }

#if RTS_CONTACT_DIAGNOSTICS
    private void DisposeCreatedDiagnosticsResources()
    {
        if (_resources.IncrementalOracleContactPairs.IsCreated)
            _resources.IncrementalOracleContactPairs.Dispose();
        if (_resources.ParallelPairCandidates.IsCreated)
            _resources.ParallelPairCandidates.Dispose();
        if (_resources.ParallelPairScratch.IsCreated)
            _resources.ParallelPairScratch.Dispose();
        if (_resources.Iterations.IsCreated)
            _resources.Iterations.Dispose();
        if (_resources.Pairs.IsCreated)
            _resources.Pairs.Dispose();
        if (_resources.SelectedBody.IsCreated)
            _resources.SelectedBody.Dispose();
        if (_resources.HeatSamples.IsCreated)
            _resources.HeatSamples.Dispose();
        if (_resources.ContactStatistics.IsCreated)
            _resources.ContactStatistics.Dispose();
        if (_resources.IncrementalStatistics.IsCreated)
            _resources.IncrementalStatistics.Dispose();
        _resources = default;
    }
#endif
}

/// <summary>
/// One-timestep lease. The public surface contains only the immutable input
/// product, the solved output product and lifetime operations.
/// </summary>
public sealed class CrowdPhysicsStep
{
    private CrowdStepBodyResources _body;
    private TimestepCache _timestep;
    private bool _isDisposed;

    internal CrowdPhysicsStep(int bodyCount)
    {
        if (bodyCount <= 0)
            throw new ArgumentOutOfRangeException(nameof(bodyCount));

        _body = CrowdStepBodyResources.Create(bodyCount);
        try
        {
            _timestep = TimestepCache.Create(bodyCount);
        }
        catch
        {
            _body.Dispose(default).Complete();
            _body = default;
            throw;
        }
    }

    public NativeArray<CrowdPhysicsBodyInput> InputBodies
    {
        get
        {
            ThrowIfDisposed();
            return _body.StepInputs;
        }
    }

    public NativeArray<CrowdBodyResult>.ReadOnly OutputBodies
    {
        get
        {
            ThrowIfDisposed();
            return _body.Results.AsReadOnly();
        }
    }

    internal CrowdStepBodyResources Body
    {
        get
        {
            ThrowIfDisposed();
            return _body;
        }
    }

    internal TimestepCache Timestep
    {
        get
        {
            ThrowIfDisposed();
            return _timestep;
        }
    }

    public bool TryGetIncidentIndexDesync(
        out IncidentIndexDesyncReport report)
    {
        ThrowIfDisposed();
#if RTS_CONTACT_DIAGNOSTICS
        int offsetsOutOfRange = 0;
        int pairIndexOutOfRange = 0;
        int correctionIndexOutOfRange = 0;
        NativeArray<byte> flags = _timestep.Solver.CorrectedBodyFlags;
        for (int i = 0; i < flags.Length; i++)
        {
            switch (flags[i])
            {
                case 2:
                    offsetsOutOfRange++;
                    break;
                case 3:
                    pairIndexOutOfRange++;
                    break;
                case 4:
                    correctionIndexOutOfRange++;
                    break;
            }
        }

        ActiveIncidentIndexState state =
            _timestep.Solver.ActiveIncidentIndexState.Value;
        report = new IncidentIndexDesyncReport(
            flags.Length,
            offsetsOutOfRange,
            pairIndexOutOfRange,
            correctionIndexOutOfRange,
            state.PairCount,
            state.BodyCount,
            state.IsValid,
            _timestep.Products.TimestepContactPairs.Length,
            _timestep.Solver.JacobiPairCorrections.Length,
            _timestep.Solver.ActiveIncidentPairIndices.Length);
        return report.TotalOutOfRange > 0;
#else
        report = default;
        return false;
#endif
    }

    public JobHandle Dispose(JobHandle outputReader, JobHandle solverReader)
    {
        ThrowIfDisposed();
        _isDisposed = true;
        JobHandle bodyDispose = _body.Dispose(outputReader);
        JobHandle timestepDispose = _timestep.Dispose(solverReader);
        _body = default;
        _timestep = default;
        return JobHandle.CombineDependencies(bodyDispose, timestepDispose);
    }

    private void ThrowIfDisposed()
    {
        if (_isDisposed)
            throw new ObjectDisposedException(nameof(CrowdPhysicsStep));
    }
}

public readonly struct CrowdPhysicsScheduleHandles
{
    public readonly JobHandle Solve;
    public readonly JobHandle OutputReady;

    internal CrowdPhysicsScheduleHandles(
        JobHandle solve,
        JobHandle outputReady)
    {
        Solve = solve;
        OutputReady = outputReady;
    }
}

public readonly struct IncidentIndexDesyncReport
{
    public readonly int BodyCount;
    public readonly int OffsetsOutOfRange;
    public readonly int PairIndexOutOfRange;
    public readonly int CorrectionIndexOutOfRange;
    public readonly int ExpectedPairCount;
    public readonly int ExpectedBodyCount;
    public readonly byte IndexIsValid;
    public readonly int ContactPairCount;
    public readonly int CorrectionCount;
    public readonly int IncidentPairIndexCount;

    public int TotalOutOfRange =>
        OffsetsOutOfRange +
        PairIndexOutOfRange +
        CorrectionIndexOutOfRange;

    internal IncidentIndexDesyncReport(
        int bodyCount,
        int offsetsOutOfRange,
        int pairIndexOutOfRange,
        int correctionIndexOutOfRange,
        int expectedPairCount,
        int expectedBodyCount,
        byte indexIsValid,
        int contactPairCount,
        int correctionCount,
        int incidentPairIndexCount)
    {
        BodyCount = bodyCount;
        OffsetsOutOfRange = offsetsOutOfRange;
        PairIndexOutOfRange = pairIndexOutOfRange;
        CorrectionIndexOutOfRange = correctionIndexOutOfRange;
        ExpectedPairCount = expectedPairCount;
        ExpectedBodyCount = expectedBodyCount;
        IndexIsValid = indexIsValid;
        ContactPairCount = contactPairCount;
        CorrectionCount = correctionCount;
        IncidentPairIndexCount = incidentPairIndexCount;
    }
}
}

namespace RTS.Unit.FlowField.Jobs
{
/// <summary>
/// Expands the public AoS input product into Physics-owned SoA arrays.
/// </summary>
[BurstCompile]
internal struct AdaptCrowdPhysicsStepInputJob : IJobParallelFor
{
    [ReadOnly] public NativeArray<CrowdPhysicsBodyInput> Inputs;
    public NativeArray<CrowdBodySnapshot> Bodies;
    public NativeArray<CrowdNavigationState> NavigationStates;
    public NativeArray<CrowdMotionIntent> MotionIntents;

    public void Execute(int bodyIndex)
    {
        CrowdPhysicsBodyInput input = Inputs[bodyIndex];
        Bodies[bodyIndex] = new CrowdBodySnapshot
        {
            Entity = input.StableId,
            Position = input.Position,
            Rotation = input.Rotation,
            Velocity = input.Velocity,
            MoveSpeed = input.MoveSpeed,
            MaxAcceleration = input.MaxAcceleration,
            InverseMass = input.InverseMass,
            Radius = input.Radius,
            ShapeVersion = input.ShapeVersion,
            IsInsideSimulationDomain = input.IsInsideSimulationDomain
        };
        NavigationStates[bodyIndex] = new CrowdNavigationState
        {
            IsSettled = input.IsSettled
        };
        MotionIntents[bodyIndex] = new CrowdMotionIntent
        {
            PreferredVelocity = input.PreferredVelocity,
            SteeringVelocityError = input.SteeringVelocityError
        };
    }
}
}
