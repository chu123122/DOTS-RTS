using Unity.Burst;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics;
using Unity.Profiling.LowLevel.Unsafe;
using RTS.Unit.FlowField;
using RTS.Unit.FlowField.Diagnostics;

namespace RTS.Unit.FlowField.Jobs
{
public struct SoftAvoidancePairContribution
{
    public float3 VelocityA;
    public float3 VelocityB;
    public byte ActiveA;
    public byte ActiveB;
}

public struct ParallelBodyStageResult
{
    // EscapeCount is authoritative: it drives dirty-body repair and must exist
    // independently of observation.
    public int EscapeCount;
#if RTS_CONTACT_DIAGNOSTICS
    public float Total;
    public float Maximum;
    public float SecondaryTotal;
    public float TertiaryTotal;
    public int Count;
    public int ActivatedCount;
#else
    public float Total { get => default; set { } }
    public float Maximum { get => default; set { } }
    public float SecondaryTotal { get => default; set { } }
    public float TertiaryTotal { get => default; set { } }
    public int Count { get => default; set { } }
    public int ActivatedCount { get => default; set { } }
#endif
}

public struct ActiveIncidentIndexState
{
    public ulong Fingerprint;
    public int PairCount;
    public int BodyCount;
    public int SoftPairCount;
    public int SoftBodyCount;
    public byte IsValid;
    public byte SoftIsValid;
}

/// <summary>
/// Named responsibilities behind the historical P1-P6 implementation labels.
/// The enum is documentation for scheduling boundaries, not mutable runtime state.
/// </summary>
internal enum StagedContactPipelinePhase : byte
{
    Initialize,
    ResolveInteractionSource,
    RepairPersistentTopology,
    BuildSoftAvoidanceView,
    SolveContactConstraints,
    ReconstructVelocity,
    FinalizeTimestep
}

/// <summary>
/// Staged parallel contact pipeline. The historical P1-P6 labels map to the
/// named <see cref="StagedContactPipelinePhase"/> responsibilities above.
/// Independent body/pair work runs in parallel; topology mutation, repair and
/// deterministic compaction remain serialized at explicit phase boundaries.
/// </summary>
public partial struct CrowdContactPipelineScheduler
{
    private const int ParallelBodyBatchSize = 64;
    private const int SoftPairBatchSize = 64;



    public JobHandle ScheduleParallelJacobiP1P6(
        NativeReference<ParallelJacobiExecutionState> runtimeState,
#if RTS_CONTACT_DIAGNOSTICS
        NativeReference<ParallelJacobiIterationTelemetry> iterationState,
        NativeList<JacobiBlockTelemetry> blockStatistics,
#endif
        JobHandle dependency)
    {
        ContactPipelineLifecycleJob initialize = Lifecycle;
        initialize.Operation = ContactPipelineLifecycleOperation.InitializeParallel;
        initialize.RuntimeState = runtimeState;
        JobHandle handle = initialize.Schedule(dependency);

        int substepCount = math.max(1, Configuration.SubstepCount);
        int iterationCount = math.max(1, Configuration.IterationCount);
        float substepDeltaTime = Configuration.DeltaTime / substepCount;
#if RTS_CONTACT_DIAGNOSTICS
        bool captureSelectedPairs =
            Configuration.EnableDiagnostics && DiagnosticSelectedEntity != Entity.Null &&
            (SimulationDebuggerCaptureMask &
             SimulationDebuggerCaptureMask.SelectedUnit) != 0 &&
            (SimulationDebuggerCaptureMask &
             SimulationDebuggerCaptureMask.SelectedPairs) != 0;
#endif
        int escapeBlockCount =
            (Bodies.Length + ParallelBodyBatchSize - 1) / ParallelBodyBatchSize;
        if (substepDeltaTime <= 0f)
        {
#if RTS_CONTACT_DIAGNOSTICS
            ConstraintSolverJob finalizePipeline = ConstraintSolver;
            finalizePipeline.Operation = ConstraintSolverOperation.FinalizeParallelPipeline;
            finalizePipeline.RuntimeState = runtimeState;
            return finalizePipeline.Schedule(handle);
#else
            return handle;
#endif
        }

        if (Configuration.EnableTimestepContactSetCache)
        {
            handle = new PrepareTimestepPredictionBodiesJob
            {
                Bodies = Bodies,
                NavigationStates = NavigationStates,
                MotionIntents = MotionIntents,
                MotionEvidence = MotionEvidence,
                StepStates = StepStates,
                PersistentProxies = PersistentSweptProxies.AsDeferredJobArray(),
                PersistentProxyIndexByBody = PersistentProxyIndexByBody.AsDeferredJobArray(),
                PersistentCacheState = IncrementalCacheState,
                DirtyFlagsByBody = IncrementalDirtyFlagsByBody,
                Duration = Configuration.DeltaTime,
                Skin = Configuration.PredictiveSkin,
                Margin = Configuration.TimestepContactMargin,
                GuardMargin = Configuration.GuardEnvelopeMargin,
                GridOrigin = GridOrigin,
                CellRadius = CellRadius,
                FromSolvedPosition = 0,
                DetectPersistentDirty = (byte)(Configuration.EnablePersistentContactCache ? 1 : 0),
                SoftAvoidanceShell = Configuration.SoftAvoidanceShell,
                SoftAvoidanceResponseRate = Configuration.SoftAvoidanceResponseRate,
                SoftSolverMode = Configuration.SoftAvoidanceVelocitySolver,
                RvoTimeHorizon = Configuration.RvoTimeHorizon
            }.Schedule(Bodies.Length, ParallelBodyBatchSize, handle);

            if (Configuration.EnablePersistentContactCache)
            {
                handle = new CountInitialP1P6DirtyBodyBlocksJob
                {
                    DirtyFlagsByBody = IncrementalDirtyFlagsByBody,
                    BlockOffsetsAndCounts = DirtyBodyBlockOffsets,
                    BodyCount = Bodies.Length
                }.Schedule(escapeBlockCount, 1, handle);
                handle = new PrefixInitialP1P6DirtyBodiesJob
                {
                    BlockOffsetsAndCounts = DirtyBodyBlockOffsets,
                    DirtyBodies = IncrementalDirtyBodies,
                    BlockCount = escapeBlockCount
                }.Schedule(handle);
                handle = new ScatterInitialP1P6DirtyBodiesJob
                {
                    DirtyFlagsByBody = IncrementalDirtyFlagsByBody,
                    BlockOffsets = DirtyBodyBlockOffsets,
                    DirtyBodies = IncrementalDirtyBodies.AsDeferredJobArray(),
                    BodyCount = Bodies.Length
                }.Schedule(escapeBlockCount, 1, handle);
            }
        }

        if (Configuration.EnableTimestepContactSetCache &&
            Configuration.EnablePersistentContactCache)
        {
            handle = Certification.ScheduleInitialPersistentContactSetP1P6(
                runtimeState,
                handle);
        }
        else
        {
            InteractionCertificationJob buildInitial = Certification;
            buildInitial.Operation = InteractionCertificationOperation.BuildInitialP1P6;
            buildInitial.RuntimeState = runtimeState;
            handle = buildInitial.Schedule(handle);
        }

        for (int substepIndex = 0; substepIndex < substepCount; substepIndex++)
        {
            handle = new PrepareBaseVelocityBodiesJob
            {
                Bodies = Bodies,
                NavigationStates = NavigationStates,
                MotionIntents = MotionIntents,
                MotionEvidence = MotionEvidence,
                StepStates = StepStates,
                SubstepDeltaTime = substepDeltaTime,
                GridOrigin = GridOrigin,
                CellRadius = CellRadius
            }.Schedule(Bodies.Length, ParallelBodyBatchSize, handle);

            if (!Configuration.EnableTimestepContactSetCache)
            {
                handle = new PrepareTimestepPredictionBodiesJob
                {
                    Bodies = Bodies,
                NavigationStates = NavigationStates,
                MotionIntents = MotionIntents,
                MotionEvidence = MotionEvidence,
                StepStates = StepStates,
                    Duration = substepDeltaTime,
                    Skin = Configuration.PredictiveSkin,
                    Margin = Configuration.TimestepContactMargin,
                    GridOrigin = GridOrigin,
                    CellRadius = CellRadius,
                    FromSolvedPosition = 1,
                    SoftAvoidanceShell = Configuration.SoftAvoidanceShell,
                    SoftAvoidanceResponseRate = Configuration.SoftAvoidanceResponseRate,
                    SoftSolverMode = Configuration.SoftAvoidanceVelocitySolver,
                    RvoTimeHorizon = Configuration.RvoTimeHorizon,
                    DetectPersistentDirty = 0
                }.Schedule(Bodies.Length, ParallelBodyBatchSize, handle);
            }

            handle = new ValidateBaseMotionBodiesJob
            {
                Bodies = Bodies,
                NavigationStates = NavigationStates,
                MotionIntents = MotionIntents,
                MotionEvidence = MotionEvidence,
                StepStates = StepStates,
                EscapeFlags = EnvelopeEscapeFlags,
                Enabled = (byte)(Configuration.EnableTimestepContactSetCache ? 1 : 0),
                PredictiveSkin = Configuration.PredictiveSkin,
                TimestepContactMargin = Configuration.TimestepContactMargin,
                SoftAvoidanceShell = Configuration.SoftAvoidanceShell,
                SoftAvoidanceResponseRate = Configuration.SoftAvoidanceResponseRate,
                SoftSolverMode = Configuration.SoftAvoidanceVelocitySolver,
                RvoTimeHorizon = Configuration.RvoTimeHorizon
            }.Schedule(Bodies.Length, ParallelBodyBatchSize, handle);

            handle = new CountP1P6EnvelopeEscapeBlocksJob
            {
                EscapeFlags = EnvelopeEscapeFlags,
                BlockOffsetsAndCounts = DirtyBodyBlockOffsets,
                BodyCount = Bodies.Length,
                Enabled = (byte)(Configuration.EnableTimestepContactSetCache ? 1 : 0)
            }.Schedule(escapeBlockCount, 1, handle);

            handle = new PrefixP1P6EnvelopeEscapesJob
            {
                BlockOffsetsAndCounts = DirtyBodyBlockOffsets,
                DirtyBodies = IncrementalDirtyBodies,
                DirtyFlagsByBody = IncrementalDirtyFlagsByBody,
                BlockCount = escapeBlockCount
            }.Schedule(handle);

            handle = new ScatterP1P6EnvelopeEscapesJob
            {
                EscapeFlags = EnvelopeEscapeFlags,
                BlockOffsets = DirtyBodyBlockOffsets,
                DirtyBodies = IncrementalDirtyBodies.AsDeferredJobArray(),
                DirtyFlagsByBody = IncrementalDirtyFlagsByBody,
                Bodies = Bodies,
                NavigationStates = NavigationStates,
                MotionIntents = MotionIntents,
                MotionEvidence = MotionEvidence,
                StepStates = StepStates,
                BodyStatistics = ParallelBodyStatistics,
                BodyCount = Bodies.Length
            }.Schedule(escapeBlockCount, 1, handle);

            InteractionCertificationJob finalizeEscapes = Certification;
            finalizeEscapes.Operation = InteractionCertificationOperation.FinalizeEnvelopeEscapesP1P6;
            finalizeEscapes.RuntimeState = runtimeState;
            finalizeEscapes.SubstepIndex = substepIndex;
            handle = finalizeEscapes.Schedule(handle);

            handle = new PrepareP1P6RepairPredictionBodiesJob
            {
                Bodies = Bodies,
                NavigationStates = NavigationStates,
                MotionIntents = MotionIntents,
                MotionEvidence = MotionEvidence,
                StepStates = StepStates,
                DirtyBodies = IncrementalDirtyBodies.AsDeferredJobArray(),
                Duration = math.max(
                    substepDeltaTime,
                    (substepCount - substepIndex) * substepDeltaTime),
                Skin = Configuration.PredictiveSkin,
                Margin = Configuration.TimestepContactMargin,
                GridOrigin = GridOrigin,
                CellRadius = CellRadius,
                SoftAvoidanceShell = Configuration.SoftAvoidanceShell,
                SoftAvoidanceResponseRate = Configuration.SoftAvoidanceResponseRate,
                SoftSolverMode = Configuration.SoftAvoidanceVelocitySolver,
                RvoTimeHorizon = Configuration.RvoTimeHorizon,
                Enabled = (byte)(Configuration.EnableTimestepContactSetCache ? 1 : 0)
            }.Schedule(IncrementalDirtyBodies, ParallelBodyBatchSize, handle);

            InteractionCertificationJob prepareRepair = Certification;
            prepareRepair.Operation = InteractionCertificationOperation.PrepareSubstepRepairP1P6;
            prepareRepair.RuntimeState = runtimeState;
            prepareRepair.SubstepIndex = substepIndex;
            handle = prepareRepair.Schedule(handle);

            handle = new EvaluatePersistentPairClassificationsP1P6Job
            {
                Bodies = Bodies,
                NavigationStates = NavigationStates,
                MotionIntents = MotionIntents,
                MotionEvidence = MotionEvidence,
                StepStates = StepStates,
                RawPairs = ClassificationBodyPairs.AsDeferredJobArray(),
                PersistentProxies = PersistentSweptProxies.AsDeferredJobArray(),
                PreviousContacts = PersistentPredictiveContacts.AsDeferredJobArray(),
                DirtyFlagsByBody = IncrementalDirtyFlagsByBody,
                PhaseState = PersistentClassificationState,
                Results = PersistentClassificationResults.AsDeferredJobArray(),
                PredictiveSkin = Configuration.PredictiveSkin,
                TimestepContactMargin = Configuration.TimestepContactMargin,
                SoftAvoidanceShell = Configuration.SoftAvoidanceShell,
                SoftAvoidanceResponseRate = Configuration.SoftAvoidanceResponseRate,
                SoftAvoidanceVelocitySolver = Configuration.SoftAvoidanceVelocitySolver,
                RvoTimeHorizon = Configuration.RvoTimeHorizon,
                EnablePredictivePairGeneration =
                    (byte)(Configuration.EnablePredictivePairGeneration ? 1 : 0),
                EnablePredictiveContacts =
                    (byte)(Configuration.EnablePredictiveContacts ? 1 : 0),
                SubstepCount = math.max(1, Configuration.SubstepCount),
                ScheduleStartSubstep = substepIndex
            }.Schedule(PersistentClassificationResults, SoftPairBatchSize, handle);

            InteractionCertificationJob commitRepair = Certification;
            commitRepair.Operation = InteractionCertificationOperation.CommitSubstepRepairP1P6;
            commitRepair.RuntimeState = runtimeState;
            commitRepair.SubstepIndex = substepIndex;
            handle = commitRepair.Schedule(handle);

            SoftAvoidanceJob prepareSoft = SoftAvoidance;
            prepareSoft.Operation = SoftAvoidanceOperation.PrepareParallelWorkset;
            prepareSoft.RuntimeState = runtimeState;
#if RTS_CONTACT_DIAGNOSTICS
            prepareSoft.BlockStatistics = blockStatistics;
#endif
            handle = prepareSoft.Schedule(handle);

            handle = new InitializeSoftAvoidanceBodiesJob
            {
                Bodies = Bodies,
                NavigationStates = NavigationStates,
                MotionIntents = MotionIntents,
                MotionEvidence = MotionEvidence,
                StepStates = StepStates,
                Grid = Grid,
                GridOrigin = GridOrigin,
                GridDimensions = GridDimensions,
                CellRadius = CellRadius,
                SoftShell = Configuration.SoftAvoidanceShell
            }.Schedule(Bodies.Length, ParallelBodyBatchSize, handle);

            var evaluateSoftPairsJob = new EvaluateSoftAvoidancePairsJob
            {
                Bodies = Bodies,
                NavigationStates = NavigationStates,
                MotionIntents = MotionIntents,
                MotionEvidence = MotionEvidence,
                StepStates = StepStates,
                Pairs = SoftAvoidancePairs.AsDeferredJobArray(),
                Contributions = SoftPairContributions.AsDeferredJobArray(),
                SolverMode = Configuration.SoftAvoidanceVelocitySolver,
                SoftShell = Configuration.SoftAvoidanceShell,
                RvoTimeHorizon = Configuration.RvoTimeHorizon,
                SubstepDeltaTime = substepDeltaTime
            };
            handle = evaluateSoftPairsJob.Schedule(
                SoftAvoidancePairs,
                SoftPairBatchSize,
                handle);

#if RTS_CONTACT_DIAGNOSTICS
            handle = new ReduceSoftAvoidanceBlocksJob
            {
                Contributions = SoftPairContributions.AsDeferredJobArray(),
                Blocks = blockStatistics.AsDeferredJobArray()
            }.Schedule(blockStatistics, 1, handle);
#endif

            handle = new GatherSoftAvoidanceBodiesJob
            {
                Bodies = Bodies,
                NavigationStates = NavigationStates,
                MotionIntents = MotionIntents,
                MotionEvidence = MotionEvidence,
                StepStates = StepStates,
                Pairs = SoftAvoidancePairs.AsDeferredJobArray(),
                Contributions = SoftPairContributions.AsDeferredJobArray(),
                IncidentOffsets = SoftIncidentOffsets,
                IncidentPairIndices = SoftIncidentPairIndices.AsDeferredJobArray(),
                EscapeFlags = EnvelopeEscapeFlags,
                SoftSolverMode = Configuration.SoftAvoidanceVelocitySolver,
                SoftAvoidanceResponseRate = Configuration.SoftAvoidanceResponseRate,
                SettledMultiplier = Configuration.SettledSoftAvoidanceMultiplier,
                SubstepDeltaTime = substepDeltaTime,
                PredictiveSkin = Configuration.PredictiveSkin,
                TimestepContactMargin = Configuration.TimestepContactMargin,
                SoftShell = Configuration.SoftAvoidanceShell,
                ClampToEnvelope = (byte)(Configuration.EnableTimestepContactSetCache ? 1 : 0)
            }.Schedule(Bodies.Length, ParallelBodyBatchSize, handle);

#if RTS_CONTACT_DIAGNOSTICS
            handle = new ReduceP1P6SoftEscapeBlocksJob
            {
                EscapeFlags = EnvelopeEscapeFlags,
                EscapeCountsByBlock = DirtyBodyBlockOffsets,
                BodyCount = Bodies.Length
            }.Schedule(escapeBlockCount, 1, handle);

            SoftAvoidanceJob finalizeSoft = SoftAvoidance;
            finalizeSoft.Operation = SoftAvoidanceOperation.FinalizeParallel;
            finalizeSoft.RuntimeState = runtimeState;
            finalizeSoft.BlockStatistics = blockStatistics;
            finalizeSoft.EscapeCountsByBlock = DirtyBodyBlockOffsets;
            finalizeSoft.EscapeBlockCount = escapeBlockCount;
            handle = finalizeSoft.Schedule(handle);
#endif

            handle = new PredictUnconstrainedBodiesJob
            {
                Bodies = Bodies,
                NavigationStates = NavigationStates,
                MotionIntents = MotionIntents,
                MotionEvidence = MotionEvidence,
                StepStates = StepStates,
                SoftAvoidanceResponseRate = Configuration.SoftAvoidanceResponseRate,
                SettledMultiplier = Configuration.SettledSoftAvoidanceMultiplier,
                SubstepDeltaTime = substepDeltaTime
            }.Schedule(Bodies.Length, ParallelBodyBatchSize, handle);

            handle = new ValidatePredictedContactEnvelopeBodiesJob
            {
                Bodies = Bodies,
                NavigationStates = NavigationStates,
                MotionIntents = MotionIntents,
                MotionEvidence = MotionEvidence,
                StepStates = StepStates,
                EscapeFlags = EnvelopeEscapeFlags,
                PredictiveSkin = Configuration.PredictiveSkin
            }.Schedule(Bodies.Length, ParallelBodyBatchSize, handle);

            handle = new CountP1P6EnvelopeEscapeBlocksJob
            {
                EscapeFlags = EnvelopeEscapeFlags,
                BlockOffsetsAndCounts = DirtyBodyBlockOffsets,
                BodyCount = Bodies.Length,
                Enabled = 1
            }.Schedule(escapeBlockCount, 1, handle);

            handle = new PrefixP1P6EnvelopeEscapesJob
            {
                BlockOffsetsAndCounts = DirtyBodyBlockOffsets,
                DirtyBodies = IncrementalDirtyBodies,
                DirtyFlagsByBody = IncrementalDirtyFlagsByBody,
                BlockCount = escapeBlockCount
            }.Schedule(handle);

            handle = new ScatterP1P6EnvelopeEscapesJob
            {
                EscapeFlags = EnvelopeEscapeFlags,
                BlockOffsets = DirtyBodyBlockOffsets,
                DirtyBodies = IncrementalDirtyBodies.AsDeferredJobArray(),
                DirtyFlagsByBody = IncrementalDirtyFlagsByBody,
                Bodies = Bodies,
                NavigationStates = NavigationStates,
                MotionIntents = MotionIntents,
                MotionEvidence = MotionEvidence,
                StepStates = StepStates,
                BodyStatistics = ParallelBodyStatistics,
                BodyCount = Bodies.Length
            }.Schedule(escapeBlockCount, 1, handle);

            InteractionCertificationJob finalizePrepared = Certification;
            finalizePrepared.Operation = InteractionCertificationOperation.FinalizePreparedSubstepP1P6;
            finalizePrepared.RuntimeState = runtimeState;
            finalizePrepared.SubstepIndex = substepIndex;
            handle = finalizePrepared.Schedule(handle);

            handle = new ResetContactPairStateJob
            {
                Pairs = TimestepContactPairs.AsDeferredJobArray()
            }.Schedule(TimestepContactPairs, SoftPairBatchSize, handle);

            for (int iterationIndex = 0; iterationIndex < iterationCount; iterationIndex++)
            {
                ConstraintSolverJob beginIteration = ConstraintSolver;
                beginIteration.Operation = ConstraintSolverOperation.BeginParallelIteration;
                beginIteration.RuntimeState = runtimeState;
#if RTS_CONTACT_DIAGNOSTICS
                beginIteration.IterationState = iterationState;
#endif
                beginIteration.SubstepIndex = substepIndex;
                handle = beginIteration.Schedule(handle);

                handle = new SolveWallConstraintBodiesJob
                {
                    Bodies = Bodies,
                NavigationStates = NavigationStates,
                MotionIntents = MotionIntents,
                MotionEvidence = MotionEvidence,
                StepStates = StepStates,
                    Grid = Grid,
                    GridOrigin = GridOrigin,
                    GridDimensions = GridDimensions,
                    CellRadius = CellRadius,
                    CorrectedBodyFlags = CorrectedBodyFlags,
                    BodyStatistics = ParallelBodyStatistics
                }.Schedule(Bodies.Length, ParallelBodyBatchSize, handle);

                handle = new CountAndReduceP1P6WallBlocksJob
                {
                    CorrectedBodyFlags = CorrectedBodyFlags,
                    BodyStatistics = ParallelBodyStatistics,
                    BlockOffsetsAndCounts = DirtyBodyBlockOffsets,
                    BodyCount = Bodies.Length
                }.Schedule(escapeBlockCount, 1, handle);

                handle = new PrefixP1P6CorrectedBodiesJob
                {
                    BlockOffsetsAndCounts = DirtyBodyBlockOffsets,
                    CorrectedBodyIndices = CorrectedBodyIndices,
                    BlockCount = escapeBlockCount
                }.Schedule(handle);

                handle = new ScatterP1P6CorrectedBodiesJob
                {
                    CorrectedBodyFlags = CorrectedBodyFlags,
                    BlockOffsets = DirtyBodyBlockOffsets,
                    CorrectedBodyIndices = CorrectedBodyIndices.AsDeferredJobArray(),
                    BodyCount = Bodies.Length
                }.Schedule(escapeBlockCount, 1, handle);

                InteractionCertificationJob finalizeWall = Certification;
                finalizeWall.Operation = InteractionCertificationOperation.FinalizeWallIterationP1P6;
                finalizeWall.RuntimeState = runtimeState;
#if RTS_CONTACT_DIAGNOSTICS
                finalizeWall.IterationState = iterationState;
                finalizeWall.BlockStatistics = blockStatistics;
#endif
                finalizeWall.SubstepIndex = substepIndex;
                finalizeWall.BodyBlockCount = escapeBlockCount;
                handle = finalizeWall.Schedule(handle);

#if RTS_CONTACT_DIAGNOSTICS
                if (captureSelectedPairs)
                {
                    handle = new EvaluateParallelJacobiPairsWithDiagnosticsJob
                    {
                        Alpha = Configuration.Compliance /
                                math.max(
                                    0.0000001f,
                                    substepDeltaTime * substepDeltaTime),
                        SubstepIndex = substepIndex,
                        Bodies = Bodies,
                NavigationStates = NavigationStates,
                MotionIntents = MotionIntents,
                MotionEvidence = MotionEvidence,
                StepStates = StepStates,
                        Pairs = TimestepContactPairs.AsDeferredJobArray(),
                        Corrections = JacobiPairCorrections.AsDeferredJobArray(),
                        DiagnosticPairCandidates =
                            ParallelSimulationDebuggerPairCandidates
                                .AsDeferredJobArray(),
                        DiagnosticSelectedEntity = DiagnosticSelectedEntity
                    }.Schedule(TimestepContactPairs, JacobiPairBatchSize, handle);
                }
                else
#endif
                {
                    handle = new EvaluateParallelJacobiPairsJob
                    {
                        Alpha = Configuration.Compliance /
                                math.max(
                                    0.0000001f,
                                    substepDeltaTime * substepDeltaTime),
                        SubstepIndex = substepIndex,
                        Bodies = Bodies,
                NavigationStates = NavigationStates,
                MotionIntents = MotionIntents,
                MotionEvidence = MotionEvidence,
                StepStates = StepStates,
                        Pairs = TimestepContactPairs.AsDeferredJobArray(),
                        Corrections = JacobiPairCorrections.AsDeferredJobArray()
                    }.Schedule(TimestepContactPairs, JacobiPairBatchSize, handle);
                }

#if RTS_CONTACT_DIAGNOSTICS
                handle = new ReduceParallelJacobiBlocksJob
                {
                    Corrections = JacobiPairCorrections.AsDeferredJobArray(),
                    Blocks = blockStatistics.AsDeferredJobArray()
                }.Schedule(blockStatistics, 1, handle);
#endif

#if RTS_CONTACT_DIAGNOSTICS
                if (captureSelectedPairs)
                {
                    handle = new CountParallelSimulationDebuggerPairBlocksJob
                    {
                        Candidates =
                            ParallelSimulationDebuggerPairCandidates
                                .AsDeferredJobArray(),
                        Blocks = blockStatistics.AsDeferredJobArray()
                    }.Schedule(blockStatistics, 1, handle);

                    handle = new PrefixParallelSimulationDebuggerPairsJob
                    {
                        Blocks = blockStatistics.AsDeferredJobArray(),
                        Scratch = ParallelSimulationDebuggerPairScratch
                    }.Schedule(handle);

                    handle = new ScatterParallelSimulationDebuggerPairsJob
                    {
                        Candidates =
                            ParallelSimulationDebuggerPairCandidates.AsDeferredJobArray(),
                        Blocks = blockStatistics.AsDeferredJobArray(),
                        Scratch =
                            ParallelSimulationDebuggerPairScratch.AsDeferredJobArray()
                    }.Schedule(blockStatistics, 1, handle);

                    ConstraintSolverJob mergeDebuggerPairs = ConstraintSolver;
                    mergeDebuggerPairs.Operation = ConstraintSolverOperation.MergeParallelDebuggerPairs;
                    handle = mergeDebuggerPairs.Schedule(handle);
                }
#endif

                handle = new GatherAndApplyParallelJacobiBodiesJob
                {
                    Bodies = Bodies,
                NavigationStates = NavigationStates,
                MotionIntents = MotionIntents,
                MotionEvidence = MotionEvidence,
                StepStates = StepStates,
                    Pairs = TimestepContactPairs.AsDeferredJobArray(),
                    Corrections = JacobiPairCorrections.AsDeferredJobArray(),
                    IncidentOffsets = ActiveIncidentOffsets,
                    IncidentPairIndices = ActiveIncidentPairIndices.AsDeferredJobArray(),
                    CorrectedBodyFlags = CorrectedBodyFlags
                }.Schedule(Bodies.Length, ParallelBodyBatchSize, handle);

                InteractionCertificationJob finalizeContact = Certification;
                finalizeContact.Operation = InteractionCertificationOperation.FinalizeContactIterationP1P6;
                finalizeContact.RuntimeState = runtimeState;
#if RTS_CONTACT_DIAGNOSTICS
                finalizeContact.IterationState = iterationState;
                finalizeContact.BlockStatistics = blockStatistics;
#endif
                finalizeContact.SubstepIndex = substepIndex;
                finalizeContact.IterationIndex = iterationIndex;
                handle = finalizeContact.Schedule(handle);

                ConstraintSolverJob recovery = ConstraintSolver;
                recovery.Operation = ConstraintSolverOperation.SolveParallelRecovery;
                recovery.RuntimeState = runtimeState;
                recovery.SubstepIndex = substepIndex;
                handle = recovery.Schedule(handle);
            }

#if RTS_CONTACT_DIAGNOSTICS
            ConstraintSolverJob beginFinalizeSubstep = ConstraintSolver;
            beginFinalizeSubstep.Operation = ConstraintSolverOperation.BeginParallelFinalizeSubstep;
            beginFinalizeSubstep.RuntimeState = runtimeState;
            handle = beginFinalizeSubstep.Schedule(handle);
#endif

            handle = new ReconstructVelocityBodiesJob
            {
                Bodies = Bodies,
                NavigationStates = NavigationStates,
                MotionIntents = MotionIntents,
                MotionEvidence = MotionEvidence,
                StepStates = StepStates,
                BodyStatistics = ParallelBodyStatistics,
                SubstepDeltaTime = substepDeltaTime
            }.Schedule(Bodies.Length, ParallelBodyBatchSize, handle);

#if RTS_CONTACT_DIAGNOSTICS
            handle = new ReduceP1P6VelocityBodyBlocksJob
            {
                BodyStatistics = ParallelBodyStatistics,
                BodyCount = Bodies.Length
            }.Schedule(escapeBlockCount, 1, handle);

            ConstraintSolverJob finalizeVelocity = ConstraintSolver;
            finalizeVelocity.Operation = ConstraintSolverOperation.FinalizeParallelVelocity;
            finalizeVelocity.RuntimeState = runtimeState;
            finalizeVelocity.BlockCount = escapeBlockCount;
            handle = finalizeVelocity.Schedule(handle);
#endif
        }

#if RTS_CONTACT_DIAGNOSTICS
        ConstraintSolverJob finalizePipeline = ConstraintSolver;
        finalizePipeline.Operation = ConstraintSolverOperation.FinalizeParallelPipeline;
        finalizePipeline.RuntimeState = runtimeState;
        return finalizePipeline.Schedule(handle);
#else
        return handle;
#endif
    }





    [BurstCompile]
    private struct PrepareTimestepPredictionBodiesJob : IJobParallelFor
    {
        public NativeArray<CrowdBodySnapshot> Bodies;
        public NativeArray<CrowdNavigationState> NavigationStates;
        public NativeArray<CrowdMotionIntent> MotionIntents;
        public NativeArray<CrowdMotionEvidence> MotionEvidence;
        public NativeArray<CrowdBodyStepState> StepStates;
        [NativeDisableParallelForRestriction]
        public NativeArray<PersistentSweptProxy> PersistentProxies;
        [ReadOnly] public NativeArray<int> PersistentProxyIndexByBody;
        [ReadOnly] public NativeReference<IncrementalContactCacheState> PersistentCacheState;
        public NativeArray<byte> DirtyFlagsByBody;
        public float Duration;
        public float Skin;
        public float Margin;
        public float GuardMargin;
        public float3 GridOrigin;
        public float CellRadius;
        public byte FromSolvedPosition;
        public byte DetectPersistentDirty;
        public float SoftAvoidanceShell;
        public float SoftAvoidanceResponseRate;
        public SoftAvoidanceVelocitySolverMode SoftSolverMode;
        public float RvoTimeHorizon;

        public void Execute(int bodyIndex)
        {
            CrowdBodySnapshot stateSnapshot = Bodies[bodyIndex];
            CrowdNavigationState stateNavigation = NavigationStates[bodyIndex];
            CrowdMotionIntent stateIntent = MotionIntents[bodyIndex];
            CrowdMotionEvidence stateEvidence = MotionEvidence[bodyIndex];
            CrowdBodyStepState stateStep = StepStates[bodyIndex];
            if (!(stateSnapshot.IsInsideSimulationDomain != 0))
            {
                if (DetectPersistentDirty != 0)
                    DirtyFlagsByBody[bodyIndex] = (byte)InteractionCertificationJob.ClassifyAndUpdatePersistentProxyForBodyP1P6(
                        bodyIndex, stateSnapshot, stateEvidence, stateStep,
                        PersistentProxies, PersistentProxyIndexByBody,
                        PersistentCacheState.Value, GuardMargin, SoftAvoidanceShell,
                        SoftAvoidanceResponseRate, SoftSolverMode, RvoTimeHorizon);
                return;
            }

            float3 start = FromSolvedPosition != 0
                ? stateStep.SolvedPosition
                : stateSnapshot.Position;
            float3 velocity = FromSolvedPosition != 0
                ? stateStep.BaseVelocity
                : ContactPipelineMath.CalculateBaseVelocity(stateSnapshot, stateNavigation, stateIntent, stateStep, Duration, GridOrigin, CellRadius);
            if ((stateNavigation.IsSettled != 0))
                velocity *= math.pow(0.8f, Duration * 60f);
            if (math.lengthsq(velocity) > stateSnapshot.MoveSpeed * stateSnapshot.MoveSpeed)
                velocity = math.normalizesafe(velocity) * stateSnapshot.MoveSpeed;

            float3 end = start + velocity * Duration;
            end.y = stateSnapshot.Position.y;
            float extent = math.max(0f, stateSnapshot.Radius) + math.max(0f, Skin) + math.max(0f, Margin);
            stateEvidence.TrajectoryStart = start;
            stateEvidence.BaselineEnd = end;
            stateStep.BaseVelocity = velocity;
            stateEvidence.ContactEnvelopeMin = math.min(start.xz, end.xz) - extent;
            stateEvidence.ContactEnvelopeMax = math.max(start.xz, end.xz) + extent;
            ContactPipelineMath.CalculateInteractionBounds(
                stateSnapshot,
                stateEvidence,
                stateStep,
                Skin,
                Margin,
                SoftAvoidanceShell,
                SoftAvoidanceResponseRate,
                SoftSolverMode,
                RvoTimeHorizon,
                out stateEvidence.InteractionEnvelopeMin,
                out stateEvidence.InteractionEnvelopeMax);
            if (FromSolvedPosition == 0)
                stateEvidence.EnvelopeEscaped = 0;
            Bodies[bodyIndex] = stateSnapshot;
            NavigationStates[bodyIndex] = stateNavigation;
            MotionIntents[bodyIndex] = stateIntent;
            MotionEvidence[bodyIndex] = stateEvidence;
            StepStates[bodyIndex] = stateStep;
            if (DetectPersistentDirty != 0)
                DirtyFlagsByBody[bodyIndex] = (byte)InteractionCertificationJob.ClassifyAndUpdatePersistentProxyForBodyP1P6(
                    bodyIndex, stateSnapshot, stateEvidence, stateStep,
                        PersistentProxies, PersistentProxyIndexByBody,
                    PersistentCacheState.Value, GuardMargin, SoftAvoidanceShell,
                    SoftAvoidanceResponseRate, SoftSolverMode, RvoTimeHorizon);
        }
    }

    [BurstCompile]
    private struct CountInitialP1P6DirtyBodyBlocksJob : IJobParallelFor
    {
        [ReadOnly] public NativeArray<byte> DirtyFlagsByBody;
        public NativeArray<int> BlockOffsetsAndCounts;
        public int BodyCount;
        public void Execute(int blockIndex)
        {
            int begin = blockIndex * ParallelBodyBatchSize;
            int end = math.min(begin + ParallelBodyBatchSize, BodyCount);
            int count = 0;
            for (int bodyIndex = begin; bodyIndex < end; bodyIndex++)
                count += DirtyFlagsByBody[bodyIndex] != 0 ? 1 : 0;
            BlockOffsetsAndCounts[blockIndex] = count;
        }
    }

    [BurstCompile]
    private struct PrefixInitialP1P6DirtyBodiesJob : IJob
    {
        public NativeArray<int> BlockOffsetsAndCounts;
        public NativeList<IncrementalDirtyBody> DirtyBodies;
        public int BlockCount;
        public void Execute()
        {
            int offset = 0;
            for (int blockIndex = 0; blockIndex < BlockCount; blockIndex++)
            {
                int count = BlockOffsetsAndCounts[blockIndex];
                BlockOffsetsAndCounts[blockIndex] = offset;
                offset += count;
            }
            DirtyBodies.ResizeUninitialized(offset);
        }
    }

    [BurstCompile]
    private struct ScatterInitialP1P6DirtyBodiesJob : IJobParallelFor
    {
        [ReadOnly] public NativeArray<byte> DirtyFlagsByBody;
        [ReadOnly] public NativeArray<int> BlockOffsets;
        [NativeDisableParallelForRestriction] public NativeArray<IncrementalDirtyBody> DirtyBodies;
        public int BodyCount;
        public void Execute(int blockIndex)
        {
            int begin = blockIndex * ParallelBodyBatchSize;
            int end = math.min(begin + ParallelBodyBatchSize, BodyCount);
            int writeIndex = BlockOffsets[blockIndex];
            for (int bodyIndex = begin; bodyIndex < end; bodyIndex++)
            {
                IncrementalBodyDirtyFlags flags = (IncrementalBodyDirtyFlags)DirtyFlagsByBody[bodyIndex];
                if (flags == IncrementalBodyDirtyFlags.None)
                    continue;
                DirtyBodies[writeIndex++] = new IncrementalDirtyBody { BodyIndex = bodyIndex, Flags = flags };
            }
        }
    }

    [BurstCompile]
    private struct PrepareBaseVelocityBodiesJob : IJobParallelFor
    {
        public NativeArray<CrowdBodySnapshot> Bodies;
        public NativeArray<CrowdNavigationState> NavigationStates;
        public NativeArray<CrowdMotionIntent> MotionIntents;
        public NativeArray<CrowdMotionEvidence> MotionEvidence;
        public NativeArray<CrowdBodyStepState> StepStates;
        public float SubstepDeltaTime;
        public float3 GridOrigin;
        public float CellRadius;

        public void Execute(int bodyIndex)
        {
            CrowdBodySnapshot stateSnapshot = Bodies[bodyIndex];
            CrowdNavigationState stateNavigation = NavigationStates[bodyIndex];
            CrowdMotionIntent stateIntent = MotionIntents[bodyIndex];
            CrowdMotionEvidence stateEvidence = MotionEvidence[bodyIndex];
            CrowdBodyStepState stateStep = StepStates[bodyIndex];
            if (!(stateSnapshot.IsInsideSimulationDomain != 0))
                return;
            stateStep.BaseVelocity = ContactPipelineMath.CalculateBaseVelocity(
                stateSnapshot,
                stateNavigation,
                stateIntent,
                stateStep,
                SubstepDeltaTime,
                GridOrigin,
                CellRadius);
            Bodies[bodyIndex] = stateSnapshot;
            NavigationStates[bodyIndex] = stateNavigation;
            MotionIntents[bodyIndex] = stateIntent;
            MotionEvidence[bodyIndex] = stateEvidence;
            StepStates[bodyIndex] = stateStep;
        }
    }

    [BurstCompile]
    private struct ValidateBaseMotionBodiesJob : IJobParallelFor
    {
        [ReadOnly] public NativeArray<CrowdBodySnapshot> Bodies;
        [ReadOnly] public NativeArray<CrowdNavigationState> NavigationStates;
        [ReadOnly] public NativeArray<CrowdMotionIntent> MotionIntents;
        [ReadOnly] public NativeArray<CrowdMotionEvidence> MotionEvidence;
        [ReadOnly] public NativeArray<CrowdBodyStepState> StepStates;
        public NativeArray<byte> EscapeFlags;
        public byte Enabled;
        public float PredictiveSkin;
        public float TimestepContactMargin;
        public float SoftAvoidanceShell;
        public float SoftAvoidanceResponseRate;
        public SoftAvoidanceVelocitySolverMode SoftSolverMode;
        public float RvoTimeHorizon;

        public void Execute(int bodyIndex)
        {
            if (Enabled == 0)
            {
                EscapeFlags[bodyIndex] = 0;
                return;
            }
            CrowdBodySnapshot stateSnapshot = Bodies[bodyIndex];
            CrowdNavigationState stateNavigation = NavigationStates[bodyIndex];
            CrowdMotionIntent stateIntent = MotionIntents[bodyIndex];
            CrowdMotionEvidence stateEvidence = MotionEvidence[bodyIndex];
            CrowdBodyStepState stateStep = StepStates[bodyIndex];
            if (!(stateSnapshot.IsInsideSimulationDomain != 0))
            {
                EscapeFlags[bodyIndex] = 0;
                return;
            }
            ContactPipelineMath.CalculateValidationBounds(
                stateSnapshot,
                stateEvidence,
                stateStep,
                PredictiveSkin,
                TimestepContactMargin,
                SoftAvoidanceShell,
                SoftAvoidanceResponseRate,
                SoftSolverMode,
                RvoTimeHorizon,
                out float2 min,
                out float2 max);
            EscapeFlags[bodyIndex] = (byte)(ContactPipelineMath.Contains(
                stateEvidence.InteractionEnvelopeMin,
                stateEvidence.InteractionEnvelopeMax,
                min,
                max) ? 0 : 1);
        }
    }

    [BurstCompile]
    private struct CountP1P6EnvelopeEscapeBlocksJob : IJobParallelFor
    {
        [ReadOnly] public NativeArray<byte> EscapeFlags;
        public NativeArray<int> BlockOffsetsAndCounts;
        public int BodyCount;
        public byte Enabled;

        public void Execute(int blockIndex)
        {
            int begin = blockIndex * ParallelBodyBatchSize;
            int end = math.min(begin + ParallelBodyBatchSize, BodyCount);
            int count = 0;
            for (int bodyIndex = begin; bodyIndex < end; bodyIndex++)
            {
                if (Enabled != 0 && EscapeFlags[bodyIndex] != 0)
                    count++;
            }
            BlockOffsetsAndCounts[blockIndex] = count;
        }
    }

    [BurstCompile]
    private struct PrefixP1P6EnvelopeEscapesJob : IJob
    {
        public NativeArray<int> BlockOffsetsAndCounts;
        public NativeList<IncrementalDirtyBody> DirtyBodies;
        public NativeArray<byte> DirtyFlagsByBody;
        public int BlockCount;

        public void Execute()
        {
            for (int dirtyIndex = 0; dirtyIndex < DirtyBodies.Length; dirtyIndex++)
            {
                int bodyIndex = DirtyBodies[dirtyIndex].BodyIndex;
                if ((uint)bodyIndex < (uint)DirtyFlagsByBody.Length)
                    DirtyFlagsByBody[bodyIndex] = 0;
            }

            int offset = 0;
            for (int blockIndex = 0; blockIndex < BlockCount; blockIndex++)
            {
                int count = BlockOffsetsAndCounts[blockIndex];
                BlockOffsetsAndCounts[blockIndex] = offset;
                offset += count;
            }
            DirtyBodies.ResizeUninitialized(offset);
        }
    }

    [BurstCompile]
    private struct ScatterP1P6EnvelopeEscapesJob : IJobParallelFor
    {
        [ReadOnly] public NativeArray<byte> EscapeFlags;
        [ReadOnly] public NativeArray<int> BlockOffsets;
        [NativeDisableParallelForRestriction]
        public NativeArray<IncrementalDirtyBody> DirtyBodies;
        [NativeDisableParallelForRestriction]
        public NativeArray<byte> DirtyFlagsByBody;
        [NativeDisableParallelForRestriction]
        public NativeArray<CrowdBodySnapshot> Bodies;
        public NativeArray<CrowdNavigationState> NavigationStates;
        public NativeArray<CrowdMotionIntent> MotionIntents;
        public NativeArray<CrowdMotionEvidence> MotionEvidence;
        public NativeArray<CrowdBodyStepState> StepStates;
        [NativeDisableParallelForRestriction]
        public NativeArray<ParallelBodyStageResult> BodyStatistics;
        public int BodyCount;

        public void Execute(int blockIndex)
        {
            int begin = blockIndex * ParallelBodyBatchSize;
            int end = math.min(begin + ParallelBodyBatchSize, BodyCount);
            int writeIndex = BlockOffsets[blockIndex];
            for (int bodyIndex = begin; bodyIndex < end; bodyIndex++)
            {
                if (EscapeFlags[bodyIndex] == 0)
                    continue;

                const IncrementalBodyDirtyFlags flags =
                    IncrementalBodyDirtyFlags.Motion |
                    IncrementalBodyDirtyFlags.CorrectedEscape;
                DirtyBodies[writeIndex++] = new IncrementalDirtyBody
                {
                    BodyIndex = bodyIndex,
                    Flags = flags
                };
                DirtyFlagsByBody[bodyIndex] = (byte)flags;

                CrowdBodySnapshot stateSnapshot = Bodies[bodyIndex];
                CrowdNavigationState stateNavigation = NavigationStates[bodyIndex];
                CrowdMotionIntent stateIntent = MotionIntents[bodyIndex];
                CrowdMotionEvidence stateEvidence = MotionEvidence[bodyIndex];
                CrowdBodyStepState stateStep = StepStates[bodyIndex];
                int newlyEscaped = stateEvidence.EnvelopeEscaped == 0 ? 1 : 0;
                stateEvidence.EnvelopeEscaped = 1;
                Bodies[bodyIndex] = stateSnapshot;
                NavigationStates[bodyIndex] = stateNavigation;
                MotionIntents[bodyIndex] = stateIntent;
                MotionEvidence[bodyIndex] = stateEvidence;
                StepStates[bodyIndex] = stateStep;

                ParallelBodyStageResult body = BodyStatistics[bodyIndex];
                body.EscapeCount = newlyEscaped;
                BodyStatistics[bodyIndex] = body;
            }
        }
    }



    [BurstCompile]
    private struct PrepareP1P6RepairPredictionBodiesJob : IJobParallelFor
    {
        public NativeArray<CrowdBodySnapshot> Bodies;
        public NativeArray<CrowdNavigationState> NavigationStates;
        public NativeArray<CrowdMotionIntent> MotionIntents;
        public NativeArray<CrowdMotionEvidence> MotionEvidence;
        public NativeArray<CrowdBodyStepState> StepStates;
        [ReadOnly] public NativeArray<IncrementalDirtyBody> DirtyBodies;
        public float Duration;
        public float Skin;
        public float Margin;
        public float3 GridOrigin;
        public float CellRadius;
        public float SoftAvoidanceShell;
        public float SoftAvoidanceResponseRate;
        public SoftAvoidanceVelocitySolverMode SoftSolverMode;
        public float RvoTimeHorizon;
        public byte Enabled;

        public void Execute(int dirtyIndex)
        {
            if (Enabled == 0)
                return;

            int bodyIndex = DirtyBodies[dirtyIndex].BodyIndex;
            if ((uint)bodyIndex >= (uint)Bodies.Length)
                return;
            CrowdBodySnapshot stateSnapshot = Bodies[bodyIndex];
            CrowdNavigationState stateNavigation = NavigationStates[bodyIndex];
            CrowdMotionIntent stateIntent = MotionIntents[bodyIndex];
            CrowdMotionEvidence stateEvidence = MotionEvidence[bodyIndex];
            CrowdBodyStepState stateStep = StepStates[bodyIndex];
            if (!(stateSnapshot.IsInsideSimulationDomain != 0))
                return;

            float3 start = stateStep.SolvedPosition;
            float3 velocity = stateStep.BaseVelocity;
            if ((stateNavigation.IsSettled != 0))
                velocity *= math.pow(0.8f, Duration * 60f);
            if (math.lengthsq(velocity) > stateSnapshot.MoveSpeed * stateSnapshot.MoveSpeed)
                velocity = math.normalizesafe(velocity) * stateSnapshot.MoveSpeed;

            float3 end = start + velocity * Duration;
            end.y = stateSnapshot.Position.y;
            float extent = math.max(0f, stateSnapshot.Radius) +
                           math.max(0f, Skin) + math.max(0f, Margin);
            stateEvidence.TrajectoryStart = start;
            stateEvidence.BaselineEnd = end;
            stateStep.BaseVelocity = velocity;
            stateEvidence.ContactEnvelopeMin = math.min(start.xz, end.xz) - extent;
            stateEvidence.ContactEnvelopeMax = math.max(start.xz, end.xz) + extent;
            ContactPipelineMath.CalculateInteractionBounds(
                stateSnapshot,
                stateEvidence,
                stateStep,
                Skin,
                Margin,
                SoftAvoidanceShell,
                SoftAvoidanceResponseRate,
                SoftSolverMode,
                RvoTimeHorizon,
                out stateEvidence.InteractionEnvelopeMin,
                out stateEvidence.InteractionEnvelopeMax);
            Bodies[bodyIndex] = stateSnapshot;
            NavigationStates[bodyIndex] = stateNavigation;
            MotionIntents[bodyIndex] = stateIntent;
            MotionEvidence[bodyIndex] = stateEvidence;
            StepStates[bodyIndex] = stateStep;
        }
    }







    [BurstCompile]
    private struct InitializeSoftAvoidanceBodiesJob : IJobParallelFor
    {
        public NativeArray<CrowdBodySnapshot> Bodies;
        public NativeArray<CrowdNavigationState> NavigationStates;
        public NativeArray<CrowdMotionIntent> MotionIntents;
        public NativeArray<CrowdMotionEvidence> MotionEvidence;
        public NativeArray<CrowdBodyStepState> StepStates;
        [ReadOnly] public NativeArray<FlowFieldCell> Grid;
        public float3 GridOrigin;
        public int2 GridDimensions;
        public float CellRadius;
        public float SoftShell;

        public void Execute(int bodyIndex)
        {
            CrowdBodySnapshot stateSnapshot = Bodies[bodyIndex];
            CrowdNavigationState stateNavigation = NavigationStates[bodyIndex];
            CrowdMotionIntent stateIntent = MotionIntents[bodyIndex];
            CrowdMotionEvidence stateEvidence = MotionEvidence[bodyIndex];
            CrowdBodyStepState stateStep = StepStates[bodyIndex];
            stateStep.SoftAvoidanceVelocity = float3.zero;
            stateStep.WallAvoidanceVelocity = float3.zero;
            stateStep.SoftAvoidanceNeighborCount = 0;
            if (!(stateSnapshot.IsInsideSimulationDomain != 0))
            {
                Bodies[bodyIndex] = stateSnapshot;
                NavigationStates[bodyIndex] = stateNavigation;
                MotionIntents[bodyIndex] = stateIntent;
                MotionEvidence[bodyIndex] = stateEvidence;
                StepStates[bodyIndex] = stateStep;
                return;
            }

            int2 currentCell = FlowFieldUtils.WorldToCell(
                stateStep.SolvedPosition,
                GridOrigin,
                CellRadius);
            FlowGridGeometry obstacleGeometry = new FlowGridGeometry(
                GridOrigin, GridDimensions, CellRadius);
            for (int x = -1; x <= 1; x++)
            {
                for (int y = -1; y <= 1; y++)
                {
                    int2 checkCell = currentCell + new int2(x, y);
                    if (!GridObstacleView.IsBlocked(
                            Grid, obstacleGeometry, checkCell))
                        continue;
                    float3 wallPosition = GridObstacleView.CellCenter(
                        obstacleGeometry, checkCell, stateStep.SolvedPosition.y);
                    float wallRadius = CellRadius + math.max(0f, stateSnapshot.Radius) + math.max(0f, SoftShell);
                    stateStep.WallAvoidanceVelocity += SoftAvoidanceMath.CalculateWallVelocity(
                        stateStep.SolvedPosition,
                        wallPosition,
                        stateSnapshot.MoveSpeed,
                        wallRadius);
                }
            }
            Bodies[bodyIndex] = stateSnapshot;
            NavigationStates[bodyIndex] = stateNavigation;
            MotionIntents[bodyIndex] = stateIntent;
            MotionEvidence[bodyIndex] = stateEvidence;
            StepStates[bodyIndex] = stateStep;
        }
    }

    [BurstCompile]
    private struct EvaluateSoftAvoidancePairsJob : IJobParallelForDefer
    {
        [ReadOnly] public NativeArray<CrowdBodySnapshot> Bodies;
        [ReadOnly] public NativeArray<CrowdNavigationState> NavigationStates;
        [ReadOnly] public NativeArray<CrowdMotionIntent> MotionIntents;
        [ReadOnly] public NativeArray<CrowdMotionEvidence> MotionEvidence;
        [ReadOnly] public NativeArray<CrowdBodyStepState> StepStates;
        [ReadOnly] public NativeArray<BodyPair> Pairs;
        public NativeArray<SoftAvoidancePairContribution> Contributions;
        public SoftAvoidanceVelocitySolverMode SolverMode;
        public float SoftShell;
        public float RvoTimeHorizon;
        public float SubstepDeltaTime;

        public void Execute(int pairIndex)
        {
            BodyPair pair = Pairs[pairIndex];
            CrowdBodySnapshot bodyASnapshot = Bodies[pair.BodyA];
            CrowdNavigationState bodyANavigation = NavigationStates[pair.BodyA];
            CrowdMotionIntent bodyAIntent = MotionIntents[pair.BodyA];
            CrowdMotionEvidence bodyAEvidence = MotionEvidence[pair.BodyA];
            CrowdBodyStepState bodyAStep = StepStates[pair.BodyA];
            CrowdBodySnapshot bodyBSnapshot = Bodies[pair.BodyB];
            CrowdNavigationState bodyBNavigation = NavigationStates[pair.BodyB];
            CrowdMotionIntent bodyBIntent = MotionIntents[pair.BodyB];
            CrowdMotionEvidence bodyBEvidence = MotionEvidence[pair.BodyB];
            CrowdBodyStepState bodyBStep = StepStates[pair.BodyB];
            SoftAvoidancePairContribution result = default;

            float3 delta = bodyAStep.SolvedPosition - bodyBStep.SolvedPosition;
            delta.y = 0f;
            float maxDistance = bodyASnapshot.Radius + bodyBSnapshot.Radius + math.max(0f, SoftShell);
            if (math.lengthsq(delta) <= maxDistance * maxDistance &&
                SoftAvoidanceMath.TryCalculatePairVelocities(
                    SolverMode,
                    bodyAStep.SolvedPosition,
                    bodyBStep.SolvedPosition,
                    bodyAStep.BaseVelocity,
                    bodyBStep.BaseVelocity,
                    bodyASnapshot.Radius,
                    bodyBSnapshot.Radius,
                    bodyASnapshot.InverseMass,
                    bodyBSnapshot.InverseMass,
                    bodyASnapshot.MoveSpeed,
                    bodyBSnapshot.MoveSpeed,
                    SoftShell,
                    RvoTimeHorizon,
                    SubstepDeltaTime,
                    ContactPipelineMath.DeterministicPairNormal(pair.BodyA, pair.BodyB),
                    out result.VelocityA,
                    out result.VelocityB))
            {
                result.ActiveA = 1;
                result.ActiveB = 1;
            }
            Contributions[pairIndex] = result;
        }
    }

#if RTS_CONTACT_DIAGNOSTICS
    [BurstCompile]
    private struct ReduceSoftAvoidanceBlocksJob : IJobParallelForDefer
    {
        [ReadOnly] public NativeArray<SoftAvoidancePairContribution> Contributions;
        public NativeArray<JacobiBlockTelemetry> Blocks;

        public void Execute(int blockIndex)
        {
            int begin = blockIndex * SoftPairBatchSize;
            int end = math.min(begin + SoftPairBatchSize, Contributions.Length);
            int active = 0;
            for (int i = begin; i < end; i++)
                active += Contributions[i].ActiveA != 0 ? 1 : 0;
            Blocks[blockIndex] = new JacobiBlockTelemetry
            {
                NewlyActivatedPairCount = active
            };
        }
    }

#endif

    [BurstCompile]
    private struct GatherSoftAvoidanceBodiesJob : IJobParallelFor
    {
        public NativeArray<CrowdBodySnapshot> Bodies;
        public NativeArray<CrowdNavigationState> NavigationStates;
        public NativeArray<CrowdMotionIntent> MotionIntents;
        public NativeArray<CrowdMotionEvidence> MotionEvidence;
        public NativeArray<CrowdBodyStepState> StepStates;
        [ReadOnly] public NativeArray<BodyPair> Pairs;
        [ReadOnly] public NativeArray<SoftAvoidancePairContribution> Contributions;
        [ReadOnly] public NativeArray<int> IncidentOffsets;
        [ReadOnly] public NativeArray<int> IncidentPairIndices;
        public NativeArray<byte> EscapeFlags;
        public SoftAvoidanceVelocitySolverMode SoftSolverMode;
        public float SoftAvoidanceResponseRate;
        public float SettledMultiplier;
        public float SubstepDeltaTime;
        public float PredictiveSkin;
        public float TimestepContactMargin;
        public float SoftShell;
        public byte ClampToEnvelope;

        public void Execute(int bodyIndex)
        {
            CrowdBodySnapshot stateSnapshot = Bodies[bodyIndex];
            CrowdNavigationState stateNavigation = NavigationStates[bodyIndex];
            CrowdMotionIntent stateIntent = MotionIntents[bodyIndex];
            CrowdMotionEvidence stateEvidence = MotionEvidence[bodyIndex];
            CrowdBodyStepState stateStep = StepStates[bodyIndex];
            if (!(stateSnapshot.IsInsideSimulationDomain != 0))
            {
                EscapeFlags[bodyIndex] = 0;
                return;
            }

            float3 sum = float3.zero;
            int count = 0;
            int begin = IncidentOffsets[bodyIndex];
            int end = IncidentOffsets[bodyIndex + 1];
            for (int incident = begin; incident < end; incident++)
            {
                int pairIndex = IncidentPairIndices[incident];
                BodyPair pair = Pairs[pairIndex];
                SoftAvoidancePairContribution contribution = Contributions[pairIndex];
                if (pair.BodyA == bodyIndex && contribution.ActiveA != 0)
                {
                    sum += contribution.VelocityA;
                    count++;
                }
                else if (pair.BodyB == bodyIndex && contribution.ActiveB != 0)
                {
                    sum += contribution.VelocityB;
                    count++;
                }
            }

            if (count > 0 && SoftSolverMode == SoftAvoidanceVelocitySolverMode.SurfaceVelocityBuffer)
                sum /= count;
            stateStep.SoftAvoidanceVelocity = sum + stateStep.WallAvoidanceVelocity;
            stateStep.SoftAvoidanceNeighborCount = count;
            float maxSpeed = math.max(0f, stateSnapshot.MoveSpeed);
            if (math.lengthsq(stateStep.SoftAvoidanceVelocity) > maxSpeed * maxSpeed)
                stateStep.SoftAvoidanceVelocity = math.normalizesafe(stateStep.SoftAvoidanceVelocity) * maxSpeed;

            byte escaped = 0;
            if (ClampToEnvelope != 0)
            {
                float3 requested = stateStep.SoftAvoidanceVelocity;
                if (!ContactPipelineMath.SoftOutputInsideEnvelope(
                        stateSnapshot,
                        stateNavigation,
                        stateEvidence,
                        stateStep,
                        requested,
                        SoftAvoidanceResponseRate,
                        SettledMultiplier,
                        SubstepDeltaTime,
                        PredictiveSkin,
                        TimestepContactMargin,
                        SoftShell))
                {
                    float lower = 0f;
                    float upper = 1f;
                    if (ContactPipelineMath.SoftOutputInsideEnvelope(
                            stateSnapshot,
                            stateNavigation,
                            stateEvidence,
                            stateStep,
                            float3.zero,
                            SoftAvoidanceResponseRate,
                            SettledMultiplier,
                            SubstepDeltaTime,
                            PredictiveSkin,
                            TimestepContactMargin,
                            SoftShell))
                    {
                        for (int i = 0; i < 8; i++)
                        {
                            float middle = (lower + upper) * 0.5f;
                            if (ContactPipelineMath.SoftOutputInsideEnvelope(
                                    stateSnapshot,
                                    stateNavigation,
                                    stateEvidence,
                                    stateStep,
                                    requested * middle,
                                    SoftAvoidanceResponseRate,
                                    SettledMultiplier,
                                    SubstepDeltaTime,
                                    PredictiveSkin,
                                    TimestepContactMargin,
                                    SoftShell))
                                lower = middle;
                            else
                                upper = middle;
                        }
                    }
                    stateStep.SoftAvoidanceVelocity = requested * lower;
                    escaped = 1;
                }
            }
            EscapeFlags[bodyIndex] = escaped;
            Bodies[bodyIndex] = stateSnapshot;
            NavigationStates[bodyIndex] = stateNavigation;
            MotionIntents[bodyIndex] = stateIntent;
            MotionEvidence[bodyIndex] = stateEvidence;
            StepStates[bodyIndex] = stateStep;
        }
    }

#if RTS_CONTACT_DIAGNOSTICS
    [BurstCompile]
    private struct ReduceP1P6SoftEscapeBlocksJob : IJobParallelFor
    {
        [ReadOnly] public NativeArray<byte> EscapeFlags;
        public NativeArray<int> EscapeCountsByBlock;
        public int BodyCount;

        public void Execute(int blockIndex)
        {
            int begin = blockIndex * ParallelBodyBatchSize;
            int end = math.min(begin + ParallelBodyBatchSize, BodyCount);
            int escaped = 0;
            for (int bodyIndex = begin; bodyIndex < end; bodyIndex++)
                escaped += EscapeFlags[bodyIndex] != 0 ? 1 : 0;
            EscapeCountsByBlock[blockIndex] = escaped;
        }
    }



#endif

    [BurstCompile]
    private struct PredictUnconstrainedBodiesJob : IJobParallelFor
    {
        public NativeArray<CrowdBodySnapshot> Bodies;
        public NativeArray<CrowdNavigationState> NavigationStates;
        public NativeArray<CrowdMotionIntent> MotionIntents;
        public NativeArray<CrowdMotionEvidence> MotionEvidence;
        public NativeArray<CrowdBodyStepState> StepStates;
        public float SoftAvoidanceResponseRate;
        public float SettledMultiplier;
        public float SubstepDeltaTime;

        public void Execute(int bodyIndex)
        {
            CrowdBodySnapshot stateSnapshot = Bodies[bodyIndex];
            CrowdNavigationState stateNavigation = NavigationStates[bodyIndex];
            CrowdMotionIntent stateIntent = MotionIntents[bodyIndex];
            CrowdMotionEvidence stateEvidence = MotionEvidence[bodyIndex];
            CrowdBodyStepState stateStep = StepStates[bodyIndex];
            if (!(stateSnapshot.IsInsideSimulationDomain != 0))
                return;
            stateStep.SubstepStartPosition = stateStep.SolvedPosition;
            stateStep.PreviousSubstepPosition = stateStep.SubstepStartPosition;
            stateStep.ContactCorrection = float3.zero;
            stateStep.WallCorrection = float3.zero;
            float response = math.max(0f, SoftAvoidanceResponseRate);
            if ((stateNavigation.IsSettled != 0))
                response *= math.max(0f, SettledMultiplier);
            float3 velocity = SoftAvoidanceMath.ApplyVelocityBuffer(
                stateStep.BaseVelocity,
                stateStep.SoftAvoidanceVelocity,
                response,
                SubstepDeltaTime,
                stateSnapshot.MoveSpeed);
            if ((stateNavigation.IsSettled != 0))
                velocity *= math.pow(0.8f, SubstepDeltaTime * 60f);
            if (math.lengthsq(velocity) > stateSnapshot.MoveSpeed * stateSnapshot.MoveSpeed)
                velocity = math.normalizesafe(velocity) * stateSnapshot.MoveSpeed;
            stateStep.SolvedPosition = stateStep.SubstepStartPosition + velocity * SubstepDeltaTime;
            stateStep.SolvedPosition.y = stateSnapshot.Position.y;
            stateStep.UnconstrainedPosition = stateStep.SolvedPosition;
            stateStep.VelocityBeforeContact = velocity;
            stateStep.IntegratedVelocity = velocity;
            Bodies[bodyIndex] = stateSnapshot;
            NavigationStates[bodyIndex] = stateNavigation;
            MotionIntents[bodyIndex] = stateIntent;
            MotionEvidence[bodyIndex] = stateEvidence;
            StepStates[bodyIndex] = stateStep;
        }
    }

    [BurstCompile]
    private struct ValidatePredictedContactEnvelopeBodiesJob : IJobParallelFor
    {
        [ReadOnly] public NativeArray<CrowdBodySnapshot> Bodies;
        [ReadOnly] public NativeArray<CrowdNavigationState> NavigationStates;
        [ReadOnly] public NativeArray<CrowdMotionIntent> MotionIntents;
        [ReadOnly] public NativeArray<CrowdMotionEvidence> MotionEvidence;
        [ReadOnly] public NativeArray<CrowdBodyStepState> StepStates;
        public NativeArray<byte> EscapeFlags;
        public float PredictiveSkin;

        public void Execute(int bodyIndex)
        {
            CrowdBodySnapshot stateSnapshot = Bodies[bodyIndex];
            CrowdNavigationState stateNavigation = NavigationStates[bodyIndex];
            CrowdMotionIntent stateIntent = MotionIntents[bodyIndex];
            CrowdMotionEvidence stateEvidence = MotionEvidence[bodyIndex];
            CrowdBodyStepState stateStep = StepStates[bodyIndex];
            if (!(stateSnapshot.IsInsideSimulationDomain != 0))
            {
                EscapeFlags[bodyIndex] = 0;
                return;
            }
            float extent = math.max(0f, stateSnapshot.Radius) + math.max(0f, PredictiveSkin);
            EscapeFlags[bodyIndex] = (byte)(ContactPipelineMath.Contains(
                stateEvidence.ContactEnvelopeMin,
                stateEvidence.ContactEnvelopeMax,
                stateStep.SolvedPosition.xz - extent,
                stateStep.SolvedPosition.xz + extent) ? 0 : 1);
        }
    }



    [BurstCompile]
    private struct ResetContactPairStateJob : IJobParallelForDefer
    {
        public NativeArray<ContactConstraint> Pairs;
        public void Execute(int pairIndex)
        {
            ContactConstraint pair = Pairs[pairIndex];
            pair.Lambda = 0f;
            pair.WasActivated = 0;
            Pairs[pairIndex] = pair;
        }
    }

#if RTS_CONTACT_DIAGNOSTICS
    [BurstCompile]
    private struct CountParallelSimulationDebuggerPairBlocksJob :
        IJobParallelForDefer
    {
        [ReadOnly] public NativeArray<ParallelSimulationDebuggerPairCapture>
            Candidates;
        public NativeArray<JacobiBlockTelemetry> Blocks;

        public void Execute(int blockIndex)
        {
            int begin = blockIndex * JacobiPairBatchSize;
            int end = math.min(begin + JacobiPairBatchSize, Candidates.Length);
            int selectedPairCount = 0;
            for (int pairIndex = begin; pairIndex < end; pairIndex++)
            {
                selectedPairCount += Candidates[pairIndex].IsValid != 0 ? 1 : 0;
            }

            JacobiBlockTelemetry block = Blocks[blockIndex];
            block.SelectedPairCount = selectedPairCount;
            Blocks[blockIndex] = block;
        }
    }

    [BurstCompile]
    private struct PrefixParallelSimulationDebuggerPairsJob : IJob
    {
        public NativeArray<JacobiBlockTelemetry> Blocks;
        public NativeList<SimulationDebuggerPairSample> Scratch;

        public void Execute()
        {
            int offset = 0;
            for (int blockIndex = 0; blockIndex < Blocks.Length; blockIndex++)
            {
                JacobiBlockTelemetry block = Blocks[blockIndex];
                block.SelectedPairOffset = offset;
                offset += block.SelectedPairCount;
                Blocks[blockIndex] = block;
            }
            Scratch.ResizeUninitialized(offset);
        }
    }

    [BurstCompile]
    private struct ScatterParallelSimulationDebuggerPairsJob : IJobParallelForDefer
    {
        [ReadOnly] public NativeArray<ParallelSimulationDebuggerPairCapture>
            Candidates;
        [ReadOnly] public NativeArray<JacobiBlockTelemetry> Blocks;
        [NativeDisableParallelForRestriction]
        public NativeArray<SimulationDebuggerPairSample> Scratch;

        public void Execute(int blockIndex)
        {
            int begin = blockIndex * JacobiPairBatchSize;
            int end = math.min(begin + JacobiPairBatchSize, Candidates.Length);
            int writeIndex = Blocks[blockIndex].SelectedPairOffset;
            for (int pairIndex = begin; pairIndex < end; pairIndex++)
            {
                ParallelSimulationDebuggerPairCapture candidate =
                    Candidates[pairIndex];
                if (candidate.IsValid == 0)
                    continue;
                Scratch[writeIndex++] = candidate.Sample;
            }
        }
    }



#endif



    [BurstCompile]
    private struct SolveWallConstraintBodiesJob : IJobParallelFor
    {
        public NativeArray<CrowdBodySnapshot> Bodies;
        public NativeArray<CrowdNavigationState> NavigationStates;
        public NativeArray<CrowdMotionIntent> MotionIntents;
        public NativeArray<CrowdMotionEvidence> MotionEvidence;
        public NativeArray<CrowdBodyStepState> StepStates;
        [ReadOnly] public NativeArray<FlowFieldCell> Grid;
        public float3 GridOrigin;
        public int2 GridDimensions;
        public float CellRadius;
        public NativeArray<byte> CorrectedBodyFlags;
        public NativeArray<ParallelBodyStageResult> BodyStatistics;

        public void Execute(int bodyIndex)
        {
            CrowdBodySnapshot stateSnapshot = Bodies[bodyIndex];
            CrowdNavigationState stateNavigation = NavigationStates[bodyIndex];
            CrowdMotionIntent stateIntent = MotionIntents[bodyIndex];
            CrowdMotionEvidence stateEvidence = MotionEvidence[bodyIndex];
            CrowdBodyStepState stateStep = StepStates[bodyIndex];
            float total = 0f;
            float maximum = 0f;
            int corrected = 0;
            if (Grid.IsCreated && (stateSnapshot.IsInsideSimulationDomain != 0) && stateSnapshot.InverseMass > 0f)
            {
                int2 currentCell = FlowFieldUtils.WorldToCell(
                    stateStep.SolvedPosition,
                    GridOrigin,
                    CellRadius);
                FlowGridGeometry obstacleGeometry = new FlowGridGeometry(
                    GridOrigin, GridDimensions, CellRadius);
                for (int x = -1; x <= 1; x++)
                {
                    for (int y = -1; y <= 1; y++)
                    {
                        int2 checkCell = currentCell + new int2(x, y);
                        if (!GridObstacleView.IsBlocked(
                                Grid, obstacleGeometry, checkCell))
                            continue;
                        float3 wallPosition = GridObstacleView.CellCenter(
                            obstacleGeometry, checkCell, stateStep.SolvedPosition.y);
                        float3 delta = stateStep.SolvedPosition - wallPosition;
                        delta.y = 0f;
                        float distance = math.length(delta);
                        float hardDistance = CellRadius + math.max(0f, stateSnapshot.Radius);
                        if (distance >= hardDistance)
                            continue;
                        float3 normal = distance > 0.00001f
                            ? delta / distance
                            : ContactPipelineMath.DeterministicPairNormal(bodyIndex, checkIndex);
                        float3 correction = normal * ((hardDistance - distance) * 0.5f);
                        stateStep.SolvedPosition += correction;
                        stateStep.SolvedPosition.y = stateSnapshot.Position.y;
                        stateStep.WallCorrection += correction;
                        stateEvidence.WallCorrection += correction;
                        float length = math.length(correction);
                        total += length;
                        maximum = math.max(maximum, length);
                        corrected = 1;
                    }
                }
            }
            Bodies[bodyIndex] = stateSnapshot;
            NavigationStates[bodyIndex] = stateNavigation;
            MotionIntents[bodyIndex] = stateIntent;
            MotionEvidence[bodyIndex] = stateEvidence;
            StepStates[bodyIndex] = stateStep;
            CorrectedBodyFlags[bodyIndex] = (byte)corrected;
            BodyStatistics[bodyIndex] = new ParallelBodyStageResult
            {
                Total = total,
                Maximum = maximum,
                Count = corrected
            };
        }
    }

    [BurstCompile]
    private struct CountAndReduceP1P6WallBlocksJob : IJobParallelFor
    {
        [ReadOnly] public NativeArray<byte> CorrectedBodyFlags;
        [NativeDisableParallelForRestriction]
        public NativeArray<ParallelBodyStageResult> BodyStatistics;
        public NativeArray<int> BlockOffsetsAndCounts;
        public int BodyCount;

        public void Execute(int blockIndex)
        {
            int begin = blockIndex * ParallelBodyBatchSize;
            int end = math.min(begin + ParallelBodyBatchSize, BodyCount);
            int correctedCount = 0;
            ParallelBodyStageResult aggregate = default;
            for (int bodyIndex = begin; bodyIndex < end; bodyIndex++)
            {
                ParallelBodyStageResult body = BodyStatistics[bodyIndex];
                aggregate.Total += body.Total;
                aggregate.Maximum = math.max(aggregate.Maximum, body.Maximum);
                correctedCount += CorrectedBodyFlags[bodyIndex] != 0 ? 1 : 0;
            }

            BlockOffsetsAndCounts[blockIndex] = correctedCount;
            if (begin < BodyCount)
                BodyStatistics[begin] = aggregate;
        }
    }

    [BurstCompile]
    private struct PrefixP1P6CorrectedBodiesJob : IJob
    {
        public NativeArray<int> BlockOffsetsAndCounts;
        public NativeList<int> CorrectedBodyIndices;
        public int BlockCount;

        public void Execute()
        {
            int offset = 0;
            for (int blockIndex = 0; blockIndex < BlockCount; blockIndex++)
            {
                int count = BlockOffsetsAndCounts[blockIndex];
                BlockOffsetsAndCounts[blockIndex] = offset;
                offset += count;
            }
            CorrectedBodyIndices.ResizeUninitialized(offset);
        }
    }

    [BurstCompile]
    private struct ScatterP1P6CorrectedBodiesJob : IJobParallelFor
    {
        [ReadOnly] public NativeArray<byte> CorrectedBodyFlags;
        [ReadOnly] public NativeArray<int> BlockOffsets;
        [NativeDisableParallelForRestriction]
        public NativeArray<int> CorrectedBodyIndices;
        public int BodyCount;

        public void Execute(int blockIndex)
        {
            int begin = blockIndex * ParallelBodyBatchSize;
            int end = math.min(begin + ParallelBodyBatchSize, BodyCount);
            int writeIndex = BlockOffsets[blockIndex];
            for (int bodyIndex = begin; bodyIndex < end; bodyIndex++)
            {
                if (CorrectedBodyFlags[bodyIndex] != 0)
                    CorrectedBodyIndices[writeIndex++] = bodyIndex;
            }
        }
    }



#if RTS_CONTACT_DIAGNOSTICS


#endif

    [BurstCompile]
    private struct ReconstructVelocityBodiesJob : IJobParallelFor
    {
        public NativeArray<CrowdBodySnapshot> Bodies;
        public NativeArray<CrowdNavigationState> NavigationStates;
        public NativeArray<CrowdMotionIntent> MotionIntents;
        public NativeArray<CrowdMotionEvidence> MotionEvidence;
        public NativeArray<CrowdBodyStepState> StepStates;
        public NativeArray<ParallelBodyStageResult> BodyStatistics;
        public float SubstepDeltaTime;

        public void Execute(int bodyIndex)
        {
            CrowdBodySnapshot stateSnapshot = Bodies[bodyIndex];
            CrowdNavigationState stateNavigation = NavigationStates[bodyIndex];
            CrowdMotionIntent stateIntent = MotionIntents[bodyIndex];
            CrowdMotionEvidence stateEvidence = MotionEvidence[bodyIndex];
            CrowdBodyStepState stateStep = StepStates[bodyIndex];
            if (!(stateSnapshot.IsInsideSimulationDomain != 0))
            {
                BodyStatistics[bodyIndex] = default;
                return;
            }
            stateStep.IntegratedVelocity =
                (stateStep.SolvedPosition - stateStep.PreviousSubstepPosition) / SubstepDeltaTime;
            stateStep.IntegratedVelocity.y = 0f;
            float change = math.distance(stateStep.IntegratedVelocity, stateStep.VelocityBeforeContact);
            BodyStatistics[bodyIndex] = new ParallelBodyStageResult
            {
                Total = change,
                Maximum = change,
                SecondaryTotal = math.length(stateStep.VelocityBeforeContact),
                TertiaryTotal = math.length(stateStep.IntegratedVelocity),
                Count = 1
            };
            Bodies[bodyIndex] = stateSnapshot;
            NavigationStates[bodyIndex] = stateNavigation;
            MotionIntents[bodyIndex] = stateIntent;
            MotionEvidence[bodyIndex] = stateEvidence;
            StepStates[bodyIndex] = stateStep;
        }
    }

#if RTS_CONTACT_DIAGNOSTICS
    [BurstCompile]
    private struct ReduceP1P6VelocityBodyBlocksJob : IJobParallelFor
    {
        [NativeDisableParallelForRestriction]
        public NativeArray<ParallelBodyStageResult> BodyStatistics;
        public int BodyCount;

        public void Execute(int blockIndex)
        {
            int begin = blockIndex * ParallelBodyBatchSize;
            int end = math.min(begin + ParallelBodyBatchSize, BodyCount);
            ParallelBodyStageResult aggregate = default;
            for (int bodyIndex = begin; bodyIndex < end; bodyIndex++)
            {
                ParallelBodyStageResult body = BodyStatistics[bodyIndex];
                aggregate.Total += body.Total;
                aggregate.Maximum = math.max(aggregate.Maximum, body.Maximum);
                aggregate.SecondaryTotal += body.SecondaryTotal;
                aggregate.TertiaryTotal += body.TertiaryTotal;
                aggregate.Count += body.Count;
            }
            if (begin < BodyCount)
                BodyStatistics[begin] = aggregate;
        }
    }



#endif





















#if RTS_CONTACT_DIAGNOSTICS


#endif







#if RTS_CONTACT_DIAGNOSTICS




#endif
























}
}
