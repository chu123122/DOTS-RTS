using System.Collections.Generic;
using Unity.Collections;
using RTS.Unit.FlowField;

namespace RTS.Unit.FlowField.Diagnostics
{
public static class IncrementalContactPipelineExperimentRuntime
{
#if RTS_CONTACT_DIAGNOSTICS
    private sealed class OverrideState
    {
        public bool OverrideEnabled;
        public bool TimestepCacheEnabled=true;
        public bool CrossFrameContactCacheEnabled=true;
        public bool PredictiveContactsEnabled=true;
        public bool DiagnosticsEnabled=true;
        public int SubstepCount=4;
        public int IterationCount=4;
        public ContactPositionSolverMode ContactPositionSolver=ContactPositionSolverMode.GaussSeidel;
        public float GuardEnvelopeMargin=0.5f;
        public float PredictiveSkin=0.05f;
        public float TimestepContactMargin=0.02f;
        public string ExperimentId="manual";
        public string Scenario="unspecified";
        public string ConfigurationLabel="runtime";
    }
    private static readonly object Gate=new object();
    private static readonly Dictionary<ulong,OverrideState> Worlds=new Dictionary<ulong,OverrideState>();
    private static OverrideState GetLocked(ulong worldId)
    {
        if(!Worlds.TryGetValue(worldId,out OverrideState state))
        {
            state=new OverrideState();
            Worlds.Add(worldId,state);
        }
        return state;
    }
    private static ulong Target=>SimulationDebuggerRuntime.TargetWorldId;
    public static void RegisterWorld(ulong worldId){if(worldId!=0)lock(Gate)GetLocked(worldId);}
    public static void UnregisterWorld(ulong worldId){lock(Gate)Worlds.Remove(worldId);}
    public static bool OverrideEnabled {get{lock(Gate)return GetLocked(Target).OverrideEnabled;}set{lock(Gate)GetLocked(Target).OverrideEnabled=value;}}
    public static bool TimestepCacheEnabled {get{lock(Gate)return GetLocked(Target).TimestepCacheEnabled;}set{lock(Gate)GetLocked(Target).TimestepCacheEnabled=value;}}
    public static bool CrossFrameContactCacheEnabled {get{lock(Gate)return GetLocked(Target).CrossFrameContactCacheEnabled;}set{lock(Gate)GetLocked(Target).CrossFrameContactCacheEnabled=value;}}
    public static bool PredictiveContactsEnabled {get{lock(Gate)return GetLocked(Target).PredictiveContactsEnabled;}set{lock(Gate)GetLocked(Target).PredictiveContactsEnabled=value;}}
    public static bool DiagnosticsEnabled {get{lock(Gate)return GetLocked(Target).DiagnosticsEnabled;}set{lock(Gate)GetLocked(Target).DiagnosticsEnabled=value;}}
    public static int SubstepCount {get{lock(Gate)return GetLocked(Target).SubstepCount;}set{lock(Gate)GetLocked(Target).SubstepCount=value;}}
    public static int IterationCount {get{lock(Gate)return GetLocked(Target).IterationCount;}set{lock(Gate)GetLocked(Target).IterationCount=value;}}
    public static ContactPositionSolverMode ContactPositionSolver {get{lock(Gate)return GetLocked(Target).ContactPositionSolver;}set{lock(Gate)GetLocked(Target).ContactPositionSolver=value;}}
    public static float GuardEnvelopeMargin {get{lock(Gate)return GetLocked(Target).GuardEnvelopeMargin;}set{lock(Gate)GetLocked(Target).GuardEnvelopeMargin=value;}}
    public static float PredictiveSkin {get{lock(Gate)return GetLocked(Target).PredictiveSkin;}set{lock(Gate)GetLocked(Target).PredictiveSkin=value;}}
    public static float TimestepContactMargin {get{lock(Gate)return GetLocked(Target).TimestepContactMargin;}set{lock(Gate)GetLocked(Target).TimestepContactMargin=value;}}
    public static string ExperimentId {get{lock(Gate)return GetLocked(Target).ExperimentId;}set{lock(Gate)GetLocked(Target).ExperimentId=value;}}
    public static string Scenario {get{lock(Gate)return GetLocked(Target).Scenario;}set{lock(Gate)GetLocked(Target).Scenario=value;}}
    public static string ConfigurationLabel {get{lock(Gate)return GetLocked(Target).ConfigurationLabel;}set{lock(Gate)GetLocked(Target).ConfigurationLabel=value;}}
    public static bool OverrideEnabledFor(ulong worldId){lock(Gate)return GetLocked(worldId).OverrideEnabled;}
    public static bool CrossFrameContactCacheEnabledFor(ulong worldId){lock(Gate)return GetLocked(worldId).CrossFrameContactCacheEnabled;}

    public static void Apply(ulong worldId,ref UnitContactSolverSettings settings)
    {
        OverrideState state;
        lock(Gate) state=GetLocked(worldId);
        if(!state.OverrideEnabled)return;
        settings.SubstepCount=state.SubstepCount<1?1:state.SubstepCount;
        settings.IterationCount=state.IterationCount<1?1:state.IterationCount;
        settings.ContactPositionSolver=state.ContactPositionSolver;
        settings.PersistentGuardEnvelopeMargin=state.GuardEnvelopeMargin<0f?0f:state.GuardEnvelopeMargin;
        settings.PredictiveSkin=state.PredictiveSkin<0f?0f:state.PredictiveSkin;
        settings.TimestepContactMargin=state.TimestepContactMargin<0f?0f:state.TimestepContactMargin;
        settings.EnablePredictiveContacts=state.PredictiveContactsEnabled;
        settings.EnableTimestepContactSetCache=state.TimestepCacheEnabled;
        settings.EnablePersistentContactCache=state.CrossFrameContactCacheEnabled&&state.TimestepCacheEnabled;
        settings.EnableDiagnostics=state.DiagnosticsEnabled;
    }
    public static void Apply(ref UnitContactSolverSettings settings)=>Apply(Target,ref settings);

    public static IncrementalContactPipelineConfiguration CaptureConfiguration(
        ulong worldId,int unitCount,float deltaTime,float softAvoidanceShell,
        UnitContactSolverSettings settings,bool effectiveTimestepCacheEnabled,
        bool effectiveCrossFrameTopologyEnabled)
    {
        OverrideState state;
        lock(Gate) state=GetLocked(worldId);
        return new IncrementalContactPipelineConfiguration
        {
            ExperimentId=ToFixedString(state.ExperimentId,"manual"),
            Scenario=ToFixedString(state.Scenario,"unspecified"),
            ConfigurationLabel=ToFixedString(state.ConfigurationLabel,"runtime"),
            UnitCount=unitCount,SubstepCount=settings.SubstepCount,
            IterationCount=settings.IterationCount,
            ContactPositionSolver=(byte)settings.ContactPositionSolver,
            DeltaTime=deltaTime,GuardEnvelopeMargin=settings.PersistentGuardEnvelopeMargin,
            PredictiveSkin=settings.PredictiveSkin,
            TimestepContactMargin=settings.TimestepContactMargin,
            SoftAvoidanceShell=softAvoidanceShell,
            TimestepCacheEnabled=(byte)(effectiveTimestepCacheEnabled?1:0),
            CrossFrameTopologyEnabled=(byte)(effectiveCrossFrameTopologyEnabled?1:0),
            PredictiveContactsEnabled=(byte)(settings.EnablePredictiveContacts?1:0),
            DiagnosticsEnabled=(byte)(settings.EnableDiagnostics?1:0)
        };
    }
    public static IncrementalContactPipelineConfiguration CaptureConfiguration(
        int unitCount,float deltaTime,float softAvoidanceShell,
        UnitContactSolverSettings settings,bool effectiveTimestepCacheEnabled,
        bool effectiveCrossFrameTopologyEnabled)=>CaptureConfiguration(
            Target,unitCount,deltaTime,softAvoidanceShell,settings,
            effectiveTimestepCacheEnabled,effectiveCrossFrameTopologyEnabled);
    private static FixedString64Bytes ToFixedString(string value,string fallback)
    {
        string resolved=string.IsNullOrWhiteSpace(value)?fallback:value.Trim();
        return new FixedString64Bytes(resolved);
    }
#else
    public static void RegisterWorld(ulong worldId){ }
    public static void UnregisterWorld(ulong worldId){ }
    public static bool OverrideEnabled {get=>false;set{}}
    public static bool TimestepCacheEnabled {get=>true;set{}}
    public static bool CrossFrameContactCacheEnabled {get=>true;set{}}
    public static bool PredictiveContactsEnabled {get=>true;set{}}
    public static bool DiagnosticsEnabled {get=>false;set{}}
    public static int SubstepCount {get=>4;set{}}
    public static int IterationCount {get=>4;set{}}
    public static ContactPositionSolverMode ContactPositionSolver {get=>ContactPositionSolverMode.GaussSeidel;set{}}
    public static float GuardEnvelopeMargin {get=>0.5f;set{}}
    public static float PredictiveSkin {get=>0.05f;set{}}
    public static float TimestepContactMargin {get=>0.02f;set{}}
    public static string ExperimentId {get=>"disabled";set{}}
    public static string Scenario {get=>"disabled";set{}}
    public static string ConfigurationLabel {get=>"gameplay";set{}}
    public static bool OverrideEnabledFor(ulong worldId)=>false;
    public static bool CrossFrameContactCacheEnabledFor(ulong worldId)=>true;
    public static void Apply(ulong worldId,ref UnitContactSolverSettings settings){ }
    public static void Apply(ref UnitContactSolverSettings settings){ }
    public static IncrementalContactPipelineConfiguration CaptureConfiguration(ulong worldId,int unitCount,float deltaTime,float softAvoidanceShell,UnitContactSolverSettings settings,bool effectiveTimestepCacheEnabled,bool effectiveCrossFrameTopologyEnabled)=>default;
    public static IncrementalContactPipelineConfiguration CaptureConfiguration(int unitCount,float deltaTime,float softAvoidanceShell,UnitContactSolverSettings settings,bool effectiveTimestepCacheEnabled,bool effectiveCrossFrameTopologyEnabled)=>default;
#endif
}
}
