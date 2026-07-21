using RTS.Unit.Components;
using RTS.Unit.FlowField;
using RTS.Unit.FlowField.Jobs;
using RTS.Unit.FlowField.Systems;

namespace RTS.Unit.FlowField.Diagnostics
{
public sealed class Stage3ContactDiagnosticAutoCapture
{
    public const int DefaultRoundCount = 3;
    public const int RunsPerRound = 3;
    public const float InitialWarmupSeconds = 3f;
    public const float TransitionWarmupSeconds = 2f;
    public const float ExperimentPredictiveSkin = 0.05f;
    public const float ExperimentFatAabbMargin = 0.25f;
    public const float ExperimentCaptureDuration = 10f;
    public const float ExperimentCaptureInterval = 0.1f;

    private UnitContactSolverSettings _originalSettings;
    private int _roundCount;
    private int _runIndex;
    private bool _waitingForCaptureCompletion;
    private double _nextStartTime;

    public bool Active { get; private set; }
    public int TotalRuns => _roundCount * RunsPerRound;
    public int CurrentRunNumber => Active ? _runIndex + 1 : 0;
    public string CurrentRunLabel => Active ? BuildRunLabel(_runIndex) : string.Empty;

    public void Start(
        ref UnitContactSolverSettings settings,
        double simulationTime,
        int roundCount = DefaultRoundCount)
    {
        _originalSettings = settings;
        _roundCount = roundCount > 0 ? roundCount : 1;
        _runIndex = 0;
        _waitingForCaptureCompletion = false;
        _nextStartTime = simulationTime + InitialWarmupSeconds;
        Active = true;
        ApplyExperimentSettings(ref settings, _runIndex);
    }

    public bool Tick(
        ref UnitContactSolverSettings settings,
        double simulationTime,
        bool captureActive,
        out string runLabel,
        out bool completed)
    {
        runLabel = string.Empty;
        completed = false;
        if (!Active)
            return false;

        ApplyExperimentSettings(ref settings, _runIndex);

        if (_waitingForCaptureCompletion)
        {
            if (captureActive)
                return false;

            _waitingForCaptureCompletion = false;
            _runIndex++;
            if (_runIndex >= TotalRuns)
            {
                RestoreOriginalSettings(ref settings);
                completed = true;
                return false;
            }

            ApplyExperimentSettings(ref settings, _runIndex);
            _nextStartTime = simulationTime + TransitionWarmupSeconds;
            return false;
        }

        if (captureActive || simulationTime < _nextStartTime)
            return false;

        runLabel = BuildRunLabel(_runIndex);
        _waitingForCaptureCompletion = true;
        return true;
    }

    public void Cancel(ref UnitContactSolverSettings settings)
    {
        if (!Active)
            return;

        RestoreOriginalSettings(ref settings);
    }

    public float GetWarmupRemaining(double simulationTime)
    {
        if (!Active || _waitingForCaptureCompletion)
            return 0f;
        return (float)System.Math.Max(0d, _nextStartTime - simulationTime);
    }

    public static bool IsCacheEnabledForRun(int runIndex)
    {
        return runIndex % RunsPerRound == 1;
    }

    public static int GetSubstepsForRun(int runIndex)
    {
        return (runIndex / RunsPerRound) switch
        {
            0 => 1,
            1 => 2,
            _ => 4
        };
    }

    public static int GetIterationsForRun(int runIndex)
    {
        return (runIndex / RunsPerRound) switch
        {
            0 => 8,
            1 => 4,
            _ => 2
        };
    }

    public static string BuildRunLabel(int runIndex)
    {
        int round = runIndex / RunsPerRound + 1;
        int substeps = GetSubstepsForRun(runIndex);
        int iterations = GetIterationsForRun(runIndex);
        string phase = (runIndex % RunsPerRound) switch
        {
            0 => "off-before",
            1 => "on",
            _ => "off-after"
        };
        return $"fat-aabb-r{round:00}-s{substeps}-i{iterations}-{phase}";
    }

    private static void ApplyExperimentSettings(
        ref UnitContactSolverSettings settings,
        int runIndex)
    {
        settings.SubstepCount = GetSubstepsForRun(runIndex);
        settings.IterationCount = GetIterationsForRun(runIndex);
        settings.PredictiveSkin = ExperimentPredictiveSkin;
        settings.EnablePredictivePairGeneration = true;
        settings.EnablePredictiveContacts = true;
        settings.EnableDiagnostics = true;
        settings.DiagnosticCaptureDuration = ExperimentCaptureDuration;
        settings.DiagnosticCaptureInterval = ExperimentCaptureInterval;
        settings.EnableFatAabbCache = IsCacheEnabledForRun(runIndex);
        settings.FatAabbCacheMargin = ExperimentFatAabbMargin;
    }

    private void RestoreOriginalSettings(ref UnitContactSolverSettings settings)
    {
        settings = _originalSettings;
        Active = false;
        _roundCount = 0;
        _runIndex = 0;
        _waitingForCaptureCompletion = false;
        _nextStartTime = 0d;
    }
}
}
