#if UNITY_EDITOR
using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Text;
using UnityEditor;
using UnityEditor.Profiling;
using UnityEditorInternal;
using UnityEngine;

namespace RTS.Unit.FlowField.Editor
{
[InitializeOnLoad]
internal static class CodexProfilerDataExporter
{
    private const string InputPath =
        @"E:\WorkAndStudy\GameAndUnityPackage\RTS_Replay_2\Record\1.data";
    private const string OutputDirectory =
        @"E:\WorkAndStudy\GameAndUnityPackage\RTS_Replay_2\Record\1_codex_analysis";

    private static readonly string[] RelevantTokens =
    {
        "Jacobi",
        "SolveXpbdUnitContactsJob",
        "JobHandle.Complete",
        "WaitForJobGroup",
        "FlowMovement",
        "ContactPipeline"
    };

    static CodexProfilerDataExporter()
    {
        EditorApplication.delayCall += ExportOnce;
    }

    private static void ExportOnce()
    {
        string completionPath = Path.Combine(OutputDirectory, "complete.txt");
        if (File.Exists(completionPath) || !File.Exists(InputPath))
            return;

        try
        {
            Directory.CreateDirectory(OutputDirectory);
            if (!ProfilerDriver.LoadProfile(InputPath, false))
                throw new InvalidOperationException("ProfilerDriver.LoadProfile returned false.");

            int firstFrame = Math.Max(0, ProfilerDriver.firstFrameIndex);
            int lastFrame = ProfilerDriver.lastFrameIndex;
            if (lastFrame < firstFrame)
                throw new InvalidOperationException(
                    $"Invalid Profiler frame range: {firstFrame}..{lastFrame}");

            var frames = new Dictionary<int, FrameMetrics>();
            var markerTotals = new Dictionary<string, MarkerMetrics>(StringComparer.Ordinal);
            var threadTotals = new Dictionary<string, ThreadMetrics>(StringComparer.Ordinal);
            var rawRows = new List<RawRow>(8192);

            for (int frameIndex = firstFrame; frameIndex <= lastFrame; frameIndex++)
            {
                var frameMetrics = new FrameMetrics(frameIndex);
                frames.Add(frameIndex, frameMetrics);

                for (int threadIndex = 0;; threadIndex++)
                {
                    using (RawFrameDataView frameData =
                           ProfilerDriver.GetRawFrameDataView(frameIndex, threadIndex))
                    {
                        if (!frameData.valid)
                            break;

                        string threadName = string.IsNullOrEmpty(frameData.threadName)
                            ? $"Thread {threadIndex}"
                            : frameData.threadName;
                        if (!threadTotals.TryGetValue(threadName, out ThreadMetrics threadMetrics))
                        {
                            threadMetrics = new ThreadMetrics(threadName);
                            threadTotals.Add(threadName, threadMetrics);
                        }
                        threadMetrics.FramesSeen++;

                        if (frameData.sampleCount > 0)
                        {
                            float rootDuration = frameData.GetSampleTimeMs(0);
                            threadMetrics.RootDurationMs += rootDuration;
                            threadMetrics.MaxRootDurationMs =
                                Math.Max(threadMetrics.MaxRootDurationMs, rootDuration);
                            if (threadIndex == 0)
                                frameMetrics.MainThreadRootMs = rootDuration;
                        }

                        for (int sampleIndex = 0;
                             sampleIndex < frameData.sampleCount;
                             sampleIndex++)
                        {
                            string sampleName = frameData.GetSampleName(sampleIndex);
                            if (string.IsNullOrEmpty(sampleName) ||
                                !IsRelevant(sampleName))
                                continue;

                            float durationMs = frameData.GetSampleTimeMs(sampleIndex);
                            float startMs = frameData.GetSampleStartTimeMs(sampleIndex);
                            rawRows.Add(new RawRow(
                                frameIndex,
                                threadName,
                                sampleName,
                                startMs,
                                durationMs));

                            if (!markerTotals.TryGetValue(
                                    sampleName,
                                    out MarkerMetrics markerMetrics))
                            {
                                markerMetrics = new MarkerMetrics(sampleName);
                                markerTotals.Add(sampleName, markerMetrics);
                            }
                            markerMetrics.Add(frameIndex, threadName, durationMs);
                            threadMetrics.AddRelevant(sampleName, durationMs);
                            frameMetrics.Add(threadName, sampleName, durationMs);
                        }
                    }
                }
            }

            WriteRawRows(rawRows);
            WriteFrameSummary(frames.Values.ToList());
            WriteMarkerSummary(markerTotals.Values.ToList());
            WriteThreadSummary(threadTotals.Values.ToList());
            WriteSummary(
                firstFrame,
                lastFrame,
                frames.Values.ToList(),
                markerTotals.Values.ToList(),
                threadTotals.Values.ToList());
            File.WriteAllText(
                completionPath,
                $"completed_utc={DateTime.UtcNow:O}{Environment.NewLine}",
                Encoding.UTF8);
            Debug.Log($"[CodexProfilerDataExporter] Exported to {OutputDirectory}");
        }
        catch (Exception exception)
        {
            Directory.CreateDirectory(OutputDirectory);
            File.WriteAllText(
                Path.Combine(OutputDirectory, "error.txt"),
                exception.ToString(),
                Encoding.UTF8);
            Debug.LogException(exception);
        }
    }

    private static bool IsRelevant(string sampleName)
    {
        for (int i = 0; i < RelevantTokens.Length; i++)
        {
            if (sampleName.IndexOf(
                    RelevantTokens[i],
                    StringComparison.OrdinalIgnoreCase) >= 0)
                return true;
        }
        return false;
    }

    private static bool IsWorker(string threadName)
    {
        return threadName.IndexOf("Worker", StringComparison.OrdinalIgnoreCase) >= 0 ||
               threadName.IndexOf("Job", StringComparison.OrdinalIgnoreCase) >= 0;
    }

    private static string StageOf(string sampleName)
    {
        if (sampleName.Contains("EvaluateParallelJacobiPairsJob"))
            return "PairEvaluate";
        if (sampleName.Contains("ReduceParallelJacobiBlocksJob"))
            return "BlockReduce";
        if (sampleName.Contains("GatherAndApplyParallelJacobiBodiesJob"))
            return "BodyGather";
        if (sampleName.Contains("PrepareParallelJacobiSubstepJob"))
            return "PrepareSubstep";
        if (sampleName.Contains("PrepareParallelJacobiIterationJob"))
            return "PrepareIteration";
        if (sampleName.Contains("FinalizeParallelJacobiIterationJob"))
            return "FinalizeIteration";
        if (sampleName.Contains("FinalizeParallelJacobiSubstepJob"))
            return "FinalizeSubstep";
        if (sampleName.Contains("InitializeParallelJacobiPipelineJob"))
            return "InitializePipeline";
        if (sampleName.Contains("FinalizeParallelJacobiPipelineJob"))
            return "FinalizePipeline";
        if (sampleName.Contains("SolveXpbdUnitContactsJob"))
            return "SerialSolver";
        if (sampleName.Contains("JobHandle.Complete"))
            return "JobHandleComplete";
        if (sampleName.Contains("WaitForJobGroup"))
            return "WaitForJobGroup";
        return "Other";
    }

    private static void WriteRawRows(List<RawRow> rows)
    {
        using (var writer = new StreamWriter(
                   Path.Combine(OutputDirectory, "relevant_samples.csv"),
                   false,
                   new UTF8Encoding(false)))
        {
            writer.WriteLine("Frame,Thread,Stage,Marker,StartMs,DurationMs");
            foreach (RawRow row in rows)
            {
                writer.Write(row.Frame);
                writer.Write(',');
                writer.Write(Csv(row.Thread));
                writer.Write(',');
                writer.Write(StageOf(row.Marker));
                writer.Write(',');
                writer.Write(Csv(row.Marker));
                writer.Write(',');
                writer.Write(row.StartMs.ToString("R", CultureInfo.InvariantCulture));
                writer.Write(',');
                writer.WriteLine(row.DurationMs.ToString("R", CultureInfo.InvariantCulture));
            }
        }
    }

    private static void WriteFrameSummary(List<FrameMetrics> frames)
    {
        using (var writer = new StreamWriter(
                   Path.Combine(OutputDirectory, "frame_summary.csv"),
                   false,
                   new UTF8Encoding(false)))
        {
            writer.WriteLine(
                "Frame,MainThreadRootMs,PairEvaluateMs,BlockReduceMs,BodyGatherMs," +
                "PrepareSubstepMs,PrepareIterationMs,FinalizeIterationMs,FinalizeSubstepMs," +
                "InitializePipelineMs,FinalizePipelineMs,SerialSolverMs,JobHandleCompleteMs," +
                "WaitForJobGroupMs,JacobiThreadCount,JacobiWorkerCount,JacobiThreads");
            foreach (FrameMetrics frame in frames)
            {
                writer.Write(frame.Frame);
                writer.Write(',');
                writer.Write(F(frame.MainThreadRootMs));
                foreach (string stage in FrameMetrics.StageColumns)
                {
                    writer.Write(',');
                    writer.Write(F(frame.GetStage(stage)));
                }
                writer.Write(',');
                writer.Write(frame.JacobiThreads.Count);
                writer.Write(',');
                writer.Write(frame.JacobiThreads.Count(IsWorker));
                writer.Write(',');
                writer.WriteLine(Csv(string.Join("|", frame.JacobiThreads.OrderBy(v => v))));
            }
        }
    }

    private static void WriteMarkerSummary(List<MarkerMetrics> markers)
    {
        using (var writer = new StreamWriter(
                   Path.Combine(OutputDirectory, "marker_summary.csv"),
                   false,
                   new UTF8Encoding(false)))
        {
            writer.WriteLine(
                "Stage,Marker,Calls,Frames,Threads,TotalMs,MeanCallMs,MaxCallMs,P50FrameMs,P95FrameMs");
            foreach (MarkerMetrics marker in markers.OrderByDescending(v => v.TotalMs))
            {
                List<double> perFrame = marker.PerFrame.Values.ToList();
                writer.Write(StageOf(marker.Name));
                writer.Write(',');
                writer.Write(Csv(marker.Name));
                writer.Write(',');
                writer.Write(marker.Calls);
                writer.Write(',');
                writer.Write(marker.PerFrame.Count);
                writer.Write(',');
                writer.Write(Csv(string.Join("|", marker.Threads.OrderBy(v => v))));
                writer.Write(',');
                writer.Write(F(marker.TotalMs));
                writer.Write(',');
                writer.Write(F(marker.Calls > 0 ? marker.TotalMs / marker.Calls : 0d));
                writer.Write(',');
                writer.Write(F(marker.MaxCallMs));
                writer.Write(',');
                writer.Write(F(Percentile(perFrame, 0.50)));
                writer.Write(',');
                writer.WriteLine(F(Percentile(perFrame, 0.95)));
            }
        }
    }

    private static void WriteThreadSummary(List<ThreadMetrics> threads)
    {
        using (var writer = new StreamWriter(
                   Path.Combine(OutputDirectory, "thread_summary.csv"),
                   false,
                   new UTF8Encoding(false)))
        {
            writer.WriteLine(
                "Thread,FramesSeen,RootDurationMs,MaxRootDurationMs,RelevantCalls,RelevantInclusiveMs," +
                "PairEvaluateMs,BlockReduceMs,BodyGatherMs");
            foreach (ThreadMetrics thread in threads.OrderByDescending(v => v.RelevantInclusiveMs))
            {
                writer.Write(Csv(thread.Name));
                writer.Write(',');
                writer.Write(thread.FramesSeen);
                writer.Write(',');
                writer.Write(F(thread.RootDurationMs));
                writer.Write(',');
                writer.Write(F(thread.MaxRootDurationMs));
                writer.Write(',');
                writer.Write(thread.RelevantCalls);
                writer.Write(',');
                writer.Write(F(thread.RelevantInclusiveMs));
                writer.Write(',');
                writer.Write(F(thread.GetStage("PairEvaluate")));
                writer.Write(',');
                writer.Write(F(thread.GetStage("BlockReduce")));
                writer.Write(',');
                writer.WriteLine(F(thread.GetStage("BodyGather")));
            }
        }
    }

    private static void WriteSummary(
        int firstFrame,
        int lastFrame,
        List<FrameMetrics> frames,
        List<MarkerMetrics> markers,
        List<ThreadMetrics> threads)
    {
        List<FrameMetrics> jacobiFrames = frames
            .Where(frame => frame.HasJacobiWork)
            .ToList();
        List<double> workerCounts = jacobiFrames
            .Select(frame => (double)frame.JacobiThreads.Count(IsWorker))
            .ToList();

        var writer = new StringBuilder();
        writer.AppendLine($"input={InputPath}");
        writer.AppendLine($"first_frame={firstFrame}");
        writer.AppendLine($"last_frame={lastFrame}");
        writer.AppendLine($"frame_count={lastFrame - firstFrame + 1}");
        writer.AppendLine($"jacobi_frame_count={jacobiFrames.Count}");
        writer.AppendLine($"thread_count={threads.Count}");
        writer.AppendLine($"marker_count={markers.Count}");
        writer.AppendLine($"jacobi_worker_count_mean={F(workerCounts.Count > 0 ? workerCounts.Average() : 0d)}");
        writer.AppendLine($"jacobi_worker_count_p50={F(Percentile(workerCounts, 0.50))}");
        writer.AppendLine($"jacobi_worker_count_p95={F(Percentile(workerCounts, 0.95))}");
        writer.AppendLine($"jacobi_worker_count_max={F(workerCounts.Count > 0 ? workerCounts.Max() : 0d)}");
        writer.AppendLine();
        foreach (string stage in FrameMetrics.StageColumns)
        {
            List<double> values = jacobiFrames
                .Select(frame => frame.GetStage(stage))
                .ToList();
            writer.AppendLine(
                $"{stage}: mean_ms={F(values.Count > 0 ? values.Average() : 0d)}, " +
                $"p50_ms={F(Percentile(values, 0.50))}, " +
                $"p95_ms={F(Percentile(values, 0.95))}, " +
                $"max_ms={F(values.Count > 0 ? values.Max() : 0d)}");
        }
        writer.AppendLine();
        writer.AppendLine("top_relevant_markers:");
        foreach (MarkerMetrics marker in markers
                     .OrderByDescending(value => value.TotalMs)
                     .Take(40))
        {
            writer.AppendLine(
                $"{StageOf(marker.Name)} | {marker.Name} | calls={marker.Calls} | " +
                $"total_ms={F(marker.TotalMs)} | max_call_ms={F(marker.MaxCallMs)} | " +
                $"threads={string.Join("|", marker.Threads.OrderBy(value => value))}");
        }

        File.WriteAllText(
            Path.Combine(OutputDirectory, "summary.txt"),
            writer.ToString(),
            Encoding.UTF8);
    }

    private static double Percentile(List<double> values, double percentile)
    {
        if (values == null || values.Count == 0)
            return 0d;
        values.Sort();
        double position = (values.Count - 1) * percentile;
        int lower = (int)Math.Floor(position);
        int upper = (int)Math.Ceiling(position);
        if (lower == upper)
            return values[lower];
        double fraction = position - lower;
        return values[lower] * (1d - fraction) + values[upper] * fraction;
    }

    private static string Csv(string value)
    {
        return '"' + (value ?? string.Empty).Replace("\"", "\"\"") + '"';
    }

    private static string F(double value)
    {
        return value.ToString("0.######", CultureInfo.InvariantCulture);
    }

    private readonly struct RawRow
    {
        public readonly int Frame;
        public readonly string Thread;
        public readonly string Marker;
        public readonly float StartMs;
        public readonly float DurationMs;

        public RawRow(
            int frame,
            string thread,
            string marker,
            float startMs,
            float durationMs)
        {
            Frame = frame;
            Thread = thread;
            Marker = marker;
            StartMs = startMs;
            DurationMs = durationMs;
        }
    }

    private sealed class FrameMetrics
    {
        public static readonly string[] StageColumns =
        {
            "PairEvaluate",
            "BlockReduce",
            "BodyGather",
            "PrepareSubstep",
            "PrepareIteration",
            "FinalizeIteration",
            "FinalizeSubstep",
            "InitializePipeline",
            "FinalizePipeline",
            "SerialSolver",
            "JobHandleComplete",
            "WaitForJobGroup"
        };

        private readonly Dictionary<string, double> _stages =
            new Dictionary<string, double>(StringComparer.Ordinal);

        public readonly int Frame;
        public readonly HashSet<string> JacobiThreads =
            new HashSet<string>(StringComparer.Ordinal);
        public double MainThreadRootMs;

        public bool HasJacobiWork =>
            GetStage("PairEvaluate") > 0d ||
            GetStage("BodyGather") > 0d ||
            GetStage("SerialSolver") > 0d;

        public FrameMetrics(int frame)
        {
            Frame = frame;
        }

        public void Add(string threadName, string sampleName, double durationMs)
        {
            string stage = StageOf(sampleName);
            _stages.TryGetValue(stage, out double current);
            _stages[stage] = current + durationMs;
            if (stage == "PairEvaluate" ||
                stage == "BlockReduce" ||
                stage == "BodyGather")
            {
                JacobiThreads.Add(threadName);
            }
        }

        public double GetStage(string stage)
        {
            return _stages.TryGetValue(stage, out double value) ? value : 0d;
        }
    }

    private sealed class MarkerMetrics
    {
        public readonly string Name;
        public readonly HashSet<string> Threads =
            new HashSet<string>(StringComparer.Ordinal);
        public readonly Dictionary<int, double> PerFrame =
            new Dictionary<int, double>();
        public int Calls;
        public double TotalMs;
        public double MaxCallMs;

        public MarkerMetrics(string name)
        {
            Name = name;
        }

        public void Add(int frame, string thread, double durationMs)
        {
            Calls++;
            TotalMs += durationMs;
            MaxCallMs = Math.Max(MaxCallMs, durationMs);
            Threads.Add(thread);
            PerFrame.TryGetValue(frame, out double current);
            PerFrame[frame] = current + durationMs;
        }
    }

    private sealed class ThreadMetrics
    {
        private readonly Dictionary<string, double> _stages =
            new Dictionary<string, double>(StringComparer.Ordinal);

        public readonly string Name;
        public int FramesSeen;
        public int RelevantCalls;
        public double RootDurationMs;
        public double MaxRootDurationMs;
        public double RelevantInclusiveMs;

        public ThreadMetrics(string name)
        {
            Name = name;
        }

        public void AddRelevant(string sampleName, double durationMs)
        {
            RelevantCalls++;
            RelevantInclusiveMs += durationMs;
            string stage = StageOf(sampleName);
            _stages.TryGetValue(stage, out double current);
            _stages[stage] = current + durationMs;
        }

        public double GetStage(string stage)
        {
            return _stages.TryGetValue(stage, out double value) ? value : 0d;
        }
    }
}
}
#endif
