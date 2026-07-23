#!/usr/bin/env python3
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
PATH = ROOT / "Entities/Unit/Systems/FlowField/Jobs/ContactPipeline/Solver/ParallelContactPipelineP1P6.cs"
text = PATH.read_text(encoding="utf-8")

schedule_block = '''            handle = new PrepareSubstepContactPredictionBodiesJob
            {
                States = States,
                Skin = Configuration.PredictiveSkin,
                Margin = Configuration.TimestepContactMargin,
                Enabled = (byte)(Configuration.EnableTimestepContactSetCache ? 0 : 1)
            }.Schedule(States.Length, ParallelBodyBatchSize, handle);

'''
if schedule_block in text:
    text = text.replace(schedule_block, "", 1)

old_build = '''        if (!EnableTimestepContactSetCache && !rebuilt)
        {
            long start = ProfilerUnsafeUtility.Timestamp;
            BuildSubstepContactView(ref statistics, ref incremental);
'''
new_build = '''        if (!EnableTimestepContactSetCache && !rebuilt)
        {
            // Preserve the reference ordering: first validate the pre-soft swept
            // envelope, then publish the actual solved substep trajectory used by
            // Narrow Phase. Preparing this before validation would make every B0
            // validation trivially pass.
            PrepareSubstepContactPrediction();
            long start = ProfilerUnsafeUtility.Timestamp;
            BuildSubstepContactView(ref statistics, ref incremental);
'''
if old_build in text:
    text = text.replace(old_build, new_build, 1)
elif "PrepareSubstepContactPrediction();\n            long start" not in text:
    raise RuntimeError("B0 contact-view build anchor missing")

old_stats = '''        statistics.SoftAvoidanceCandidatePairCount += SoftAvoidancePairs.Length;
        statistics.SoftAvoidanceActivatedPairCount += activated;
'''
new_stats = '''        if (EnablePersistentContactCache &&
            SoftAvoidanceShell > 0f && SoftAvoidanceResponseRate > 0f)
            statistics.SoftAvoidanceFatAabbUseCount++;
        statistics.SoftAvoidanceCandidatePairCount += SoftAvoidancePairs.Length;
        statistics.SoftAvoidanceActivatedPairCount += activated;
'''
if old_stats in text:
    text = text.replace(old_stats, new_stats, 1)
elif "statistics.SoftAvoidanceFatAabbUseCount++;" not in text:
    raise RuntimeError("soft statistics anchor missing")

zero_delta_anchor = '''        float substepDeltaTime = Configuration.DeltaTime / substepCount;

        if (Configuration.EnableTimestepContactSetCache)
'''
zero_delta_replacement = '''        float substepDeltaTime = Configuration.DeltaTime / substepCount;
        if (substepDeltaTime <= 0f)
        {
            return new FinalizeParallelJacobiPipelineJob
            {
                Solver = this,
                RuntimeState = runtimeState
            }.Schedule(handle);
        }

        if (Configuration.EnableTimestepContactSetCache)
'''
if zero_delta_anchor in text:
    text = text.replace(zero_delta_anchor, zero_delta_replacement, 1)
elif "if (substepDeltaTime <= 0f)" not in text:
    raise RuntimeError("zero-delta scheduler anchor missing")

if "PrepareSubstepContactPredictionBodiesJob\n            {" in text:
    raise RuntimeError("B0 prediction is still scheduled before validation")
if "PrepareSubstepContactPrediction();\n            long start" not in text:
    raise RuntimeError("B0 post-validation prediction is missing")

PATH.write_text(text, encoding="utf-8", newline="\n")
