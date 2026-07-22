#!/usr/bin/env python3
"""Summarise AdaptiveParameterTuner scenario benchmark output without third-party packages."""
from __future__ import annotations

import csv
import statistics
import sys
from collections import defaultdict
from pathlib import Path

METRICS = [
    "AvgSolverNs", "AvgPairGenerationNs", "AvgTimestepContactSetBuildNs",
    "AvgIterationNs", "AvgSoftAvoidNs", "AvgFullSweepSourceNs",
    "AvgPersistentMapNs", "AvgProxyValidationNs",
    "AvgLocalBroadPhaseNs", "AvgPairDiffNs", "AvgClassificationNs",
    "AvgContactActivationNs", "AvgFallbackNs",
    "AvgDirtyBodies", "AvgMotionDirtyBodies", "AvgPersistentPairs", "AvgInteractionPairs",
    "AvgSoftAvoidancePairs", "AvgClassificationEvaluations", "AvgClassificationSkipped",
    "AvgSoftPairEvaluations", "AvgConstraintEvaluations", "AvgPersistentViewReuse",
    "AvgPersistentViewRebuild", "AvgInteractionEnvelopeEscapes", "AvgFullRebuilds",
    "AvgIncrementalRepairs", "AvgContactPairs", "AvgActivePairs", "AvgPredictivePairs",
    "AvgSoftOracleMissing",
]


def number(row, key):
    try:
        return float(row[key])
    except (KeyError, ValueError):
        return 0.0


def percentile(values, p):
    values = sorted(values)
    if not values:
        return 0.0
    index = (len(values) - 1) * p
    lo, hi = int(index), min(int(index) + 1, len(values) - 1)
    return values[lo] + (values[hi] - values[lo]) * (index - lo)


def choose_input(arg):
    path = Path(arg).resolve()
    if path.is_dir():
        return path / "adaptive_tuning_summary.csv", path
    return path, path.parent


def main():
    if len(sys.argv) != 2:
        print("usage: analyze_adaptive_benchmark.py <benchmark directory | adaptive_tuning_summary.csv>")
        return 2
    summary_path, output_dir = choose_input(sys.argv[1])
    if not summary_path.exists():
        print(f"missing summary: {summary_path}", file=sys.stderr)
        return 2

    with summary_path.open(newline="", encoding="utf-8-sig") as handle:
        rows = list(csv.DictReader(handle))
    if not rows:
        print("summary contains no completed trials", file=sys.stderr)
        return 2
    has_pair_generation = "AvgPairGenerationNs" in rows[0]
    has_unified_metrics = all(key in rows[0] for key in (
        "AvgFullSweepSourceNs", "AvgPersistentMapNs", "AvgInteractionPairs",
        "AvgSoftAvoidancePairs", "AvgClassificationEvaluations",
        "AvgClassificationSkipped", "AvgConstraintEvaluations"))

    issues = []
    by_scenario = defaultdict(list)
    for row in rows:
        by_scenario[row["Scenario"]].append(row)
    for scenario, scenario_rows in by_scenario.items():
        hashes = {row["BaselineHash"] for row in scenario_rows}
        if len(hashes) != 1:
            issues.append(f"INVALID {scenario}: baseline hash differs across trials ({', '.join(sorted(hashes))})")
        if any(number(row, "AvgSoftOracleMissing") > 0 for row in scenario_rows):
            issues.append(f"INVALID {scenario}: soft-avoidance oracle reported missing pairs")

    grouped = defaultdict(list)
    for row in rows:
        grouped[(row["Scenario"], row["Mode"], row["Profile"])].append(row)

    aggregate_rows = []
    for key, group in sorted(grouped.items()):
        scenario, mode, profile = key
        out = {"Scenario": scenario, "Mode": mode, "Profile": profile, "Runs": len(group)}
        for metric in METRICS:
            values = [number(row, metric) for row in group]
            out[metric + "Mean"] = statistics.fmean(values)
            out[metric + "P50"] = percentile(values, 0.50)
            out[metric + "P95"] = percentile(values, 0.95)
        aggregate_rows.append(out)

    comparison_rows = []
    for scenario, scenario_rows in sorted(by_scenario.items()):
        baselines = [r for r in scenario_rows if r.get("CrossFrameCache") == "0" and r.get("CrossSubstepCache") == "1"]
        candidates = [r for r in scenario_rows if r.get("CrossFrameCache") == "1" and r.get("CrossSubstepCache") == "1"]
        if not baselines or not candidates:
            issues.append(f"REVIEW {scenario}: need A0_B1 and A1_B1 rows for comparison")
            continue
        base_by_repeat = {r["Repetition"]: r for r in baselines}
        candidate_by_repeat = {r["Repetition"]: r for r in candidates}
        common = sorted(set(base_by_repeat) & set(candidate_by_repeat))
        if not common:
            issues.append(f"INVALID {scenario}: no matched repetitions")
            continue
        base_values = [number(base_by_repeat[r], "AvgSolverNs") for r in common]
        candidate_values = [number(candidate_by_repeat[r], "AvgSolverNs") for r in common]
        base_mean, candidate_mean = statistics.fmean(base_values), statistics.fmean(candidate_values)
        comparison_rows.append({
            "Scenario": scenario,
            "Pairs": len(common),
            "BaselineProfile": baselines[0]["Profile"],
            "CandidateProfile": candidates[0]["Profile"],
            "BaselineSolverNsMean": base_mean,
            "CandidateSolverNsMean": candidate_mean,
            "SolverDeltaNs": candidate_mean - base_mean,
            "SolverDeltaPercent": 0.0 if base_mean == 0 else (candidate_mean - base_mean) * 100.0 / base_mean,
            "PairGenerationAvailable": int(has_pair_generation),
            "UnifiedMetricsAvailable": int(has_unified_metrics),
            "BaselinePairGenerationNsMean": statistics.fmean(number(base_by_repeat[r], "AvgPairGenerationNs") for r in common),
            "CandidatePairGenerationNsMean": statistics.fmean(number(candidate_by_repeat[r], "AvgPairGenerationNs") for r in common),
            "PairGenerationDeltaNs": statistics.fmean(number(candidate_by_repeat[r], "AvgPairGenerationNs") - number(base_by_repeat[r], "AvgPairGenerationNs") for r in common),
            "BaselineFullSweepSourceNsMean": statistics.fmean(number(base_by_repeat[r], "AvgFullSweepSourceNs") for r in common),
            "CandidatePersistentMapNsMean": statistics.fmean(number(candidate_by_repeat[r], "AvgPersistentMapNs") for r in common),
            "CandidateProxyValidationNsMean": statistics.fmean(number(candidate_by_repeat[r], "AvgProxyValidationNs") for r in common),
            "CandidateLocalBroadPhaseNsMean": statistics.fmean(number(candidate_by_repeat[r], "AvgLocalBroadPhaseNs") for r in common),
            "CandidatePairDiffNsMean": statistics.fmean(number(candidate_by_repeat[r], "AvgPairDiffNs") for r in common),
            "BaselineClassificationNsMean": statistics.fmean(number(base_by_repeat[r], "AvgClassificationNs") for r in common),
            "CandidateClassificationNsMean": statistics.fmean(number(candidate_by_repeat[r], "AvgClassificationNs") for r in common),
            "CandidateFallbackNsMean": statistics.fmean(number(candidate_by_repeat[r], "AvgFallbackNs") for r in common),
            "BaselineDirtyBodiesMean": statistics.fmean(number(base_by_repeat[r], "AvgDirtyBodies") for r in common),
            "CandidateDirtyBodiesMean": statistics.fmean(number(candidate_by_repeat[r], "AvgDirtyBodies") for r in common),
            "CandidateMotionDirtyBodiesMean": statistics.fmean(number(candidate_by_repeat[r], "AvgMotionDirtyBodies") for r in common),
            "CandidatePersistentPairsMean": statistics.fmean(number(candidate_by_repeat[r], "AvgPersistentPairs") for r in common),
            "InteractionPairDelta": statistics.fmean(number(candidate_by_repeat[r], "AvgInteractionPairs") - number(base_by_repeat[r], "AvgInteractionPairs") for r in common),
            "SoftAvoidancePairDelta": statistics.fmean(number(candidate_by_repeat[r], "AvgSoftAvoidancePairs") - number(base_by_repeat[r], "AvgSoftAvoidancePairs") for r in common),
            "SoftPairEvaluationDelta": statistics.fmean(number(candidate_by_repeat[r], "AvgSoftPairEvaluations") - number(base_by_repeat[r], "AvgSoftPairEvaluations") for r in common),
            "BaselineClassificationEvaluationsMean": statistics.fmean(number(base_by_repeat[r], "AvgClassificationEvaluations") for r in common),
            "CandidateClassificationEvaluationsMean": statistics.fmean(number(candidate_by_repeat[r], "AvgClassificationEvaluations") for r in common),
            "CandidateClassificationSkippedMean": statistics.fmean(number(candidate_by_repeat[r], "AvgClassificationSkipped") for r in common),
            "CandidatePersistentViewReuseMean": statistics.fmean(number(candidate_by_repeat[r], "AvgPersistentViewReuse") for r in common),
            "CandidatePersistentViewRebuildMean": statistics.fmean(number(candidate_by_repeat[r], "AvgPersistentViewRebuild") for r in common),
            "CandidateInteractionEnvelopeEscapesMean": statistics.fmean(number(candidate_by_repeat[r], "AvgInteractionEnvelopeEscapes") for r in common),
            "ConstraintEvaluationDelta": statistics.fmean(number(candidate_by_repeat[r], "AvgConstraintEvaluations") - number(base_by_repeat[r], "AvgConstraintEvaluations") for r in common),
            "IterationDeltaNs": statistics.fmean(number(candidate_by_repeat[r], "AvgIterationNs") - number(base_by_repeat[r], "AvgIterationNs") for r in common),
            "SoftAvoidDeltaNs": statistics.fmean(number(candidate_by_repeat[r], "AvgSoftAvoidNs") - number(base_by_repeat[r], "AvgSoftAvoidNs") for r in common),
        })

    aggregate_path = output_dir / "analysis_summary.csv"
    with aggregate_path.open("w", newline="", encoding="utf-8") as handle:
        fields = ["Scenario", "Mode", "Profile", "Runs"] + [f"{m}{s}" for m in METRICS for s in ("Mean", "P50", "P95")]
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader(); writer.writerows(aggregate_rows)

    comparison_path = output_dir / "analysis_comparison.csv"
    with comparison_path.open("w", newline="", encoding="utf-8") as handle:
        fields = ["Scenario", "Pairs", "BaselineProfile", "CandidateProfile", "BaselineSolverNsMean", "CandidateSolverNsMean", "SolverDeltaNs", "SolverDeltaPercent", "PairGenerationAvailable", "UnifiedMetricsAvailable", "BaselinePairGenerationNsMean", "CandidatePairGenerationNsMean", "PairGenerationDeltaNs", "BaselineFullSweepSourceNsMean", "CandidatePersistentMapNsMean", "CandidateProxyValidationNsMean", "CandidateLocalBroadPhaseNsMean", "CandidatePairDiffNsMean", "BaselineClassificationNsMean", "CandidateClassificationNsMean", "CandidateFallbackNsMean", "BaselineDirtyBodiesMean", "CandidateDirtyBodiesMean", "CandidateMotionDirtyBodiesMean", "CandidatePersistentPairsMean", "InteractionPairDelta", "SoftAvoidancePairDelta", "SoftPairEvaluationDelta", "BaselineClassificationEvaluationsMean", "CandidateClassificationEvaluationsMean", "CandidateClassificationSkippedMean", "CandidatePersistentViewReuseMean", "CandidatePersistentViewRebuildMean", "CandidateInteractionEnvelopeEscapesMean", "ConstraintEvaluationDelta", "IterationDeltaNs", "SoftAvoidDeltaNs"]
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader(); writer.writerows(comparison_rows)

    report_path = output_dir / "analysis_report.md"
    with report_path.open("w", encoding="utf-8") as handle:
        handle.write("# Adaptive Parameter Tuner 分析\n\n")
        if issues:
            handle.write("## 数据有效性\n" + "\n".join(f"- {item}" for item in issues) + "\n\n")
        else:
            handle.write("## 数据有效性\n- PASS：同一场景内所有 trial 使用同一 BaselineHash。\n\n")
        handle.write("## A0_B1 vs A1_B1\n")
        for row in comparison_rows:
            pair_generation = (
                f"PairGen Δ={row['PairGenerationDeltaNs'] / 1000:+.1f}us，"
                if row["PairGenerationAvailable"] else "PairGen=旧数据未记录，")
            base = (f"- **{row['Scenario']}**：{row['SolverDeltaPercent']:+.2f}% "
                    f"({row['BaselineSolverNsMean'] / 1000:.1f}us → {row['CandidateSolverNsMean'] / 1000:.1f}us)，"
                    f"{pair_generation}")
            if not row["UnifiedMetricsAvailable"]:
                handle.write(base + "统一模型指标=旧数据未记录。\n")
                continue
            handle.write(base +
                         f"A0 source/classify={row['BaselineFullSweepSourceNsMean'] / 1000:.1f}/"
                         f"{row['BaselineClassificationNsMean'] / 1000:.1f}us；"
                         f"A1 validation/map/local/diff/classify="
                         f"{row['CandidateProxyValidationNsMean'] / 1000:.1f}/"
                         f"{row['CandidatePersistentMapNsMean'] / 1000:.1f}/"
                         f"{row['CandidateLocalBroadPhaseNsMean'] / 1000:.1f}/"
                         f"{row['CandidatePairDiffNsMean'] / 1000:.1f}/"
                         f"{row['CandidateClassificationNsMean'] / 1000:.1f}us；"
                         f"Interaction/Soft视图/Soft评估/XPBD求解次数 Δ="
                         f"{row['InteractionPairDelta']:+.1f}/"
                         f"{row['SoftAvoidancePairDelta']:+.1f}/"
                         f"{row['SoftPairEvaluationDelta']:+.1f}/"
                         f"{row['ConstraintEvaluationDelta']:+.1f}；"
                         f"分类 eval A0→A1={row['BaselineClassificationEvaluationsMean']:.1f}→"
                         f"{row['CandidateClassificationEvaluationsMean']:.1f}，"
                         f"skipped={row['CandidateClassificationSkippedMean']:.1f}；"
                         f"持久视图 reuse/rebuild={row['CandidatePersistentViewReuseMean']:.2f}/"
                         f"{row['CandidatePersistentViewRebuildMean']:.2f}，"
                         f"interaction escapes={row['CandidateInteractionEnvelopeEscapesMean']:.2f}；"
                         f"Soft/Iteration Δ={row['SoftAvoidDeltaNs'] / 1000:+.1f}/"
                         f"{row['IterationDeltaNs'] / 1000:+.1f}us；"
                         f"A1 persistent pairs={row['CandidatePersistentPairsMean']:.1f}，"
                         f"topology/motion dirty={row['CandidateDirtyBodiesMean']:.2f}/"
                         f"{row['CandidateMotionDirtyBodiesMean']:.2f}。\n")
        handle.write("\nPairGeneration 是父级完整生成阶段；A0 source/classify 与 A1 validation/map/local/diff/classify 是其内部归因项，不能再与 PairGeneration 相加。\n")

    print(f"wrote {aggregate_path}")
    print(f"wrote {comparison_path}")
    print(f"wrote {report_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
