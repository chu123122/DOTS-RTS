#!/usr/bin/env python3
"""Analyze Stage3ContactDiagnostic/v2-v3 OFF/ON captures and judge the trend."""

from __future__ import annotations

import argparse
import json
import math
import os
import re
import statistics
import sys
import tempfile
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable


DEFAULT_DIRECTORY = (
    Path.home()
    / "AppData"
    / "LocalLow"
    / "DefaultCompany"
    / "RTS"
    / "Stage3ContactDiagnostics"
)
RUN_LABEL_PATTERN = re.compile(
    r"^fat-aabb-r(?P<round>\d+)"
    r"(?:-s(?P<substeps>\d+)-i(?P<iterations>\d+))?"
    r"-(?P<phase>off-before|on|off-after)$"
)


@dataclass(frozen=True)
class RunSummary:
    path: Path
    label: str
    cache_enabled: bool
    sample_count: int
    substeps: int
    iterations: int
    pair_us: float
    solver_us: float
    penetration_p95: float
    velocity_change_p95: float
    residual_p95: float
    cache_uses: int
    cache_reuses: int
    cache_rebuilds: int
    cache_fallbacks: int
    mapping_builds: int
    mapping_reuses: int
    corrected_body_checks: int
    candidate_inflation: float


def percentile(values: Iterable[float], fraction: float) -> float:
    ordered = sorted(float(value) for value in values)
    if not ordered:
        return 0.0
    position = (len(ordered) - 1) * fraction
    lower = math.floor(position)
    upper = math.ceil(position)
    if lower == upper:
        return ordered[lower]
    weight = position - lower
    return ordered[lower] * (1.0 - weight) + ordered[upper] * weight


def number(sample: dict[str, Any], field: str) -> float:
    value = sample.get(field, 0)
    return float(value) if isinstance(value, (int, float)) else 0.0


def final_residual(sample: dict[str, Any]) -> float:
    values = sample.get("ResidualAfterByIteration") or []
    return float(values[-1]) if values else 0.0


def summarize_file(path: Path) -> RunSummary:
    with path.open("r", encoding="utf-8-sig") as handle:
        document = json.load(handle)

    if document.get("Format") not in {
        "Stage3ContactDiagnostic/v2",
        "Stage3ContactDiagnostic/v3",
    }:
        raise ValueError(f"unsupported format {document.get('Format')!r}")

    samples = document.get("Samples") or []
    if not samples:
        raise ValueError("contains no samples")

    cache_values = {bool(sample.get("FatAabbCacheEnabled")) for sample in samples}
    if len(cache_values) != 1:
        raise ValueError("mixes Fat AABB OFF and ON samples")
    cache_enabled = cache_values.pop()
    substep_values = {int(number(sample, "Substeps")) for sample in samples}
    iteration_values = {int(number(sample, "Iterations")) for sample in samples}
    if len(substep_values) != 1 or len(iteration_values) != 1:
        raise ValueError("mixes multiple Substeps/Iterations configurations")
    substeps = substep_values.pop()
    iterations = iteration_values.pop()

    inflations: list[float] = []
    for sample in samples:
        sample_substeps = max(1.0, number(sample, "Substeps"))
        # ContactPairs 是 timestep 唯一集合；将其除以 substep 后，得到的比值表示
        # 同一 Fat 候选列表在整帧各 substep 重复扫描的总倍率，而不是缓存存储膨胀。
        contacts_per_substep = number(sample, "ContactPairs") / sample_substeps
        if contacts_per_substep > 0.0:
            inflations.append(
                number(sample, "FatAabbCachedCandidatePairs")
                / contacts_per_substep
            )

    return RunSummary(
        path=path,
        label=str(document.get("RunLabel") or path.stem),
        cache_enabled=cache_enabled,
        sample_count=len(samples),
        substeps=substeps,
        iterations=iterations,
        pair_us=statistics.median(
            number(sample, "PairGenerationMicroseconds") for sample in samples
        ),
        solver_us=statistics.median(
            number(sample, "SolverMicroseconds") for sample in samples
        ),
        penetration_p95=percentile(
            (number(sample, "MaxPenetration") for sample in samples), 0.95
        ),
        velocity_change_p95=percentile(
            (number(sample, "MaxVelocityChange") for sample in samples), 0.95
        ),
        residual_p95=percentile((final_residual(sample) for sample in samples), 0.95),
        cache_uses=sum(int(number(sample, "FatAabbCacheUses")) for sample in samples),
        cache_reuses=sum(
            int(number(sample, "FatAabbCacheReuses")) for sample in samples
        ),
        cache_rebuilds=sum(
            int(number(sample, "FatAabbCacheRebuilds")) for sample in samples
        ),
        cache_fallbacks=sum(
            int(number(sample, "FatAabbFullBroadPhaseFallbacks"))
            for sample in samples
        ),
        mapping_builds=sum(
            int(number(sample, "FatAabbMappingBuilds")) for sample in samples
        ),
        mapping_reuses=sum(
            int(number(sample, "FatAabbMappingReuses")) for sample in samples
        ),
        corrected_body_checks=sum(
            int(number(sample, "FatAabbCorrectedBodyChecks")) for sample in samples
        ),
        candidate_inflation=(statistics.median(inflations) if inflations else 0.0),
    )


def median_field(runs: list[RunSummary], field: str) -> float:
    return statistics.median(float(getattr(run, field)) for run in runs)


def percent_change(on_value: float, off_value: float) -> float:
    if abs(off_value) <= 1e-12:
        return 0.0 if abs(on_value) <= 1e-12 else math.inf
    return (on_value / off_value - 1.0) * 100.0


def format_change(value: float) -> str:
    if math.isinf(value):
        return "+inf"
    return f"{value:+.1f}%"


def physical_stable(on_value: float, off_value: float, absolute_floor: float) -> bool:
    allowed = max(off_value * 1.10, off_value + absolute_floor)
    return on_value <= allowed


def bracketed_changes(runs: list[RunSummary], field: str) -> list[float]:
    rounds: dict[int, dict[str, RunSummary]] = {}
    for run in runs:
        match = RUN_LABEL_PATTERN.match(run.label)
        if not match:
            continue
        rounds.setdefault(int(match.group("round")), {})[match.group("phase")] = run

    changes: list[float] = []
    for phases in rounds.values():
        if not {"off-before", "on", "off-after"}.issubset(phases):
            continue
        off_reference = (
            float(getattr(phases["off-before"], field))
            + float(getattr(phases["off-after"], field))
        ) / 2.0
        changes.append(percent_change(float(getattr(phases["on"], field)), off_reference))
    return changes


def analyze_configuration(
    runs: list[RunSummary],
    min_reuse_rate: float,
    max_fallback_rate: float,
    max_inflation: float,
) -> tuple[str, list[str]]:
    off_runs = [run for run in runs if not run.cache_enabled]
    on_runs = [run for run in runs if run.cache_enabled]
    if not off_runs or not on_runs:
        return "INSUFFICIENT", ["必须同时存在 Fat AABB OFF 和 ON 的 v2 录制。"]

    pair_off = median_field(off_runs, "pair_us")
    pair_on = median_field(on_runs, "pair_us")
    solver_off = median_field(off_runs, "solver_us")
    solver_on = median_field(on_runs, "solver_us")
    penetration_off = median_field(off_runs, "penetration_p95")
    penetration_on = median_field(on_runs, "penetration_p95")
    velocity_off = median_field(off_runs, "velocity_change_p95")
    velocity_on = median_field(on_runs, "velocity_change_p95")
    residual_off = median_field(off_runs, "residual_p95")
    residual_on = median_field(on_runs, "residual_p95")

    paired_pair_changes = bracketed_changes(runs, "pair_us")
    paired_solver_changes = bracketed_changes(runs, "solver_us")
    pair_change = (
        statistics.median(paired_pair_changes)
        if paired_pair_changes
        else percent_change(pair_on, pair_off)
    )
    solver_change = (
        statistics.median(paired_solver_changes)
        if paired_solver_changes
        else percent_change(solver_on, solver_off)
    )

    uses = sum(run.cache_uses for run in on_runs)
    reuses = sum(run.cache_reuses for run in on_runs)
    fallbacks = sum(run.cache_fallbacks for run in on_runs)
    rebuilds = sum(run.cache_rebuilds for run in on_runs)
    mapping_builds = sum(run.mapping_builds for run in on_runs)
    mapping_reuses = sum(run.mapping_reuses for run in on_runs)
    corrected_body_checks = sum(run.corrected_body_checks for run in on_runs)
    reuse_rate = reuses / uses if uses else 0.0
    fallback_rate = fallbacks / uses if uses else 1.0
    inflation = median_field(on_runs, "candidate_inflation")

    pair_improved = pair_change <= -5.0
    solver_not_regressed = solver_change <= 10.0
    penetration_stable = physical_stable(penetration_on, penetration_off, 1e-4)
    velocity_stable = physical_stable(velocity_on, velocity_off, 1e-3)
    residual_stable = physical_stable(residual_on, residual_off, 1e-4)
    cache_healthy = (
        reuse_rate >= min_reuse_rate
        and fallback_rate <= max_fallback_rate
        and inflation <= max_inflation
    )

    notes = [
        f"Pair 生成中位数: OFF {pair_off:.2f} us -> ON {pair_on:.2f} us "
        f"({format_change(pair_change)})",
        f"Solver 中位数: OFF {solver_off:.2f} us -> ON {solver_on:.2f} us "
        f"({format_change(solver_change)})",
        f"P95 最大穿透: OFF {penetration_off:.6f} -> ON {penetration_on:.6f} "
        f"[{'稳定' if penetration_stable else '退化'}]",
        f"P95 最大速度变化: OFF {velocity_off:.6f} -> ON {velocity_on:.6f} "
        f"[{'稳定' if velocity_stable else '退化'}]",
        f"P95 最终残差: OFF {residual_off:.6f} -> ON {residual_on:.6f} "
        f"[{'稳定' if residual_stable else '退化'}]",
        f"缓存健康度: reuse={reuse_rate:.1%}, fallback={fallback_rate:.1%}, "
        f"rebuilds={rebuilds}, substep-scan/contact={inflation:.2f}x",
        f"优化计数: Pair 映射构建={mapping_builds}, 帧内复用={mapping_reuses}, "
        f"修正单位 AABB 检查={corrected_body_checks}",
    ]

    off_before = [run for run in runs if run.label.endswith("off-before")]
    off_after = [run for run in runs if run.label.endswith("off-after")]
    if off_before and off_after:
        pair_drift = percent_change(
            median_field(off_after, "pair_us"), median_field(off_before, "pair_us")
        )
        solver_drift = percent_change(
            median_field(off_after, "solver_us"), median_field(off_before, "solver_us")
        )
        notes.append(
            f"OFF 前后漂移: Pair {format_change(pair_drift)}, "
            f"Solver {format_change(solver_drift)}"
        )
        if abs(pair_drift) > 15.0 or abs(solver_drift) > 15.0:
            notes.append("警告: OFF 前后漂移超过 15%，场景或机器负载可能不稳定。")

    if not pair_improved:
        notes.append("判定: Pair 生成没有达到至少 5% 的稳定收益。")
    if not solver_not_regressed:
        notes.append("判定: Solver 总耗时回退超过 10%。")
    if not cache_healthy:
        notes.append(
            "判定: 缓存复用率、回退率或 substep 扫描倍率未达到继续扫描 margin 的门槛。"
        )
    if not (penetration_stable and velocity_stable and residual_stable):
        notes.append("判定: 物理结果相对 OFF 基线出现退化。")

    recommended = (
        pair_improved
        and solver_not_regressed
        and penetration_stable
        and velocity_stable
        and residual_stable
        and cache_healthy
    )
    return ("PASS" if recommended else "REVIEW"), notes


def analyze(
    runs: list[RunSummary],
    min_reuse_rate: float,
    max_fallback_rate: float,
    max_inflation: float,
) -> tuple[str, list[str]]:
    configurations: dict[tuple[int, int], list[RunSummary]] = {}
    for run in runs:
        configurations.setdefault((run.substeps, run.iterations), []).append(run)

    results: list[str] = []
    notes: list[str] = []
    for (substeps, iterations), configuration_runs in sorted(configurations.items()):
        result, configuration_notes = analyze_configuration(
            configuration_runs,
            min_reuse_rate=min_reuse_rate,
            max_fallback_rate=max_fallback_rate,
            max_inflation=max_inflation,
        )
        results.append(result)
        notes.append(f"配置 {substeps}x{iterations}: {result}")
        notes.extend(f"  {note}" for note in configuration_notes)

    if any(result == "INSUFFICIENT" for result in results):
        return "INSUFFICIENT", notes
    if results and all(result == "PASS" for result in results):
        return "PASS", notes
    return "REVIEW", notes


def print_runs(runs: list[RunSummary]) -> None:
    print("\n录制明细")
    print("label                           config mode samples pair_us solver_us reuse fallback scan/contact")
    for run in sorted(runs, key=lambda item: item.label):
        reuse_rate = run.cache_reuses / run.cache_uses if run.cache_uses else 0.0
        fallback_rate = run.cache_fallbacks / run.cache_uses if run.cache_uses else 0.0
        print(
            f"{run.label[:31]:31} "
            f"{run.substeps}x{run.iterations:<3} "
            f"{'ON ' if run.cache_enabled else 'OFF'} "
            f"{run.sample_count:7d} {run.pair_us:7.2f} {run.solver_us:9.2f} "
            f"{reuse_rate:5.1%} {fallback_rate:8.1%} {run.candidate_inflation:9.2f}x"
        )


def write_fixture(
    path: Path,
    label: str,
    cache_enabled: bool,
    pair_us: float,
    solver_us: float,
    substeps: int,
    iterations: int,
) -> None:
    samples = []
    for index in range(20):
        samples.append(
            {
                "Substeps": substeps,
                "Iterations": iterations,
                "PairGenerationMicroseconds": pair_us + index % 3,
                "SolverMicroseconds": solver_us + index % 5,
                "MaxPenetration": 0.01,
                "MaxVelocityChange": 0.2,
                "ResidualAfterByIteration": [0.02, 0.01],
                "FatAabbCacheEnabled": cache_enabled,
                "FatAabbCacheUses": 2 if cache_enabled else 0,
                "FatAabbCacheReuses": 2 if cache_enabled else 0,
                "FatAabbCacheRebuilds": 0,
                "FatAabbFullBroadPhaseFallbacks": 0,
                "FatAabbCachedCandidatePairs": 20 if cache_enabled else 0,
                "ContactPairs": 20,
            }
        )
    path.write_text(
        json.dumps(
            {
                "Format": "Stage3ContactDiagnostic/v2",
                "RunLabel": label,
                "Samples": samples,
            }
        ),
        encoding="utf-8",
    )


def self_test() -> None:
    with tempfile.TemporaryDirectory() as temporary_directory:
        directory = Path(temporary_directory)
        for round_index, (substeps, iterations) in enumerate(
            ((1, 8), (2, 4), (4, 2)), start=1
        ):
            prefix = f"fat-aabb-r{round_index:02}-s{substeps}-i{iterations}"
            write_fixture(
                directory / f"r{round_index}-before.json",
                f"{prefix}-off-before",
                False,
                100,
                200,
                substeps,
                iterations,
            )
            write_fixture(
                directory / f"r{round_index}-on.json",
                f"{prefix}-on",
                True,
                60,
                175,
                substeps,
                iterations,
            )
            write_fixture(
                directory / f"r{round_index}-after.json",
                f"{prefix}-off-after",
                False,
                102,
                205,
                substeps,
                iterations,
            )
        runs = [summarize_file(path) for path in sorted(directory.glob("*.json"))]
        result, _ = analyze(runs, 0.8, 0.05, 4.0)
        if result != "PASS":
            raise AssertionError(f"expected PASS, got {result}")
    print("SELF_TEST_OK")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="分析 Stage3ContactDiagnostic/v2-v3 Fat AABB OFF/ON 趋势。"
    )
    parser.add_argument("directory", nargs="?", type=Path, default=DEFAULT_DIRECTORY)
    parser.add_argument("--min-reuse-rate", type=float, default=0.80)
    parser.add_argument("--max-fallback-rate", type=float, default=0.05)
    parser.add_argument("--max-inflation", type=float, default=4.0)
    parser.add_argument("--self-test", action="store_true")
    return parser.parse_args()


def main() -> int:
    if os.name == "nt":
        sys.stdout.reconfigure(encoding="utf-8")
        sys.stderr.reconfigure(encoding="utf-8")

    args = parse_args()
    if args.self_test:
        self_test()
        return 0

    directory = args.directory.expanduser().resolve()
    if not directory.is_dir():
        print(f"INSUFFICIENT: 目录不存在: {directory}", file=sys.stderr)
        return 1

    runs: list[RunSummary] = []
    warnings: list[str] = []
    for path in sorted(directory.rglob("*.json")):
        try:
            runs.append(summarize_file(path))
        except (OSError, json.JSONDecodeError, ValueError) as exception:
            warnings.append(f"跳过 {path.name}: {exception}")

    for warning in warnings:
        print(f"WARNING: {warning}", file=sys.stderr)
    if not runs:
        print("INSUFFICIENT: 没有可分析的 v2 JSON。", file=sys.stderr)
        return 1

    print_runs(runs)
    result, notes = analyze(
        runs,
        min_reuse_rate=args.min_reuse_rate,
        max_fallback_rate=args.max_fallback_rate,
        max_inflation=args.max_inflation,
    )
    print(f"\n趋势结论: {result}")
    for note in notes:
        print(f"- {note}")
    if result == "PASS":
        print("- 建议: 可以进入 Fat AABB margin 扫描。")
        return 0
    if result == "INSUFFICIENT":
        return 1
    print("- 建议: 暂不进入 margin 扫描，先检查上述退化项。")
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
