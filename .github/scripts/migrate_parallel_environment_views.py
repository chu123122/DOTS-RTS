from pathlib import Path
import re

path = Path(
    "Entities/Unit/Systems/FlowField/Jobs/ContactPipeline/Solver/"
    "ParallelContactPipelineP1P6.cs"
)
source = path.read_text(encoding="utf-8")
original = source

# Base motion no longer retains/reads a complete FlowFieldCell.
source = source.replace(
    "state.Cell.Cost == 0",
    "state.Navigation.IsBlocked != 0",
)

world_to_cell = re.compile(
    r"(?P<indent>\s*)int2 currentCell = FlowFieldUtils\.WorldToCell\(\n"
    r"(?P=indent)    state\.PredictedPosition,\n"
    r"(?P=indent)    GridOrigin,\n"
    r"(?P=indent)    CellRadius\);"
)

def add_geometry(match: re.Match[str]) -> str:
    indent = match.group("indent")
    return (
        f"{indent}int2 currentCell = FlowFieldUtils.WorldToCell(\n"
        f"{indent}    state.PredictedPosition,\n"
        f"{indent}    GridOrigin,\n"
        f"{indent}    CellRadius);\n"
        f"{indent}FlowGridGeometry obstacleGeometry = new FlowGridGeometry(\n"
        f"{indent}    GridOrigin, GridDimensions, CellRadius);"
    )

source, geometry_count = world_to_cell.subn(add_geometry, source)

blocked_block = re.compile(
    r"(?P<indent>\s*)if \(checkCell\.x < 0 \|\| checkCell\.x >= GridDimensions\.x \|\|\n"
    r"(?P=indent)    checkCell\.y < 0 \|\| checkCell\.y >= GridDimensions\.y\)\n"
    r"(?P=indent)    continue;\n"
    r"(?P=indent)int checkIndex = FlowFieldUtils\.GetFlatIndex\(checkCell, GridDimensions\);\n"
    r"(?P=indent)if \(Grid\[checkIndex\]\.Cost != 0\)\n"
    r"(?P=indent)    continue;"
)

def replace_blocked(match: re.Match[str]) -> str:
    indent = match.group("indent")
    return (
        f"{indent}if (!GridObstacleView.IsBlocked(\n"
        f"{indent}        Grid, obstacleGeometry, checkCell))\n"
        f"{indent}    continue;"
    )

source, blocked_count = blocked_block.subn(replace_blocked, source)

center_block = re.compile(
    r"(?P<indent>\s*)float3 wallPosition = GridOrigin \+ new float3\(\n"
    r"(?P=indent)    checkCell\.x \* CellRadius \* 2f \+ CellRadius,\n"
    r"(?P=indent)    (?P<height>[^\n]+),\n"
    r"(?P=indent)    checkCell\.y \* CellRadius \* 2f \+ CellRadius\);"
)

def replace_center(match: re.Match[str]) -> str:
    indent = match.group("indent")
    height = match.group("height").strip()
    return (
        f"{indent}float3 wallPosition = GridObstacleView.CellCenter(\n"
        f"{indent}    obstacleGeometry, checkCell, {height});"
    )

source, center_count = center_block.subn(replace_center, source)

if source == original:
    # Idempotent reruns are valid only when every old dependency is gone.
    forbidden = ("state.Cell.Cost", "Grid[checkIndex].Cost")
    remaining = [token for token in forbidden if token in source]
    if remaining:
        raise SystemExit(f"Migration made no changes; remaining patterns: {remaining}")
    print("Parallel environment migration already applied")
else:
    if "state.Cell.Cost" in source or "Grid[checkIndex].Cost" in source:
        raise SystemExit("Not all direct FlowField cost reads were migrated")
    if geometry_count < 2 or blocked_count < 2 or center_count < 2:
        raise SystemExit(
            "Unexpected P1-P6 source shape: "
            f"geometry={geometry_count}, blocked={blocked_count}, center={center_count}"
        )
    path.write_text(source, encoding="utf-8")
    print(
        "Migrated P1-P6 environment semantics: "
        f"geometry={geometry_count}, blocked={blocked_count}, center={center_count}"
    )
