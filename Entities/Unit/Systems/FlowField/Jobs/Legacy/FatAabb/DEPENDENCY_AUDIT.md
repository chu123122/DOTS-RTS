# Legacy Fat AABB dependency audit

Generated from the current branch after physical quarantine.

## Legacy methods referenced outside `Legacy/FatAabb`

| File | Symbol | External references |
|---|---|---:|
| `FatAabbCacheBroadPhase.cs` | `TryFindCurrentBodyIndex` | 14 |
| `FatAabbCacheBroadPhase.cs` | `AabbContains` | 7 |
| `FatAabbCacheBroadPhase.cs` | `SortAndDeduplicateBodyPairs` | 4 |
| `FatAabbCacheBroadPhase.cs` | `MarkCorrectedBody` | 3 |
| `FatAabbCacheBroadPhase.cs` | `PrepareCurrentBodyLookup` | 3 |
| `FatAabbCacheBroadPhase.cs` | `AabbOverlaps` | 2 |
| `FatAabbCacheBroadPhase.cs` | `CalculateNeighborPathBounds` | 2 |
| `FatAabbCacheBroadPhase.cs` | `ResetCorrectedBodyTracking` | 2 |
| `AdaptiveFatAabbHotspot.cs` | `BuildAdaptiveFatAabbHotspots` | 1 |
| `FatAabbCacheBroadPhase.cs` | `TryFindProxy` | 1 |
| `AdaptiveFatAabbHotspot.cs` | `UpdateAdaptiveFatAabbHistoryAfterSolve` | 1 |

## Legacy types referenced outside `Legacy/FatAabb`

| File | Type | External references |
|---|---|---:|
| `AdaptiveFatAabbHybridBroadPhase.cs` | `SolveXpbdUnitContactsJob` | 9 |
| `FatAabbCacheBroadPhase.cs` | `SolveXpbdUnitContactsJob` | 9 |
| `AdaptiveFatAabbHotspot.cs` | `SolveXpbdUnitContactsJob` | 9 |
| `AdaptiveFatAabbTypes.cs` | `AdaptiveFatAabbDebugProxy` | 2 |
| `AdaptiveFatAabbTypes.cs` | `AdaptiveFatAabbRegionHistory` | 2 |
| `AdaptiveFatAabbTypes.cs` | `AdaptiveFatAabbBodyRouting` | 1 |
| `AdaptiveFatAabbTypes.cs` | `AdaptiveFatAabbCacheFeedback` | 1 |
| `AdaptiveFatAabbTypes.cs` | `AdaptiveFatAabbCellHistory` | 1 |
| `AdaptiveFatAabbTypes.cs` | `AdaptiveFatAabbCellMetric` | 1 |
| `AdaptiveFatAabbTypes.cs` | `AdaptiveFatAabbDebugCell` | 1 |
| `AdaptiveFatAabbTypes.cs` | `AdaptiveFatAabbDebugRegion` | 1 |
| `AdaptiveFatAabbTypes.cs` | `AdaptiveFatAabbRegion` | 1 |

## Legacy fields still present on the solver job

| Field/type marker | Occurrences in coordinator |
|---|---:|
| `ShadowCellEntries` | 1 |
| `ShadowBodyPairs` | 1 |
| `ShadowCurrentProxies` | 2 |
| `ShadowCurrentPairs` | 2 |
| `MappedFatCachePairs` | 1 |
| `ShadowPreviousProxies` | 2 |
| `ShadowPreviousPairs` | 2 |
| `FatAabbCacheState` | 3 |
| `AdaptiveSettings` | 1 |
| `AdaptiveCellDimensions` | 1 |
| `AdaptiveCellHistory` | 1 |
| `AdaptiveCellMetrics` | 1 |
| `AdaptiveBodyRouting` | 1 |
| `AdaptiveFloodQueue` | 1 |
| `AdaptiveFloodCells` | 1 |
| `AdaptiveRegions` | 1 |
| `AdaptiveDebugCells` | 1 |
| `AdaptiveDebugRegions` | 1 |
| `AdaptiveDebugProxies` | 1 |
| `AdaptiveRegionHistory` | 1 |
| `AdaptiveRegionHistoryScratch` | 1 |
| `AdaptiveNextRegionId` | 1 |
| `AdaptiveCacheFeedback` | 1 |
| `ShadowStatistics` | 3 |

## Removal rule

1. Move externally referenced neutral helpers into `ContactPipeline/Core` or `ContactPipeline/BroadPhase`.
2. Replace Adaptive-only heat/debug data with pipeline-native diagnostics.
3. Remove legacy runtime fields from `SolveXpbdUnitContactsJob` and `BaseFlowMovementSystem`.
4. Delete `Legacy/FatAabb` only after the external-reference table reaches zero.
