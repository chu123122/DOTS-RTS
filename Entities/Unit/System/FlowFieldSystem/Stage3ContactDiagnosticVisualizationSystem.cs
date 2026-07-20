using System.Text;
using Unity.Entities;
using Unity.Mathematics;
using Unity.Physics;
using Unity.Transforms;
using UnityEngine;
using UnityEngine.InputSystem;
using 客户端;
using 通用;
using RaycastHit = Unity.Physics.RaycastHit;

[WorldSystemFilter(WorldSystemFilterFlags.ClientSimulation | WorldSystemFilterFlags.LocalSimulation)]
[UpdateInGroup(typeof(PresentationSystemGroup))]
public partial class Stage3ContactDiagnosticVisualizationSystem : SystemBase
{
    private const float DrawHeight = 0.18f;
    private Stage3ContactDiagnosticOverlay _overlay;

    protected override void OnCreate()
    {
        RequireForUpdate<UnitContactSolverSettings>();
        RequireForUpdate<PredictiveDiscContactStatistics>();
        RequireForUpdate<ShadowNeighborCacheStatistics>();
        RequireForUpdate<Stage3ContactDiagnosticSelection>();
        RequireForUpdate<Stage3SelectedBodyDiagnostic>();
        RequireForUpdate<PhysicsWorldSingleton>();
        RequireForUpdate<MainCameraTag>();

        var overlayObject = new GameObject($"Stage3 Contact Diagnostic ({World.Name})")
        {
            hideFlags = HideFlags.DontSave
        };
        Object.DontDestroyOnLoad(overlayObject);
        _overlay = overlayObject.AddComponent<Stage3ContactDiagnosticOverlay>();
    }

    protected override void OnDestroy()
    {
        if (_overlay != null)
            Object.Destroy(_overlay.gameObject);
    }

    protected override void OnUpdate()
    {
        RefRW<UnitContactSolverSettings> settingsReference =
            SystemAPI.GetSingletonRW<UnitContactSolverSettings>();
        UnitContactSolverSettings settings = settingsReference.ValueRO;

        Keyboard keyboard = Keyboard.current;
        if (keyboard != null && keyboard.f8Key.wasPressedThisFrame)
            settings.EnableDiagnostics = !settings.EnableDiagnostics;
        if (keyboard != null && keyboard.f9Key.wasPressedThisFrame)
            settings.EnablePredictiveContacts = !settings.EnablePredictiveContacts;
        if (keyboard != null && keyboard.f10Key.wasPressedThisFrame)
            settings.VisualizeSelectedContacts = !settings.VisualizeSelectedContacts;
        if (keyboard != null && keyboard.f11Key.wasPressedThisFrame)
            settings.EnableShadowNeighborCacheTest = !settings.EnableShadowNeighborCacheTest;
        if (keyboard != null && keyboard.pageUpKey.wasPressedThisFrame)
            _overlay.Scale = math.min(2f, _overlay.Scale + 0.1f);
        if (keyboard != null && keyboard.pageDownKey.wasPressedThisFrame)
            _overlay.Scale = math.max(0.8f, _overlay.Scale - 0.1f);
        settingsReference.ValueRW = settings;

        if (settings.EnableDiagnostics && settings.VisualizeSelectedContacts)
            TrySelectDiagnosticUnitWithMiddleMouse();

        bool shouldShowOverlay =
            settings.EnableDiagnostics || settings.EnableShadowNeighborCacheTest;
        _overlay.Visible = shouldShowOverlay;
        if (!shouldShowOverlay)
        {
            _overlay.Text = string.Empty;
            return;
        }

        // Singleton/Buffer API 不会自动完成生产它们的异步 Job。
        // Presentation 需要本帧最终诊断快照，因此在读取前建立明确同步边界。
        EntityManager.CompleteDependencyBeforeRO<PredictiveDiscContactStatistics>();
        EntityManager.CompleteDependencyBeforeRO<ShadowNeighborCacheStatistics>();
        EntityManager.CompleteDependencyBeforeRO<Stage3ContactIterationDiagnostic>();
        EntityManager.CompleteDependencyBeforeRO<Stage3ContactPairDiagnostic>();
        EntityManager.CompleteDependencyBeforeRO<Stage3SelectedBodyDiagnostic>();

        PredictiveDiscContactStatistics statistics =
            SystemAPI.GetSingleton<PredictiveDiscContactStatistics>();
        ShadowNeighborCacheStatistics shadowStatistics =
            SystemAPI.GetSingleton<ShadowNeighborCacheStatistics>();
        DynamicBuffer<Stage3ContactIterationDiagnostic> iterationDiagnostics =
            SystemAPI.GetSingletonBuffer<Stage3ContactIterationDiagnostic>(true);
        DynamicBuffer<Stage3ContactPairDiagnostic> pairDiagnostics =
            SystemAPI.GetSingletonBuffer<Stage3ContactPairDiagnostic>(true);
        Stage3SelectedBodyDiagnostic selectedBody =
            SystemAPI.GetSingleton<Stage3SelectedBodyDiagnostic>();
        Stage3ContactDiagnosticSelection selection =
            SystemAPI.GetSingleton<Stage3ContactDiagnosticSelection>();

        _overlay.Text = BuildOverlayText(
            settings,
            statistics,
            shadowStatistics,
            iterationDiagnostics,
            pairDiagnostics,
            selection.SelectedEntity,
            selectedBody,
            _overlay.Scale);

        if (!settings.EnableDiagnostics ||
            !settings.VisualizeSelectedContacts ||
            selectedBody.IsValid == 0)
            return;

        DrawSelectedSweep(selectedBody);
        for (int i = 0; i < pairDiagnostics.Length; i++)
            DrawPair(pairDiagnostics[i]);
    }

    private void TrySelectDiagnosticUnitWithMiddleMouse()
    {
        Mouse mouse = Mouse.current;
        if (mouse == null || !mouse.middleButton.wasPressedThisFrame)
            return;

        Entity cameraEntity = SystemAPI.GetSingletonEntity<MainCameraTag>();
        Camera camera = EntityManager.GetComponentObject<MainCameraComponents>(cameraEntity).Value;
        if (camera == null)
            return;

        UnityEngine.Ray ray = camera.ScreenPointToRay(mouse.position.ReadValue());
        var raycastInput = new RaycastInput
        {
            Start = ray.origin,
            End = ray.origin + ray.direction * 1000f,
            Filter = new CollisionFilter
            {
                BelongsTo = ~0u,
                CollidesWith = 1u << 1,
                GroupIndex = 0
            }
        };

        CollisionWorld collisionWorld =
            SystemAPI.GetSingleton<PhysicsWorldSingleton>().CollisionWorld;
        if (!collisionWorld.CastRay(raycastInput, out RaycastHit hit) ||
            !EntityManager.HasComponent<BasicUnitTag>(hit.Entity))
            return;

        RefRW<Stage3ContactDiagnosticSelection> selection =
            SystemAPI.GetSingletonRW<Stage3ContactDiagnosticSelection>();
        selection.ValueRW.SelectedEntity = hit.Entity;
    }

    private static void DrawSelectedSweep(Stage3SelectedBodyDiagnostic selected)
    {
        float3 height = new float3(0, DrawHeight, 0);
        float3 start = selected.StartPosition + height;
        float3 predicted = selected.UnconstrainedPredictedPosition + height;
        float3 solved = selected.SolvedPosition + height;
        float sweptRadius = selected.Radius + selected.Skin;

        DrawWireCircle(start, selected.Radius, Color.cyan);
        DrawWireCircle(predicted, selected.Radius, Color.blue);
        DrawWireCircle(solved, selected.Radius, Color.green);
        DrawSweptCapsule(start, predicted, sweptRadius, new Color(0f, 1f, 1f));
        DrawSweptAabb(start, predicted, sweptRadius, Color.white);
        if (selected.ShadowReferenceAvailable != 0)
        {
            DrawAabb(
                selected.ShadowFatMin,
                selected.ShadowFatMax,
                start.y + 0.02f,
                selected.ShadowEscaped != 0 ? Color.red : Color.yellow);
        }

        Debug.DrawLine(start, predicted, Color.blue, 0f, false);
        Debug.DrawLine(predicted, solved, Color.green, 0f, false);
        if (math.lengthsq(selected.WallCorrection) > 0.0000001f)
        {
            Debug.DrawLine(
                solved - selected.WallCorrection,
                solved,
                Color.white,
                0f,
                false);
        }
    }

    private static void DrawAabb(float2 min, float2 max, float height, Color color)
    {
        float3 a = new float3(min.x, height, min.y);
        float3 b = new float3(max.x, height, min.y);
        float3 c = new float3(max.x, height, max.y);
        float3 d = new float3(min.x, height, max.y);
        Debug.DrawLine(a, b, color, 0f, false);
        Debug.DrawLine(b, c, color, 0f, false);
        Debug.DrawLine(c, d, color, 0f, false);
        Debug.DrawLine(d, a, color, 0f, false);
    }

    private static void DrawPair(Stage3ContactPairDiagnostic pair)
    {
        Color color = GetPairColor(pair.Kind, pair.WasActivated != 0);
        float3 height = new float3(0, DrawHeight + 0.02f, 0);
        float3 otherStart = pair.OtherStartPosition + height;
        float3 otherPredicted = pair.OtherPredictedPosition + height;
        float3 selectedClosest = pair.SelectedClosestPosition + height;
        float3 otherClosest = pair.OtherClosestPosition + height;

        Debug.DrawLine(otherStart, otherPredicted, color, 0f, false);
        Debug.DrawLine(selectedClosest, otherClosest, color, 0f, false);
        DrawWireCircle(otherPredicted, pair.OtherRadius, color, 16);
        DrawCross(selectedClosest, color, 0.06f);
        DrawCross(otherClosest, color, 0.06f);
    }

    private static Color GetPairColor(
        Stage3ContactDiagnosticPairKind kind,
        bool activated)
    {
        switch (kind)
        {
            case Stage3ContactDiagnosticPairKind.BroadPhaseRejected:
                return Color.gray;
            case Stage3ContactDiagnosticPairKind.Regular:
                return activated ? new Color(1f, 0.45f, 0f) : Color.yellow;
            case Stage3ContactDiagnosticPairKind.Predictive:
                return activated ? Color.red : Color.magenta;
            case Stage3ContactDiagnosticPairKind.PredictiveDisabled:
                return new Color(0.25f, 0.55f, 1f);
            default:
                return Color.white;
        }
    }

    private static void DrawSweptCapsule(
        float3 start,
        float3 end,
        float radius,
        Color color)
    {
        float2 direction = end.xz - start.xz;
        float lengthSq = math.lengthsq(direction);
        if (lengthSq <= 0.0000001f)
        {
            DrawWireCircle(start, radius, color);
            return;
        }

        float2 perpendicular = math.normalize(new float2(-direction.y, direction.x)) * radius;
        float3 offset = new float3(perpendicular.x, 0, perpendicular.y);
        Debug.DrawLine(start + offset, end + offset, color, 0f, false);
        Debug.DrawLine(start - offset, end - offset, color, 0f, false);
        DrawWireCircle(start, radius, color);
        DrawWireCircle(end, radius, color);
    }

    private static void DrawSweptAabb(
        float3 start,
        float3 end,
        float radius,
        Color color)
    {
        float2 minimum = math.min(start.xz, end.xz) - radius;
        float2 maximum = math.max(start.xz, end.xz) + radius;
        float y = start.y + 0.01f;
        float3 a = new float3(minimum.x, y, minimum.y);
        float3 b = new float3(maximum.x, y, minimum.y);
        float3 c = new float3(maximum.x, y, maximum.y);
        float3 d = new float3(minimum.x, y, maximum.y);
        Debug.DrawLine(a, b, color, 0f, false);
        Debug.DrawLine(b, c, color, 0f, false);
        Debug.DrawLine(c, d, color, 0f, false);
        Debug.DrawLine(d, a, color, 0f, false);
    }

    private static void DrawWireCircle(
        float3 center,
        float radius,
        Color color,
        int segments = 24)
    {
        float angleStep = math.PI * 2f / segments;
        float3 previous = center + new float3(radius, 0, 0);
        for (int i = 1; i <= segments; i++)
        {
            float angle = angleStep * i;
            float3 next = center + new float3(math.cos(angle) * radius, 0, math.sin(angle) * radius);
            Debug.DrawLine(previous, next, color, 0f, false);
            previous = next;
        }
    }

    private static void DrawCross(float3 center, Color color, float radius)
    {
        Debug.DrawLine(
            center - new float3(radius, 0, 0),
            center + new float3(radius, 0, 0),
            color,
            0f,
            false);
        Debug.DrawLine(
            center - new float3(0, 0, radius),
            center + new float3(0, 0, radius),
            color,
            0f,
            false);
    }

    private static string BuildOverlayText(
        UnitContactSolverSettings settings,
        PredictiveDiscContactStatistics statistics,
        ShadowNeighborCacheStatistics shadow,
        DynamicBuffer<Stage3ContactIterationDiagnostic> iterations,
        DynamicBuffer<Stage3ContactPairDiagnostic> selectedPairs,
        Entity selectedEntity,
        Stage3SelectedBodyDiagnostic selectedBody,
        float overlayScale)
    {
        var text = new StringBuilder(1024);
        text.AppendLine("Stage 3 Contact / Shadow Cache Diagnostic");
        text.Append("F8 Diagnostic: ").Append(settings.EnableDiagnostics ? "ON" : "OFF");
        text.Append("    F9 Predictive: ").AppendLine(settings.EnablePredictiveContacts ? "ON" : "OFF");
        text.Append("F10 World lines: ")
            .AppendLine(settings.VisualizeSelectedContacts ? "ON" : "OFF");
        text.Append("F11 Shadow cache: ")
            .Append(settings.EnableShadowNeighborCacheTest ? "ON" : "OFF")
            .Append("    margin: ")
            .AppendLine(settings.ShadowCacheMargin.ToString("F3"));
        text.Append("PageUp/PageDown panel scale: ")
            .Append(overlayScale.ToString("F1")).AppendLine("x");

        text.AppendLine("[Authoritative Predictive Contact Solver]");
        text.Append("Substeps / Iterations: ").Append(settings.SubstepCount).Append(" / ")
            .AppendLine(settings.IterationCount.ToString());
        text.Append("Pairs candidate/contact/potentialPredictive/predictive: ")
            .Append(statistics.CandidatePairCount).Append(" / ")
            .Append(statistics.ContactPairCount).Append(" / ")
            .Append(statistics.PotentialPredictivePairCount).Append(" / ")
            .AppendLine(statistics.PredictivePairCount.ToString());
        text.Append("Active / predictive active: ")
            .Append(statistics.ActiveConstraintCount).Append(" / ")
            .AppendLine(statistics.PredictiveActivatedCount.ToString());
        text.Append("Final penetration max/avg: ")
            .Append(statistics.MaxPenetration.ToString("F5")).Append(" / ")
            .AppendLine(statistics.AveragePenetration.ToString("F5"));
        text.Append("Contact correction total/max: ")
            .Append(statistics.TotalContactPositionCorrection.ToString("F4")).Append(" / ")
            .AppendLine(statistics.MaxContactPositionCorrection.ToString("F4"));
        text.Append("Wall correction total/max: ")
            .Append(statistics.TotalWallPositionCorrection.ToString("F4")).Append(" / ")
            .AppendLine(statistics.MaxWallPositionCorrection.ToString("F4"));
        text.Append("Speed before/after; max delta: ")
            .Append(statistics.AverageSpeedBeforeContact.ToString("F3")).Append(" / ")
            .Append(statistics.AverageSpeedAfterContact.ToString("F3")).Append("; ")
            .AppendLine(statistics.MaxVelocityChange.ToString("F3"));

        if (settings.EnableShadowNeighborCacheTest)
        {
            text.AppendLine("[Shadow Fat-AABB Cache Probe - does not affect solving]");
            text.Append("Shadow previous-frame ready/bodies/pairs/checks: ")
                .Append(shadow.PreviousFrameCacheAvailable).Append(" / ")
                .Append(shadow.PreviousFrameCacheBodyCount).Append(" / ")
                .Append(shadow.PreviousFrameCachePairCount).Append(" / ")
                .AppendLine(shadow.PreviousFrameCheckCount.ToString());
            text.Append("Shadow prev pair hit/miss; active/predictive miss: ")
                .Append(shadow.PreviousFramePairHitCount).Append(" / ")
                .Append(shadow.PreviousFramePairMissCount).Append("; ")
                .Append(shadow.PreviousFrameActivePairMissCount).Append(" / ")
                .AppendLine(shadow.PreviousFramePredictivePairMissCount.ToString());
            text.Append("Shadow first-substep cache bodies/pairs/later-checks: ")
                .Append(shadow.CurrentFrameCacheBodyCount).Append(" / ")
                .Append(shadow.CurrentFrameCachePairCount).Append(" / ")
                .AppendLine(shadow.CurrentFrameCheckCount.ToString());
            text.Append("Shadow current pair hit/miss; active/predictive miss: ")
                .Append(shadow.CurrentFramePairHitCount).Append(" / ")
                .Append(shadow.CurrentFramePairMissCount).Append("; ")
                .Append(shadow.CurrentFrameActivePairMissCount).Append(" / ")
                .AppendLine(shadow.CurrentFramePredictivePairMissCount.ToString());
            text.Append("Shadow escape pre/final/contact/wall (prev | current): ")
                .Append(shadow.PreviousFramePreSolveEscapeBodyCount).Append("/")
                .Append(shadow.PreviousFrameFinalEscapeBodyCount).Append("/")
                .Append(shadow.PreviousFrameContactDrivenEscapeBodyCount).Append("/")
                .Append(shadow.PreviousFrameWallDrivenEscapeBodyCount).Append(" | ")
                .Append(shadow.CurrentFramePreSolveEscapeBodyCount).Append("/")
                .Append(shadow.CurrentFrameFinalEscapeBodyCount).Append("/")
                .Append(shadow.CurrentFrameContactDrivenEscapeBodyCount).Append("/")
                .AppendLine(shadow.CurrentFrameWallDrivenEscapeBodyCount.ToString());
            text.Append("Shadow build/validate ns: ")
                .Append(shadow.CacheBuildNanoseconds).Append(" / ")
                .AppendLine(shadow.ValidationNanoseconds.ToString());
        }

        if (iterations.Length > 0)
        {
            Stage3ContactIterationDiagnostic last = iterations[iterations.Length - 1];
            text.Append("Last residual max/avg: ")
                .Append(last.MaxConstraintViolation.ToString("F5")).Append(" / ")
                .AppendLine(last.AverageConstraintViolation.ToString("F5"));

            text.Append("Last substep residual curve: ");
            int displayed = 0;
            for (int i = 0; i < iterations.Length && displayed < 12; i++)
            {
                Stage3ContactIterationDiagnostic iteration = iterations[i];
                if (iteration.SubstepIndex != last.SubstepIndex)
                    continue;

                if (displayed > 0)
                    text.Append(" > ");
                text.Append(iteration.MaxConstraintViolation.ToString("F4"));
                displayed++;
            }
            if (displayed < settings.IterationCount)
                text.Append(" ...");
            text.AppendLine();
        }

        int broadOnly = 0;
        int regular = 0;
        int predictive = 0;
        int predictiveDisabled = 0;
        for (int i = 0; i < selectedPairs.Length; i++)
        {
            switch (selectedPairs[i].Kind)
            {
                case Stage3ContactDiagnosticPairKind.BroadPhaseRejected:
                    broadOnly++;
                    break;
                case Stage3ContactDiagnosticPairKind.Regular:
                    regular++;
                    break;
                case Stage3ContactDiagnosticPairKind.Predictive:
                    predictive++;
                    break;
                case Stage3ContactDiagnosticPairKind.PredictiveDisabled:
                    predictiveDisabled++;
                    break;
            }
        }

        text.Append("Selected: ").Append(selectedEntity == Entity.Null
            ? "none"
            : $"{selectedEntity.Index}:{selectedEntity.Version}");
        text.Append("    shown pairs: ").AppendLine(selectedPairs.Length.ToString());
        text.Append("Selected broad/regular/predictive/disabled: ")
            .Append(broadOnly).Append(" / ")
            .Append(regular).Append(" / ")
            .Append(predictive).Append(" / ")
            .AppendLine(predictiveDisabled.ToString());
        if (selectedBody.IsValid != 0)
        {
            text.Append("Selected contact delta / wall delta: ")
                .Append(math.length(selectedBody.ContactCorrection).ToString("F4"))
                .Append(" / ")
                .AppendLine(math.length(selectedBody.WallCorrection).ToString("F4"));
        }

        text.AppendLine("Middle click: select diagnostic unit");
        text.AppendLine("Gray broad-only | Yellow/Orange regular | Magenta/Red predictive | Blue disabled");
        text.Append("Cyan sweep | White swept AABB | Yellow/Red shadow fat AABB | Blue predicted disc | Green solved disc");
        return text.ToString();
    }
}

public sealed class Stage3ContactDiagnosticOverlay : MonoBehaviour
{
    public bool Visible;
    public string Text = string.Empty;
    public float Scale = 1.25f;

    private GUIStyle _labelStyle;

    private void OnGUI()
    {
        if (!Visible)
            return;

        _labelStyle ??= new GUIStyle(GUI.skin.label)
        {
            normal = { textColor = Color.white }
        };
        _labelStyle.fontSize = 15;

        const float logicalWidth = 900f;
        float logicalHeight = math.max(
            455f,
            _labelStyle.CalcHeight(new GUIContent(Text), logicalWidth - 36f) + 40f);
        float fitScale = math.min(
            (Screen.width - 24f) / logicalWidth,
            (Screen.height - 24f) / logicalHeight);
        float effectiveScale = math.clamp(math.min(Scale, fitScale), 0.5f, 2f);
        Matrix4x4 previousMatrix = GUI.matrix;
        GUI.matrix = Matrix4x4.Scale(new Vector3(effectiveScale, effectiveScale, 1f));
        GUI.Box(new Rect(12, 12, logicalWidth, logicalHeight), GUIContent.none);
        GUI.Label(new Rect(24, 20, logicalWidth - 36f, logicalHeight - 28f), Text, _labelStyle);
        GUI.matrix = previousMatrix;
    }
}
