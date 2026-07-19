using Unity.Entities;
using Unity.Mathematics;
using UnityEngine;
using UnityEngine.Rendering;

namespace Entities.Unit.System.FlowFieldSystem
{
    /// <summary>
    /// 将完整流场快照编码为一张纹理，并通过单个世界空间网格显示。
    /// 纹理只在 ActiveVersion 变化时更新，普通帧不遍历网格。
    /// </summary>
    [WorldSystemFilter(WorldSystemFilterFlags.ClientSimulation | WorldSystemFilterFlags.LocalSimulation)]
    [UpdateInGroup(typeof(PresentationSystemGroup))]
    public partial class FlowFieldDebugSystem : SystemBase
    {
        private const string RendererName = "Flow Field Visualization";

        private GameObject _rendererObject;
        private Mesh _mesh;
        private Material _material;
        private Texture2D _texture;
        private uint _lastRenderedVersion = uint.MaxValue;
        private int2 _lastGridDimensions;
        private int _lastPixelsPerCell;

        protected override void OnCreate()
        {
            RequireForUpdate<FlowFieldGrid>();
            RequireForUpdate<FlowFieldRuntimeState>();
            RequireForUpdate<FlowFieldVisualizationSettings>();
        }

        protected override void OnUpdate()
        {
            FlowFieldVisualizationSettings settings = SystemAPI.GetSingleton<FlowFieldVisualizationSettings>();
            EnsureRenderer();
            if (_rendererObject == null) return;

            _rendererObject.SetActive(settings.Visible);
            if (!settings.Visible) return;

            FlowFieldGrid grid = SystemAPI.GetSingleton<FlowFieldGrid>();
            if (!grid.Grid.IsCreated) return;

            uint activeVersion = SystemAPI.GetSingleton<FlowFieldRuntimeState>().ActiveVersion;
            int pixelsPerCell = Mathf.Clamp(settings.PixelsPerCell, 4, 16);
            if (activeVersion == _lastRenderedVersion &&
                grid.GridDimensions.Equals(_lastGridDimensions) &&
                pixelsPerCell == _lastPixelsPerCell)
                return;

            RebuildTexture(grid, settings, pixelsPerCell);
            UpdateMesh(grid, settings.HeightOffset);

            _lastRenderedVersion = activeVersion;
            _lastGridDimensions = grid.GridDimensions;
            _lastPixelsPerCell = pixelsPerCell;
        }

        private void EnsureRenderer()
        {
            if (_rendererObject != null) return;

            Shader shader = Shader.Find("Sprites/Default") ?? Shader.Find("Unlit/Texture");
            if (shader == null) return;

            _rendererObject = new GameObject(RendererName)
            {
                hideFlags = HideFlags.DontSave
            };

            var meshFilter = _rendererObject.AddComponent<MeshFilter>();
            var meshRenderer = _rendererObject.AddComponent<MeshRenderer>();

            _mesh = new Mesh
            {
                name = RendererName,
                hideFlags = HideFlags.DontSave
            };
            meshFilter.sharedMesh = _mesh;

            _material = new Material(shader)
            {
                name = RendererName,
                hideFlags = HideFlags.DontSave
            };
            meshRenderer.sharedMaterial = _material;
            meshRenderer.shadowCastingMode = ShadowCastingMode.Off;
            meshRenderer.receiveShadows = false;
        }

        private void RebuildTexture(
            in FlowFieldGrid grid,
            in FlowFieldVisualizationSettings settings,
            int pixelsPerCell)
        {
            int width = grid.GridDimensions.x * pixelsPerCell;
            int height = grid.GridDimensions.y * pixelsPerCell;
            EnsureTexture(width, height);

            var pixels = new Color32[width * height];
            byte opacity = (byte)math.round(math.saturate(settings.Opacity) * 255f);

            for (int y = 0; y < grid.GridDimensions.y; y++)
            {
                for (int x = 0; x < grid.GridDimensions.x; x++)
                {
                    int cellIndex = FlowFieldUtils.GetFlatIndex(new int2(x, y), grid.GridDimensions);
                    FlowFieldCell cell = grid.Grid[cellIndex];
                    FillCell(pixels, width, x, y, pixelsPerCell, cell, settings, opacity);
                }
            }

            _texture.SetPixels32(pixels);
            _texture.Apply(false, false);
            _material.mainTexture = _texture;
        }

        private void EnsureTexture(int width, int height)
        {
            if (_texture != null && _texture.width == width && _texture.height == height) return;

            DestroyObject(_texture);
            _texture = new Texture2D(width, height, TextureFormat.RGBA32, false, true)
            {
                name = RendererName,
                filterMode = FilterMode.Point,
                wrapMode = TextureWrapMode.Clamp,
                hideFlags = HideFlags.DontSave
            };
        }

        private static void FillCell(
            Color32[] pixels,
            int textureWidth,
            int cellX,
            int cellY,
            int pixelsPerCell,
            in FlowFieldCell cell,
            in FlowFieldVisualizationSettings settings,
            byte opacity)
        {
            Color32 background;
            if (settings.ShowCost && cell.Cost == 0)
                background = new Color32(220, 45, 45, opacity);
            else if (cell.IntegrationValue == 0)
                background = new Color32(40, 220, 90, opacity);
            else
                background = new Color32(30, 135, 220, (byte)(opacity / 4));

            int startX = cellX * pixelsPerCell;
            int startY = cellY * pixelsPerCell;
            for (int y = 0; y < pixelsPerCell; y++)
            {
                for (int x = 0; x < pixelsPerCell; x++)
                {
                    bool border = x == 0 || y == 0;
                    pixels[(startY + y) * textureWidth + startX + x] = border
                        ? new Color32(20, 35, 45, (byte)math.min(255, opacity + 40))
                        : background;
                }
            }

            if (!settings.ShowDirections || cell.BestDirectionIndex == 0xFF || cell.Cost == 0) return;

            int2 direction = FlowFieldUtils.GetDirectionOffset(cell.BestDirectionIndex);
            int2 center = new int2(startX + pixelsPerCell / 2, startY + pixelsPerCell / 2);
            int lineLength = math.max(1, pixelsPerCell / 3);
            int2 start = center - direction * lineLength;
            int2 end = center + direction * lineLength;
            DrawLine(pixels, textureWidth, start, end, new Color32(245, 245, 245, opacity));
        }

        private static void DrawLine(Color32[] pixels, int width, int2 start, int2 end, Color32 color)
        {
            int dx = math.abs(end.x - start.x);
            int dy = -math.abs(end.y - start.y);
            int stepX = start.x < end.x ? 1 : -1;
            int stepY = start.y < end.y ? 1 : -1;
            int error = dx + dy;
            int2 current = start;

            while (true)
            {
                int index = current.y * width + current.x;
                if ((uint)index < (uint)pixels.Length) pixels[index] = color;
                if (current.Equals(end)) break;

                int doubledError = error * 2;
                if (doubledError >= dy)
                {
                    error += dy;
                    current.x += stepX;
                }

                if (doubledError <= dx)
                {
                    error += dx;
                    current.y += stepY;
                }
            }
        }

        private void UpdateMesh(in FlowFieldGrid grid, float heightOffset)
        {
            float width = grid.GridDimensions.x * grid.CellRadius * 2f;
            float height = grid.GridDimensions.y * grid.CellRadius * 2f;

            _mesh.Clear();
            _mesh.vertices = new[]
            {
                new Vector3(0f, 0f, 0f),
                new Vector3(width, 0f, 0f),
                new Vector3(0f, 0f, height),
                new Vector3(width, 0f, height)
            };
            _mesh.uv = new[]
            {
                new Vector2(0f, 0f),
                new Vector2(1f, 0f),
                new Vector2(0f, 1f),
                new Vector2(1f, 1f)
            };
            _mesh.triangles = new[] { 0, 2, 1, 2, 3, 1 };
            _mesh.RecalculateBounds();

            _rendererObject.transform.position = new Vector3(
                grid.GridOrigin.x,
                grid.GridOrigin.y + heightOffset,
                grid.GridOrigin.z);
        }

        protected override void OnDestroy()
        {
            DestroyObject(_rendererObject);
            DestroyObject(_mesh);
            DestroyObject(_material);
            DestroyObject(_texture);
        }

        private static void DestroyObject(Object value)
        {
            if (value == null) return;
            if (Application.isPlaying)
                Object.Destroy(value);
            else
                Object.DestroyImmediate(value);
        }
    }
}
