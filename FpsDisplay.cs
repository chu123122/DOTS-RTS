using UnityEngine;

/// <summary>
/// 实时帧数显示——屏幕正上方居中。
/// </summary>
public sealed class FpsDisplay : MonoBehaviour
{
    private float _deltaTime;

    private void Awake()
    {
        DontDestroyOnLoad(gameObject);
    }

    private void Update()
    {
        _deltaTime += (Time.unscaledDeltaTime - _deltaTime) * 0.1f;
    }

    private void OnGUI()
    {
        float fps = 1f / Mathf.Max(0.0001f, _deltaTime);
        float ms = _deltaTime * 1000f;

        GUIStyle style = new GUIStyle(GUI.skin.label)
        {
            fontSize = 18,
            fontStyle = FontStyle.Bold,
            alignment = TextAnchor.UpperCenter,
            normal = { textColor = new Color(0f, 0.85f, 0.3f) }
        };

        var rect = new Rect(0, 6, Screen.width, 30);
        GUI.Label(rect, $"{fps:0.} fps  /  {ms:0.0} ms", style);
    }

    [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.AfterSceneLoad)]
    private static void Create()
    {
        var go = new GameObject("FPS Display", typeof(FpsDisplay))
        {
            hideFlags = HideFlags.DontSave
        };
        DontDestroyOnLoad(go);
    }
}
