using UnityEngine;

public interface IGridData
{
    void AddObjectAt(Vector3Int gridPosition, Vector2Int objectSize, int id, int placedObjectIndex);
    bool CanPlaceObjectAt(Vector3Int gridPosition, Vector2Int objectSize);
    void RemoveObjectAt(Vector3Int gridPosition);
    int GetRepresentationIndex(Vector3Int gridPosition);
}