using UnityEngine;

/// <summary>
/// Visualizes the road network using Unity Gizmos for debugging.
/// </summary>
[RequireComponent(typeof(RoadNetworkGenerator))]
public class RoadNetworkVisualizer : MonoBehaviour
{
    [Header("Visualization Settings")]
    [SerializeField] private bool showNodes = true;
    [SerializeField] private bool showRoads = true;
    [SerializeField] private bool showLanes = true;
    [SerializeField] private bool showTurnCurves = true;
    [SerializeField] private bool showMetrics = true;
    
    [Header("Colors")]
    [SerializeField] private Color nodeColor = Color.yellow;
    [SerializeField] private Color roadColor = Color.white;
    [SerializeField] private Color laneColor = Color.cyan;
    [SerializeField] private Color turnCurveColor = Color.green;
    
    [Header("Sizes")]
    [SerializeField] private float nodeSize = 1f;

    private RoadNetworkGenerator generator;
    private RoadMap roadMap;

    private void Start()
    {
        generator = GetComponent<RoadNetworkGenerator>();
    }

    private void OnDrawGizmos()
    {
        if (generator == null)
            generator = GetComponent<RoadNetworkGenerator>();
        
        roadMap = generator?.GetRoadMap();
        
        if (roadMap == null || roadMap.Nodes.Count == 0)
            return;

        if (showNodes)
            DrawNodes();
        
        if (showRoads)
            DrawRoads();
        
        if (showLanes)
            DrawLanes();
        
        if (showTurnCurves)
            DrawTurnCurves();
    }

    private void DrawNodes()
    {
        Gizmos.color = nodeColor;
        foreach (Node node in roadMap.Nodes)
        {
            Gizmos.DrawSphere(node.Position, nodeSize);
            
            if (showMetrics)
            {
                #if UNITY_EDITOR
                UnityEditor.Handles.Label(node.Position + Vector3.up * 3f, $"Node {node.ID}");
                #endif
            }
        }
    }

    private void DrawRoads()
    {
        Gizmos.color = roadColor;
        foreach (Road road in roadMap.Roads)
        {
            Gizmos.DrawLine(road.StartNode.Position, road.EndNode.Position);
            
            if (showMetrics)
            {
                Vector3 midPoint = (road.StartNode.Position + road.EndNode.Position) * 0.5f;
                #if UNITY_EDITOR
                UnityEditor.Handles.Label(midPoint, $"L:{road.Length:F1}");
                #endif
            }
        }
    }

    private void DrawLanes()
    {
        Gizmos.color = laneColor;
        foreach (Road road in roadMap.Roads)
        {
            // Forward lane
            DrawLaneWithBridge(road.ForwardLane);
            DrawArrow(road.ForwardLane.StartPosition, road.ForwardLane.EndPosition, 2f);
            
            // Backward lane
            DrawLaneWithBridge(road.BackwardLane);
            DrawArrow(road.BackwardLane.StartPosition, road.BackwardLane.EndPosition, 2f);
        }
    }
    
    private void DrawLaneWithBridge(Lane lane)
    {
        if (lane.Bridges == null || lane.Bridges.Count == 0)
        {
            // No bridge - draw simple line
            Gizmos.DrawLine(lane.StartPosition, lane.EndPosition);
        }
        else
        {
            // Has bridges - draw segmented curve
            int segments = 20;
            Vector3 prevPoint = lane.GetPositionAtDistance(0f);
            float prevDistance = 0f;
            
            for (int i = 1; i <= segments; i++)
            {
                float distance = (i / (float)segments) * lane.Length;
                Vector3 point = lane.GetPositionAtDistance(distance);
                
                // Check if segment is above floor level (on a bridge)
                // Use average Y of both points for segment color
                float avgY = (prevPoint.y + point.y) * 0.5f;
                float floorHeight = generator?.GetConfig()?.FloorHeight ?? 0f;
                bool isOnBridge = avgY > floorHeight + 0.1f; // 0.1 tolerance for floating point
                
                if (isOnBridge)
                {
                    Gizmos.color = Color.magenta; // Bridge segments in magenta
                }
                else
                {
                    Gizmos.color = laneColor; // Normal lane color
                }
                
                Gizmos.DrawLine(prevPoint, point);
                prevPoint = point;
                prevDistance = distance;
            }
            
            // Reset color
            Gizmos.color = laneColor;
        }
    }
    

    private void DrawTurnCurves()
    {
        foreach (Road road in roadMap.Roads)
        {
            DrawCurvesForLane(road.ForwardLane);
            DrawCurvesForLane(road.BackwardLane);
        }
    }

    private void DrawCurvesForLane(Lane lane)
    {
        foreach (TurnCurve curve in lane.AvailableTurns)
        {
            // Color-code by turn type
            switch (curve.Type)
            {
                case TurnCurve.TurnType.Straight:
                    Gizmos.color = Color.green;
                    break;
                case TurnCurve.TurnType.Left:
                    Gizmos.color = Color.blue;
                    break;
                case TurnCurve.TurnType.Right:
                    Gizmos.color = Color.red;
                    break;
            }
            
            // Draw curve with multiple segments
            int segments = 15;
            Vector3 prevPoint = curve.GetPointOnCurve(0f);
            
            for (int i = 1; i <= segments; i++)
            {
                float t = i / (float)segments;
                Vector3 point = curve.GetPointOnCurve(t);
                Gizmos.DrawLine(prevPoint, point);
                prevPoint = point;
            }
        }
    }

    private void DrawArrow(Vector3 start, Vector3 end, float arrowHeadLength)
    {
        Vector3 direction = (end - start).normalized;
        Vector3 midPoint = (start + end) * 0.5f;
        
        // Right vector on XZ plane
        Vector3 right = new Vector3(-direction.z, 0, direction.x);
        Vector3 arrowTip = midPoint + direction * arrowHeadLength;
        Vector3 arrowLeft = midPoint - direction * arrowHeadLength * 0.5f + right * arrowHeadLength * 0.5f;
        Vector3 arrowRight = midPoint - direction * arrowHeadLength * 0.5f - right * arrowHeadLength * 0.5f;
        
        Gizmos.DrawLine(arrowTip, arrowLeft);
        Gizmos.DrawLine(arrowTip, arrowRight);
    }

    public void UpdateVisualization()
    {
        roadMap = generator?.GetRoadMap();
    }
}

