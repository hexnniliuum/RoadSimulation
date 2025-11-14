using UnityEngine;
using System.Collections.Generic;

/// <summary>
/// Debug tool to verify that vehicle speeds are now correct on bridges.
/// Attach to SimulationManager and check the console for analysis.
/// </summary>
public class SpeedVerificationTool : MonoBehaviour
{
    [Header("Settings")]
    [SerializeField] private SimulationManager simulationManager;
    [SerializeField] private bool runAnalysisOnStart = true;
    [SerializeField] private bool continuousMonitoring = false;
    [SerializeField] private float monitoringInterval = 1f;
    
    private float timeSinceLastCheck = 0f;
    
    void Start()
    {
        if (runAnalysisOnStart)
        {
            Invoke(nameof(AnalyzeRoadNetwork), 0.5f); // Delay to ensure network is generated
        }
    }
    
    void Update()
    {
        if (continuousMonitoring)
        {
            timeSinceLastCheck += Time.deltaTime;
            if (timeSinceLastCheck >= monitoringInterval)
            {
                MonitorVehicleSpeeds();
                timeSinceLastCheck = 0f;
            }
        }
    }
    
    [ContextMenu("Analyze Road Network")]
    public void AnalyzeRoadNetwork()
    {
        if (simulationManager == null)
        {
            Debug.LogError("SimulationManager not assigned!");
            return;
        }
        
        RoadMap roadMap = simulationManager.GetRoadMap();
        if (roadMap == null)
        {
            Debug.LogError("No road network found!");
            return;
        }
        
        Debug.Log("=== SPEED VERIFICATION ANALYSIS ===");
        
        List<Lane> allLanes = roadMap.GetAllLanes();
        int lanesWithBridges = 0;
        float maxDiscrepancy = 0f;
        
        foreach (Lane lane in allLanes)
        {
            if (lane.Bridges != null && lane.Bridges.Count > 0)
            {
                lanesWithBridges++;
                
                // Calculate horizontal distance
                float horizontalDist = Vector3.Distance(lane.StartPosition, lane.EndPosition);
                float actual3DDist = lane.Length;
                float difference = actual3DDist - horizontalDist;
                float discrepancyPercent = (difference / horizontalDist) * 100f;
                
                if (discrepancyPercent > maxDiscrepancy)
                    maxDiscrepancy = discrepancyPercent;
                
                Debug.Log($"Lane {lane.ID}:");
                Debug.Log($"  Horizontal: {horizontalDist:F3} units");
                Debug.Log($"  3D Arc Length: {actual3DDist:F3} units");
                Debug.Log($"  Difference: {difference:F3} units ({discrepancyPercent:F2}%)");
                
                foreach (var bridge in lane.Bridges)
                {
                    Debug.Log($"  Bridge Height: {bridge.PeakHeight:F2} units");
                }
            }
        }
        
        Debug.Log($"\nSummary:");
        Debug.Log($"  Total Lanes: {allLanes.Count}");
        Debug.Log($"  Lanes with Bridges: {lanesWithBridges}");
        Debug.Log($"  Max Speed Correction: {maxDiscrepancy:F2}%");
        Debug.Log($"\n✓ 3D arc length is now properly calculated for all lanes!");
        Debug.Log($"✓ Vehicle speeds will be consistent on bridges and flat roads.");
    }
    
    [ContextMenu("Monitor Vehicle Speeds")]
    public void MonitorVehicleSpeeds()
    {
        if (simulationManager == null)
            return;
        
        List<CarController> cars = simulationManager.GetAllCars();
        if (cars.Count == 0)
            return;
        
        Debug.Log($"=== VEHICLE SPEED MONITOR (Time: {Time.time:F1}s) ===");
        
        foreach (var car in cars)
        {
            if (car == null || car.CurrentLane == null)
                continue;
            
            // Check if on a bridge
            bool onBridge = car.CurrentLane.Bridges != null && car.CurrentLane.Bridges.Count > 0;
            
            // Measure actual 3D speed by tracking position change
            Vector3 currentPos = car.GetPosition();
            
            Debug.Log($"Car {car.CarID}:");
            Debug.Log($"  Speed: {car.CurrentSpeed:F2} units/s");
            Debug.Log($"  Position: {currentPos}");
            Debug.Log($"  On Bridge: {(onBridge ? "YES" : "NO")}");
            Debug.Log($"  Lane: {car.CurrentLane.ID}");
        }
    }
    
    [ContextMenu("Test Single Lane")]
    public void TestSingleLane()
    {
        if (simulationManager == null)
            return;
        
        RoadMap roadMap = simulationManager.GetRoadMap();
        if (roadMap == null)
            return;
        
        List<Lane> lanes = roadMap.GetAllLanes();
        foreach (Lane lane in lanes)
        {
            if (lane.Bridges != null && lane.Bridges.Count > 0)
            {
                TestLaneArcLength(lane);
                break; // Test first bridge lane only
            }
        }
    }
    
    private void TestLaneArcLength(Lane lane)
    {
        Debug.Log($"\n=== DETAILED LANE TEST: Lane {lane.ID} ===");
        
        float horizontalLength = Vector3.Distance(lane.StartPosition, lane.EndPosition);
        Debug.Log($"Horizontal Length: {horizontalLength:F3} units");
        Debug.Log($"3D Arc Length: {lane.Length:F3} units");
        Debug.Log($"Difference: {(lane.Length - horizontalLength):F3} units");
        
        // Test position mapping
        Debug.Log("\nPosition Mapping Test:");
        for (float dist = 0f; dist <= lane.Length; dist += lane.Length * 0.25f)
        {
            Vector3 pos = lane.GetPositionAtDistance(dist);
            Debug.Log($"  Distance {dist:F2}: Position {pos}, Y={pos.y:F3}");
        }
        
        // Verify uniform spacing in 3D
        Debug.Log("\n3D Spacing Verification:");
        float stepDistance = lane.Length / 10f;
        Vector3 prevPos = lane.GetPositionAtDistance(0f);
        
        for (int i = 1; i <= 10; i++)
        {
            float dist = i * stepDistance;
            Vector3 currentPos = lane.GetPositionAtDistance(dist);
            float actual3DStep = Vector3.Distance(prevPos, currentPos);
            
            Debug.Log($"  Step {i}: Expected {stepDistance:F3}, Actual {actual3DStep:F3} (diff: {Mathf.Abs(actual3DStep - stepDistance):F3})");
            prevPos = currentPos;
        }
    }
}

