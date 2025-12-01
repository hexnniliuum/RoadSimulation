using System.Collections.Generic;
using UnityEngine;
using System.Linq;

/// <summary>
/// Manages traffic light phases for a single intersection.
/// Groups incoming lanes into non-conflicting phases based on angles.
/// </summary>
public class StopLightController
{
    public Node IntersectionNode;
    
    // Configuration
    private float greenDuration = 10f;
    private float yellowDuration = 3f;
    private float allRedDuration = 2f; // Safety buffer between phases
    
    // State
    private List<StopLightPhase> phases;
    private int currentPhaseIndex = 0;
    private float timer = 0f;
    private LightState currentLightState = LightState.Red;
    
    public enum LightState
    {
        Green,
        Yellow,
        Red
    }
    
    public StopLightController(Node node)
    {
        IntersectionNode = node;
        phases = new List<StopLightPhase>();
        InitializePhases();
    }
    
    private void InitializePhases()
    {
        // 1. Identify all incoming lanes with their road directions and angles
        List<LaneWithDirection> lanesWithDirs = new List<LaneWithDirection>();
        
        foreach (var road in IntersectionNode.ConnectedRoads)
        {
            Vector3 roadDirFromIntersection;
            if (road.StartNode == IntersectionNode)
                roadDirFromIntersection = (road.EndNode.Position - IntersectionNode.Position).normalized;
            else
                roadDirFromIntersection = (road.StartNode.Position - IntersectionNode.Position).normalized;
            
            float angle = Mathf.Atan2(roadDirFromIntersection.x, roadDirFromIntersection.z) * Mathf.Rad2Deg;
            if (angle < 0) angle += 360f;
            
            if (road.ForwardLane.EndNode == IntersectionNode)
            {
                lanesWithDirs.Add(new LaneWithDirection
                {
                    Lane = road.ForwardLane,
                    RoadDirection = roadDirFromIntersection,
                    Angle = angle
                });
            }
            if (road.BackwardLane.EndNode == IntersectionNode)
            {
                lanesWithDirs.Add(new LaneWithDirection
                {
                    Lane = road.BackwardLane,
                    RoadDirection = roadDirFromIntersection,
                    Angle = angle
                });
            }
        }
        
        if (lanesWithDirs.Count == 0) return;
        
        // 2. Group lanes by road direction (same angle = same road = same direction)
        var roadGroups = lanesWithDirs
            .GroupBy(l => Mathf.Round(l.Angle))
            .OrderBy(g => g.Key)
            .ToList();
        
        int numRoads = roadGroups.Count;
        if (numRoads == 0) return;
        
        // 3. Build road adjacency (roads next to each other in angular order are adjacent)
        // roadGroups[i] is adjacent to roadGroups[(i+1) % numRoads] and roadGroups[(i-1+numRoads) % numRoads]
        HashSet<(int, int)> adjacentRoadPairs = new HashSet<(int, int)>();
        for (int i = 0; i < numRoads; i++)
        {
            int next = (i + 1) % numRoads;
            int a = Mathf.Min(i, next);
            int b = Mathf.Max(i, next);
            adjacentRoadPairs.Add((a, b));
        }
        
        // 4. Find best pairing: prioritize roads closest to 180° apart, but NOT adjacent
        List<(int roadA, int roadB, float angleDiff)> possiblePairs = new List<(int, int, float)>();
        
        for (int i = 0; i < numRoads; i++)
        {
            for (int j = i + 1; j < numRoads; j++)
            {
                int a = Mathf.Min(i, j);
                int b = Mathf.Max(i, j);
                
                // Skip if adjacent
                if (adjacentRoadPairs.Contains((a, b))) continue;
                
                float angleA = roadGroups[i].First().Angle;
                float angleB = roadGroups[j].First().Angle;
                float diff = Mathf.Abs(angleA - angleB);
                if (diff > 180f) diff = 360f - diff;
                float deviationFrom180 = Mathf.Abs(diff - 180f);
                
                possiblePairs.Add((i, j, deviationFrom180));
            }
        }
        
        // Sort by closest to 180° (smallest deviation first)
        possiblePairs = possiblePairs.OrderBy(p => p.angleDiff).ToList();
        
        // 5. Greedy assignment: pair roads closest to 180° that aren't adjacent
        HashSet<int> assignedRoads = new HashSet<int>();
        Dictionary<int, int> roadToPhase = new Dictionary<int, int>();
        int currentPhase = 0;
        
        foreach (var pair in possiblePairs)
        {
            if (assignedRoads.Contains(pair.roadA) || assignedRoads.Contains(pair.roadB))
                continue;
            
            roadToPhase[pair.roadA] = currentPhase;
            roadToPhase[pair.roadB] = currentPhase;
            assignedRoads.Add(pair.roadA);
            assignedRoads.Add(pair.roadB);
            currentPhase++;
        }
        
        // 6. Assign remaining unpaired roads to their own phases
        for (int i = 0; i < numRoads; i++)
        {
            if (!assignedRoads.Contains(i))
            {
                roadToPhase[i] = currentPhase;
                currentPhase++;
            }
        }
        
        // 7. Build phases from road assignments (include all lanes from each road)
        int maxPhase = roadToPhase.Values.Max();
        for (int p = 0; p <= maxPhase; p++)
        {
            StopLightPhase phase = new StopLightPhase();
            foreach (var kvp in roadToPhase)
            {
                if (kvp.Value == p)
                {
                    foreach (var lwd in roadGroups[kvp.Key])
                    {
                        phase.Lanes.Add(lwd.Lane);
                    }
                }
            }
            if (phase.Lanes.Count > 0)
            {
                phases.Add(phase);
            }
        }
    }
    
    private struct LaneWithDirection
    {
        public Lane Lane;
        public Vector3 RoadDirection;
        public float Angle;
    }
    
    public void Update(float deltaTime)
    {
        if (phases.Count == 0) return;
        
        timer += deltaTime;
        
        switch (currentLightState)
        {
            case LightState.Green:
                if (timer >= greenDuration)
                {
                    currentLightState = LightState.Yellow;
                    timer = 0f;
                }
                break;
                
            case LightState.Yellow:
                if (timer >= yellowDuration)
                {
                    currentLightState = LightState.Red;
                    timer = 0f;
                }
                break;
                
            case LightState.Red:
                if (timer >= allRedDuration)
                {
                    // Advance to next phase
                    currentPhaseIndex = (currentPhaseIndex + 1) % phases.Count;
                    currentLightState = LightState.Green;
                    timer = 0f;
                }
                break;
        }
    }
    
    public LightState GetLightState(Lane incomingLane)
    {
        if (phases.Count == 0) return LightState.Green; // Default to green if no logic
        
        // Check if the lane is in the current active phase
        if (phases[currentPhaseIndex].Lanes.Contains(incomingLane))
        {
            return currentLightState;
        }
        
        // If not in active phase, it's Red
        return LightState.Red;
    }
    
    private class StopLightPhase
    {
        public List<Lane> Lanes = new List<Lane>();
    }
}
