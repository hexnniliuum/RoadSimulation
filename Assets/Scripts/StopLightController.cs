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
        // 1. Identify all incoming lanes
        List<Lane> incomingLanes = new List<Lane>();
        foreach (var road in IntersectionNode.ConnectedRoads)
        {
            if (road.ForwardLane.EndNode == IntersectionNode) incomingLanes.Add(road.ForwardLane);
            if (road.BackwardLane.EndNode == IntersectionNode) incomingLanes.Add(road.BackwardLane);
        }
        
        // 2. Group lanes into phases
        // Strategy: Group lanes that are roughly opposite (within ~30 degrees of 180)
        // and not already assigned.
        
        HashSet<Lane> assignedLanes = new HashSet<Lane>();
        
        // Sort lanes by angle to make processing deterministic
        // Use standard angle from (1,0)
        var lanesWithAngles = incomingLanes.Select(l => new { 
            Lane = l, 
            Angle = Mathf.Atan2(l.GetDirection().z, l.GetDirection().x) * Mathf.Rad2Deg 
        }).OrderBy(x => x.Angle).ToList();
        
        foreach (var item in lanesWithAngles)
        {
            Lane lane = item.Lane;
            if (assignedLanes.Contains(lane)) continue;
            
            // Start a new phase with this lane
            StopLightPhase phase = new StopLightPhase();
            phase.Lanes.Add(lane);
            assignedLanes.Add(lane);
            
            // Find an opposite lane to pair with
            Vector3 dir = lane.GetDirection();
            
            foreach (var otherItem in lanesWithAngles)
            {
                Lane otherLane = otherItem.Lane;
                if (assignedLanes.Contains(otherLane)) continue;
                
                Vector3 otherDir = otherLane.GetDirection();
                
                // Check if opposite (dot product close to -1)
                // Allow some variance (e.g., > 150 degrees separation)
                float dot = Vector3.Dot(dir, otherDir);
                if (dot < -0.866f) // cos(150) approx -0.866
                {
                    // Found a pair!
                    phase.Lanes.Add(otherLane);
                    assignedLanes.Add(otherLane);
                    break; // Only pair one opposite lane per phase for simplicity
                }
            }
            
            phases.Add(phase);
        }
        
        // If no phases created (shouldn't happen if there are roads), create a default one
        if (phases.Count == 0)
        {
            Debug.LogWarning($"StopLightController: No phases created for Node {IntersectionNode.ID}");
        }
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
