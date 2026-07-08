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
    
    private float greenDuration = 10f;
    private float yellowDuration = 3f;
    private float allRedDuration = 2f; // Safety buffer between phases

    // Two roads share a green only within this many degrees of exact opposite (180°). 60°
    // admits skewed "X" junctions (through-pairs as little as ~120° apart) while still soloing
    // perpendicular (~90°) and near-parallel (dev > ~110°) arms. This only decides which roads
    // can share a green - crossing movements within a phase are still serialized by the
    // per-movement turn-conflict map, so it's a throughput knob, not a safety one.
    private const float MAX_PAIRING_DEVIATION = 60f;
    
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
        
        // Round the angle to bucket lanes arriving at the same angle - i.e. from the same road.
        var roadGroups = lanesWithDirs
            .GroupBy(l => Mathf.Round(l.Angle))
            .OrderBy(g => g.Key)
            .ToList();
        
        int numRoads = roadGroups.Count;
        if (numRoads == 0) return;
        
        List<(int roadA, int roadB, float deviationFrom180)> possiblePairs = new List<(int, int, float)>();

        for (int i = 0; i < numRoads; i++)
        {
            for (int j = i + 1; j < numRoads; j++)
            {
                float angleA = roadGroups[i].First().Angle;
                float angleB = roadGroups[j].First().Angle;
                float diff = Mathf.Abs(angleA - angleB);
                if (diff > 180f) diff = 360f - diff;
                float deviationFrom180 = Mathf.Abs(diff - 180f);

                // Only near-opposite roads are eligible to share a phase.
                if (deviationFrom180 > MAX_PAIRING_DEVIATION) continue;

                possiblePairs.Add((i, j, deviationFrom180));
            }
        }

        // Greedy "smallest deviation first" can strand a road whose only eligible partner
        // already paired off, leaving avoidable solo phases; use max matching instead.
        int[] partner = FindBestMatching(numRoads, possiblePairs);

        Dictionary<int, int> roadToPhase = new Dictionary<int, int>();
        int currentPhase = 0;
        bool[] placed = new bool[numRoads];
        for (int i = 0; i < numRoads; i++)
        {
            if (placed[i]) continue;
            roadToPhase[i] = currentPhase;
            placed[i] = true;
            if (partner[i] >= 0)
            {
                roadToPhase[partner[i]] = currentPhase;
                placed[partner[i]] = true;
            }
            currentPhase++;
        }
        
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
    
    /// <summary>
    /// Exhaustive max-cardinality matching over near-opposite pairs. Returns partner[i], or -1
    /// if road i stays solo. numRoads is small (an intersection's arm count), so brute force is cheap.
    /// </summary>
    private int[] FindBestMatching(int numRoads, List<(int roadA, int roadB, float deviationFrom180)> pairs)
    {
        var adj = new Dictionary<int, List<(int other, float dev)>>();
        foreach (var pair in pairs)
        {
            if (!adj.ContainsKey(pair.roadA)) adj[pair.roadA] = new List<(int, float)>();
            if (!adj.ContainsKey(pair.roadB)) adj[pair.roadB] = new List<(int, float)>();
            adj[pair.roadA].Add((pair.roadB, pair.deviationFrom180));
            adj[pair.roadB].Add((pair.roadA, pair.deviationFrom180));
        }

        int[] match = new int[numRoads];
        for (int i = 0; i < numRoads; i++) match[i] = -1;

        int[] best = (int[])match.Clone();
        int bestCount = -1;
        float bestDev = 0f;

        // Walk roads by index so every branch strictly advances and terminates.
        void Recurse(int idx, int count, float dev)
        {
            if (idx >= numRoads)
            {
                if (count > bestCount || (count == bestCount && dev < bestDev))
                {
                    bestCount = count;
                    bestDev = dev;
                    best = (int[])match.Clone();
                }
                return;
            }

            if (match[idx] != -1)
            {
                Recurse(idx + 1, count, dev);
                return;
            }

            if (adj.ContainsKey(idx))
            {
                foreach (var (other, d) in adj[idx])
                {
                    if (other > idx && match[other] == -1)
                    {
                        match[idx] = other;
                        match[other] = idx;
                        Recurse(idx + 1, count + 1, dev + d);
                        match[idx] = -1;
                        match[other] = -1;
                    }
                }
            }

            Recurse(idx + 1, count, dev);
        }

        Recurse(0, 0, 0f);
        return best;
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
                    currentPhaseIndex = (currentPhaseIndex + 1) % phases.Count;
                    currentLightState = LightState.Green;
                    timer = 0f;
                }
                break;
        }
    }
    
    public LightState GetLightState(Lane incomingLane)
    {
        if (phases.Count == 0) return LightState.Green; // No phases (no real intersection here) - default green, not a permanent red
        
        if (phases[currentPhaseIndex].Lanes.Contains(incomingLane))
        {
            return currentLightState;
        }
        
        return LightState.Red;
    }
    
    private class StopLightPhase
    {
        public List<Lane> Lanes = new List<Lane>();
    }
}
