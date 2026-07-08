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

    // Two roads may share a green phase only if they are within this many degrees of a
    // straight 180° opposite (i.e. at least 180 - this apart). Beyond it a road counts as
    // having "no opposite" and phases alone. At 60° this admits bent through-roads whose
    // arms are as little as 120° apart (a skewed "X" whose second through-pair sits ~57°
    // off opposite), while still soloing genuinely perpendicular arms (~90° apart, which
    // land in the 60-120° dev band) and near-parallel arms (dev > ~110°). Pairing only
    // affects which approaches share a green; within a phase the per-movement turn-conflict
    // map still serializes any crossing movements, so this is a throughput/appearance knob,
    // not a safety one.
    private const float MAX_PAIRING_DEVIATION = 60f;
    
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
        
        // 3. Find candidate pairings: two roads share a green phase only when they are
        // roughly opposite each other (near 180° apart). A road whose nearest neighbour
        // to opposite is still too skewed (perpendicular, near-parallel, or a Y-junction
        // arm) has "no opposite" and gets its own solo phase. This replaces the old
        // circular-adjacency exclusion, which wrongly discarded the true opposite pair
        // whenever it fell next to another road in sorted-angle order (every pair at a
        // 3-way, and the real opposite pair at skewed 4-ways).
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

        // 4. Maximum-cardinality matching over the eligible pairs, breaking ties
        // toward the most-opposite pairing. Greedy "smallest deviation first"
        // could consume a road's only eligible partner and needlessly leave two
        // other roads soloed - producing extra phases and the "one green, three
        // red" appearance at a 4-way that actually has two valid opposite pairs.
        // A maximum matching pairs every road that has an available opposite.
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
    
    /// <summary>
    /// Maximum-cardinality matching over the eligible near-opposite pairs, with
    /// smallest total deviation as a tie-break. Returns partner[i] = matched road
    /// index for road i, or -1 if it stays solo. numRoads is small (intersection
    /// arms), so an exhaustive search is cheap and always finds the optimum,
    /// unlike the previous greedy pass.
    /// </summary>
    private int[] FindBestMatching(int numRoads, List<(int roadA, int roadB, float deviationFrom180)> pairs)
    {
        // Adjacency: for each road, its eligible partners and their deviations.
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

            // Already matched by an earlier road - move on.
            if (match[idx] != -1)
            {
                Recurse(idx + 1, count, dev);
                return;
            }

            // Option A: pair idx with each still-free partner of higher index.
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

            // Option B: leave idx solo.
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
