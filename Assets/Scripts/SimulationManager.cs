using UnityEngine;
using System.Collections.Generic;

/// <summary>
/// Main manager for the traffic simulation. Coordinates road generation and car spawning.
/// </summary>
public class SimulationManager : MonoBehaviour
{
    [Header("Components")]
    [SerializeField] private RoadNetworkGenerator roadGenerator;
    [SerializeField] private RoadNetworkVisualizer roadVisualizer;
    [SerializeField] private CSVLogger csvLogger;
    
    [Header("Road Network Settings")]
    [SerializeField] private bool generateOnStart = true;
    
    [Header("Car Spawning Settings")]
    [Tooltip("Car prefab to spawn (must have CarController component)")]
    [SerializeField] private GameObject carPrefab;
    
    [Tooltip("Number of cars to spawn in simulation")]
    [SerializeField] private int numberOfCars = 10;
    
    [Tooltip("Automatically spawn cars after road generation")]
    [SerializeField] private bool spawnCarsOnStart = true;
    
    [Tooltip("Minimum distance between spawned cars (units)")]
    [SerializeField] private float minSpawnSeparation = 10f;

    private RoadMap roadMap;
    private List<CarController> activeCars = new List<CarController>();
    private int nextCarID = 0;
    
    [Header("Intersection Management")]
    private IntersectionManager intersectionManager;

    private void Start()
    {
        if (generateOnStart)
        {
            GenerateRoadNetwork();
            
            // Initialize intersection management system
            if (roadMap != null)
            {
                intersectionManager = new IntersectionManager();
                intersectionManager.Initialize(roadMap, roadGenerator.GetConfig());
                Debug.Log("IntersectionManager initialized");
            }
            
            if (spawnCarsOnStart && roadMap != null)
            {
                SpawnCars();
            }
        }
    }

    private void Update()
    {
        if (intersectionManager != null)
        {
            intersectionManager.Update(Time.deltaTime);
        }
    }

    [ContextMenu("Generate Road Network")]
    public void GenerateRoadNetwork()
    {
        if (roadGenerator == null)
        {
            Debug.LogError("RoadNetworkGenerator is not assigned!");
            return;
        }

        roadMap = roadGenerator.GenerateRoadNetwork();
        
        if (roadMap != null)
        {
            Debug.Log($"Road network generated successfully with seed {roadMap.RandomSeed}");
            
            if (roadVisualizer != null)
                roadVisualizer.UpdateVisualization();
        }
        else
        {
            Debug.LogError("Failed to generate road network!");
        }
    }
    
    [ContextMenu("Spawn Cars")]
    public void SpawnCars()
    {
        if (roadMap == null)
        {
            Debug.LogError("Cannot spawn cars: No road network generated!");
            return;
        }
        
        if (carPrefab == null)
        {
            Debug.LogError("Cannot spawn cars: Car prefab not assigned!");
            return;
        }
        
        // Clear existing cars
        ClearCars();
        
        // Get all available lanes
        List<Lane> allLanes = roadMap.GetAllLanes();
        
        if (allLanes.Count == 0)
        {
            Debug.LogError("Cannot spawn cars: No lanes in road network!");
            return;
        }
        
        // Get intersection detection distance from config
        float intersectionDetectionDistance = 15f; // Default
        if (roadGenerator != null && roadGenerator.GetConfig() != null)
        {
            intersectionDetectionDistance = roadGenerator.GetConfig().IntersectionDetectionDistance;
        }
        
        // Spawn cars with random placement
        int carsSpawned = 0;
        int maxAttempts = numberOfCars * 10; // Prevent infinite loop
        int attempts = 0;
        
        while (carsSpawned < numberOfCars && attempts < maxAttempts)
        {
            attempts++;
            
            // Pick random lane
            Lane randomLane = allLanes[Random.Range(0, allLanes.Count)];
            
            // Calculate safe spawn range (avoid spawning too close to intersection)
            float maxSpawnDistance = randomLane.Length - intersectionDetectionDistance - 5f;
            if (maxSpawnDistance < 5f)
            {
                // Lane too short, skip
                continue;
            }
            
            // Pick random position along lane (avoiding intersection zone)
            float randomDistance = Random.Range(0f, maxSpawnDistance);
            
            // Check if position is too close to other cars
            if (IsPositionTooClose(randomLane, randomDistance))
                continue;
            
            // Spawn car
            GameObject carObj = Instantiate(carPrefab, transform);
            CarController car = carObj.GetComponent<CarController>();
            
            if (car == null)
            {
                Debug.LogError("Car prefab must have CarController component!");
                Destroy(carObj);
                break;
            }
            
            // Initialize car
            car.Initialize(nextCarID++, randomLane, randomDistance, this);
            activeCars.Add(car);
            
            carsSpawned++;
        }
        
        Debug.Log($"Spawned {carsSpawned} cars in {attempts} attempts");
    }
    
    [ContextMenu("Clear Cars")]
    public void ClearCars()
    {
        foreach (var car in activeCars)
        {
            if (car != null)
                Destroy(car.gameObject);
        }
        
        activeCars.Clear();
        nextCarID = 0;
    }
    
    /// <summary>
    /// Check if a spawn position is too close to existing cars.
    /// </summary>
    private bool IsPositionTooClose(Lane lane, float distance)
    {
        foreach (var car in activeCars)
        {
            if (car == null)
                continue;
            
            // Check if car is on same lane
            if (car.CurrentLane == lane)
            {
                float existingCarDistance = Vector3.Distance(
                    car.GetPosition(),
                    lane.GetPositionAtDistance(distance)
                );
                
                if (existingCarDistance < minSpawnSeparation)
                    return true;
            }
        }
        
        return false;
    }
    
    /// <summary>
    /// Get all active cars (for collision detection).
    /// </summary>
    public List<CarController> GetAllCars()
    {
        return activeCars;
    }

    public RoadMap GetRoadMap()
    {
        return roadMap;
    }
    
    public RoadNetworkGenerator GetRoadGenerator()
    {
        return roadGenerator;
    }
    
    public IntersectionManager IntersectionManager => intersectionManager;
}
