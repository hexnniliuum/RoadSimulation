using UnityEngine;
using System.Collections.Generic;

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

            // Rebuilt per generation so stale state can't reference lanes that no longer exist
            intersectionManager = new IntersectionManager();
            intersectionManager.Initialize(roadMap, roadGenerator.GetConfig());
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
        
        ClearCars();
        
        List<Lane> allLanes = roadMap.GetAllLanes();
        
        if (allLanes.Count == 0)
        {
            Debug.LogError("Cannot spawn cars: No lanes in road network!");
            return;
        }
        
        float intersectionDetectionDistance = 15f;
        if (roadGenerator != null && roadGenerator.GetConfig() != null)
        {
            intersectionDetectionDistance = roadGenerator.GetConfig().IntersectionDetectionDistance;
        }
        
        int carsSpawned = 0;
        int maxAttempts = numberOfCars * 10; // cap attempts in case lanes can't fit all cars
        int attempts = 0;
        
        while (carsSpawned < numberOfCars && attempts < maxAttempts)
        {
            attempts++;
            
            Lane randomLane = allLanes[Random.Range(0, allLanes.Count)];
            
            // Keep the spawn point clear of the intersection zone at the lane's end
            float maxSpawnDistance = randomLane.Length - intersectionDetectionDistance - 5f;
            if (maxSpawnDistance < 5f)
            {
                continue;
            }
            
            float randomDistance = Random.Range(0f, maxSpawnDistance);
            
            if (IsPositionTooClose(randomLane, randomDistance))
                continue;
            
            GameObject carObj = Instantiate(carPrefab, transform);
            CarController car = carObj.GetComponent<CarController>();
            
            if (car == null)
            {
                Debug.LogError("Car prefab must have CarController component!");
                Destroy(carObj);
                break;
            }
            
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

        // Stale car references in intersection queues would permanently block their lanes
        intersectionManager?.ResetCarState();
    }
    
    private bool IsPositionTooClose(Lane lane, float distance)
    {
        foreach (var car in activeCars)
        {
            if (car == null)
                continue;
            
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
