using UnityEngine;
using System.Collections.Generic;
using System.IO;

/// <summary>
/// Logs vehicle positions to CSV file for analysis and heatmap generation.
/// CSV Format: vehicle_id,positionX,positionY,positionZ,time
/// </summary>
public class CSVLogger : MonoBehaviour
{
    [Header("Logging Settings")]
    [Tooltip("Enable/disable logging")]
    [SerializeField] private bool enableLogging = true;
    
    [Tooltip("Time interval between logs (seconds)")]
    [SerializeField] private float logInterval = 0.1f;
    
    [Tooltip("Output filename (relative to project root)")]
    [SerializeField] private string outputFilename = "PythonAnalysis/vehicle_positions.csv";
    
    [Tooltip("Reference to SimulationManager")]
    [SerializeField] private SimulationManager simulationManager;
    
    private StreamWriter writer;
    private float timeSinceLastLog = 0f;
    private string filePath;
    private bool isLogging = false;
    
    void Start()
    {
        if (enableLogging && simulationManager != null)
        {
            StartLogging();
        }
    }
    
    void Update()
    {
        if (!isLogging || !enableLogging)
            return;
        
        timeSinceLastLog += Time.deltaTime;
        
        if (timeSinceLastLog >= logInterval)
        {
            LogVehiclePositions();
            // Subtract the interval instead of resetting to zero so the
            // average cadence stays exactly logInterval despite frame quantization
            timeSinceLastLog -= logInterval;
        }
    }
    
    void OnDestroy()
    {
        StopLogging();
    }
    
    void OnApplicationQuit()
    {
        StopLogging();
    }
    
    [ContextMenu("Start Logging")]
    public void StartLogging()
    {
        if (isLogging)
        {
            Debug.LogWarning("Logging already active!");
            return;
        }
        
        if (simulationManager == null)
        {
            Debug.LogError("SimulationManager not assigned to CSVLogger!");
            return;
        }
        
        // Create file path (in project root, outside Assets folder)
        string projectRoot = Path.Combine(Application.dataPath, "..");
        filePath = Path.Combine(projectRoot, outputFilename);
        
        // Ensure the directory exists
        string directory = Path.GetDirectoryName(filePath);
        if (!Directory.Exists(directory))
        {
            Directory.CreateDirectory(directory);
        }
        
        try
        {
            // Create or overwrite file with header
            writer = new StreamWriter(filePath, false);
            writer.WriteLine("vehicle_id,positionX,positionY,positionZ,time");
            writer.Flush();
            
            isLogging = true;
            timeSinceLastLog = 0f;
            
            Debug.Log($"CSV logging started: {filePath}");
            Debug.Log($"Logging interval: {logInterval:F3} seconds");
        }
        catch (System.Exception e)
        {
            Debug.LogError($"Failed to start CSV logging: {e.Message}");
        }
    }
    
    [ContextMenu("Stop Logging")]
    public void StopLogging()
    {
        if (!isLogging)
            return;
        
        if (writer != null)
        {
            writer.Flush();
            writer.Close();
            writer = null;
        }
        
        isLogging = false;
        
        Debug.Log($"CSV logging stopped. File saved: {filePath}");
    }
    
    /// <summary>
    /// Log current positions of all vehicles.
    /// </summary>
    private void LogVehiclePositions()
    {
        if (writer == null)
            return;
        
        List<CarController> cars = simulationManager.GetAllCars();
        float currentTime = Time.time;
        
        foreach (var car in cars)
        {
            if (car == null)
                continue;
            
            Vector3 pos = car.GetPosition();
            
            // Format: vehicle_id,positionX,positionY,positionZ,time
            writer.WriteLine($"{car.CarID},{pos.x:F3},{pos.y:F3},{pos.z:F3},{currentTime:F3}");
        }
        
        // Flush every N logs to prevent data loss
        writer.Flush();
    }
    
    /// <summary>
    /// Get current log file path (for display/debugging).
    /// </summary>
    public string GetLogFilePath()
    {
        return filePath;
    }
    
    /// <summary>
    /// Check if currently logging.
    /// </summary>
    public bool IsLogging()
    {
        return isLogging;
    }
}

