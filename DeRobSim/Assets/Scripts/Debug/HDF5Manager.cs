using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using PureHDF; // Assuming PureHDF namespace is available after DLL import
using System.IO;

public class HDF5Manager : MonoBehaviour
{
    public string fileName = "simulation_data.h5";
    public string filePath = "SimulationLogs";
    public bool saveLog = true;
    private string absoluteFilePath;

    void Start()
    {
        absoluteFilePath = Path.Combine(Application.dataPath, filePath, fileName);
        SaveTransformData();
    }

    void SaveTransformData()
    {
        // Example data for position and rotation
        Vector3[] positions = new Vector3[] {
            new Vector3(0, 1, 2),
            new Vector3(3, 4, 5),
            new Vector3(6, 7, 8)
        };

        Quaternion[] rotations = new Quaternion[] {
            Quaternion.identity,
            Quaternion.Euler(0, 90, 0),
            Quaternion.Euler(0, 180, 0)
        };

        // We create the file instance
        var h5file = new H5File()
        {
            ["TransformData"]= new H5Group()
            {
                // Create a dataset for positions
                ["positions"] = positions,

                // Create a dataset for rotations
                ["rotations"] = rotations,

                // Optionally, save timestamps for each frame
                ["timestamps"] = new double[] { 0.1, 0.2, 0.3 }
            }
        };

        if(saveLog) {
            h5file.Write(absoluteFilePath);
            Debug.LogWarning(absoluteFilePath);
            Debug.Log("Data saved to HDF5 file.");
        }
    }
}
