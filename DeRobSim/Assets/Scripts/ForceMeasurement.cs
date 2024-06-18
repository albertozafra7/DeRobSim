using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Threading.Tasks;
using NVIDIA.Flex;

public class ForceMeasurement : MonoBehaviour
{
    public NVIDIA.Flex.FlexActor FlexComponent;

    private FlexContainer m_container;
    private FlexContainer.ParticleData particleData = new FlexContainer.ParticleData();
    private Vector4[] _particles;
    private Dictionary<int, List<int>> regions = new Dictionary<int, List<int>>();
    private Dictionary<int, Vector3> regionForces = new Dictionary<int, Vector3>();
    private Dictionary<int, Vector3> regionCenters = new Dictionary<int, Vector3>();
    private Vector4 MeshCenter;

    void Start()
    {
        // Initialize particle data
        m_container = FlexComponent.GetContainer();
        
        particleData.container = m_container;
        particleData.particleData = FlexExt.MapParticleData(m_container.handle);
        FlexExt.UnmapParticleData(m_container.handle);

        System.Array.Resize(ref _particles, FlexComponent.GetParticleNum());// FlexComponent.GetParticleNum());
        particleData.GetParticles(FlexComponent.GetParticleStartId(), FlexComponent.GetParticleNum(), _particles);

        MeshCenter = ComputeCenter(_particles);

        // Define regions
        // for (int i = 0; i < _particles.Length; i++)
        // {
        //     Vector4 position = particleData.GetParticle(i);
        //     int region = GetRegion(position);

        //     if (!regions.ContainsKey(region))
        //     {
        //         regions[region] = new List<int>();
        //     }
        //     regions[region].Add(i);
        // }

        // Define regions
        int numTasks = System.Environment.ProcessorCount;
        int particlesPerTask = _particles.Length / numTasks;
        List<Task<Dictionary<int, List<int>>>> tasks = new List<Task<Dictionary<int, List<int>>>>();

        for (int i = 0; i < numTasks; i++)
        {
            int start = i * particlesPerTask;
            int end = (i == numTasks - 1) ? _particles.Length : start + particlesPerTask;

            tasks.Add(Task.Run(() => ProcessParticles(start, end)));
        }

        Task.WhenAll(tasks).ContinueWith(t =>
        {
            foreach (var task in tasks)
            {
                foreach (var region in task.Result)
                {
                    if (!regions.ContainsKey(region.Key))
                    {
                        regions[region.Key] = new List<int>();
                    }
                    regions[region.Key].AddRange(region.Value);
                }
            }
        }).Wait(); // Wait for all tasks to complete before proceeding
    }

    Dictionary<int, List<int>> ProcessParticles(int start, int end)
    {
        Dictionary<int, List<int>> localRegions = new Dictionary<int, List<int>>();

        for (int i = start; i < end; i++)
        {
            Vector4 position = _particles[i];
            int region = GetRegion(position);

            if (!localRegions.ContainsKey(region))
            {
                localRegions[region] = new List<int>();
            }
            localRegions[region].Add(i + FlexComponent.GetParticleStartId());
        }

        return localRegions;
    }
    void Update()
    {
        particleData.GetParticles(FlexComponent.GetParticleStartId(), FlexComponent.GetParticleNum(), _particles);
        // ApplyImpulses(particleData); // Assuming ApplyImpulses is part of this class or accessible

        // Reset force calculations
        regionForces.Clear();
        regionCenters.Clear();

        // Calculate force and center for each region
        foreach (var region in regions)
        {
            Vector3 totalForce = Vector3.zero;
            Vector4 center = Vector4.zero;

            foreach (var index in region.Value)
            {
                Vector3 force = particleData.GetVelocity(index) * m_container.damping; // Simplified force calculation
                totalForce += force;
                center += _particles[index - FlexComponent.GetParticleStartId()];
            }

            int numParticles = region.Value.Count;
            if (numParticles > 0)
            {
                center /= numParticles;
            }

            regionForces[region.Key] = totalForce;
            regionCenters[region.Key] = center;

            // Print region information
            Debug.Log($"Region {region.Key} of size {numParticles} has a force of {totalForce} with center in {center}");
        }

        // ColorizeMesh();
    }

    int GetRegion(Vector4 position)
    {
        // Example region determination (quadrants)
        if (position.x >= MeshCenter.x && position.z >= MeshCenter.z)
        {
            return 1;
        }
        else if (position.x < MeshCenter.x && position.z >= MeshCenter.z)
        {
            return 2;
        }
        else if (position.x < MeshCenter.x && position.z < MeshCenter.z)
        {
            return 3;
        }
        else // position.x >= 0 && position.z < 0
        {
            return 4;
        }
    }

    Vector3 ComputeCenter(Vector4[] positions)
    {
        Vector4 sum = Vector4.zero;
        int count = positions.Length;

        for (int i = 0; i < count; i++)
        {
            sum += positions[i];
        }

        return sum / count;
    }

    void ColorizeMesh()
    {
        // Example: Colorize the mesh based on force magnitude (simplified)
        Mesh mesh = FlexComponent.gameObject.GetComponent<MeshFilter>().mesh;
        Vector3[] vertices = mesh.vertices;
        Color[] colors = new Color[vertices.Length];

        foreach (var region in regions)
        {
            Vector3 totalForce = regionForces[region.Key];
            Color regionColor = GetColorBasedOnForce(totalForce.magnitude);

            foreach (var index in region.Value)
            {
                colors[index - FlexComponent.GetParticleStartId()] = regionColor;
            }
        }

        mesh.colors = colors;
    }

    Color GetColorBasedOnForce(float forceMagnitude)
    {
        // Example: Map force magnitude to color (simplified)
        return Color.Lerp(Color.green, Color.red, forceMagnitude / 10.0f); // Assuming max force magnitude is 10 for this example
    }
}
