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
    private Mesh mesh;
    private SkinnedMeshRenderer meshRenderer;
    private float maxForce = 0;
    private int iter_count = 0;

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

        mesh = FlexComponent.gameObject.GetComponent<MeshFilter>().mesh;
        meshRenderer = FlexComponent.gameObject.GetComponent<SkinnedMeshRenderer>();

        //ClusterizeMesh(FlexComponent.asset.shapeOffsets.Length);
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

        //Debug.LogWarning(FlexComponent.asset.shapeOffsets.Length);
        
        //Debug.LogWarning(FlexComponent.m_drawParticlesHelper.m_mesh.subMeshCount);
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
                if(m_container.damping == 0)
                    force = particleData.GetVelocity(index);
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

            if(totalForce.magnitude > maxForce)
                maxForce = totalForce.magnitude;
        }

        ColorizeMesh();

        if(iter_count > 250)
        {
            maxForce = 0;
            iter_count = 0;
        }
        iter_count++;
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
        Vector3[] vertices = mesh.vertices;
        Color[] colors = new Color[vertices.Length];

        meshRenderer.sharedMesh = (Mesh) Instantiate(meshRenderer.sharedMesh);
        mesh = meshRenderer.sharedMesh;

        if(maxForce > 0)
        {
            foreach (var region in regions)
            {
                Vector3 totalForce = regionForces[region.Key];
                Color regionColor = GetColorBasedOnForce(totalForce.magnitude);

                for (int i = 0; i < vertices.Length; ++i)
                {
                    if(GetRegion(new Vector4(vertices[i].x,vertices[i].y,vertices[i].z)) == region.Key)
                    {
                        if(totalForce.magnitude > 2)
                            colors[i] = regionColor;
                        else
                            colors[i] = Color.green;
                    }
                }
            }

            mesh.SetColors(colors);
            meshRenderer.material.SetColorArray("_Color", colors);
        }
    }

    Color GetColorBasedOnForce(float forceMagnitude)
    {
        return Color.Lerp(Color.green, Color.red, forceMagnitude / (maxForce/2)); // Assuming max force magnitude is big enough to get its half
    }

    void ClusterizeMesh(int n_clusters){
        // If there is a FlexComponent associated to the script and it has a handle
        if (FlexComponent && FlexComponent.handle)
        {
            // We get the instance of our FlexComponent to access to its particles
            FlexExt.Instance instance = FlexComponent.handle.instance;

            // We generate an array that will tell us the indices of each particle within the container
            int[] indices = new int[instance.numParticles];
            // We copy the previously mentioned indices
            if (instance.numParticles > 0) FlexUtils.FastCopy(instance.particleIndices, indices);

            if (FlexComponent is FlexSoftActor)
            {
                // We generate a Buffer that will contain 
                m_indexBuffers = new ComputeBuffer[n_clusters];
                m_particleMaterials = new Material[n_clusters];

                mesh.subMeshCount = n_clusters;
                for (int i = 0; i < n_clusters; ++i)
                {
                    int start = i == 0 ? 0 : FlexComponent.asset.shapeOffsets[i - 1];
                    int count = FlexComponent.asset.shapeOffsets[i] - start;
                    mesh.SetIndices(new int[count], MeshTopology.Points, i);
                    m_indexBuffers[i] = new ComputeBuffer(count, 4);
                    int[] shape = new int[count];
                    for (int j = 0; j < count; ++j) shape[j] = indices[FlexComponent.asset.shapeIndices[start + j]];
                    m_indexBuffers[i].SetData(shape);
                    m_particleMaterials[i] = new Material(Shader.Find(PARTICLES_SHADER));
                    m_particleMaterials[i].hideFlags = HideFlags.HideAndDontSave;
                }
            }

            mesh.bounds = FlexComponent.bounds;

            MeshFilter meshFilter = GetComponent<MeshFilter>();
            meshFilter.mesh = mesh;

            meshRenderer.materials = m_particleMaterials;
            meshRenderer.shadowCastingMode = UnityEngine.Rendering.ShadowCastingMode.On;

        }
    }

    Material[] m_particleMaterials;
    ComputeBuffer[] m_indexBuffers;
    const string PARTICLES_SHADER = "Flex/FlexDrawParticles";

}
