using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Threading.Tasks;
using NVIDIA.Flex;

public class ForceMeasurement : MonoBehaviour
{
    public NVIDIA.Flex.FlexActor FlexComponent; // Reference to the FlexActor that holds particle and force data

    private FlexContainer m_container;  // Stores the container of the FlexComponent, which manages particles
    private FlexContainer.ParticleData particleData = new FlexContainer.ParticleData(); // Used to retrieve and manipulate particle data of the scene/container
    private Vector4[] _particles;   // Array used to hold the particles' position
    private Dictionary<int, List<int>> regions = new Dictionary<int, List<int>>();  // Stores regions of particles indexed by integers (e.g., quadrants)
    private Dictionary<int, Vector3> regionForces = new Dictionary<int, Vector3>(); // Tracks the force acting on each region
    private Dictionary<int, Vector3> regionCenters = new Dictionary<int, Vector3>();// Stores the center of each region based on particle positions
    private Vector4 MeshCenter; // The center of the mesh, used to calculate regions
    private Mesh mesh;
    private SkinnedMeshRenderer meshRenderer;
    private float maxForce = 0; // Keeps track of the highest force magnitude across regions
    private int iter_count = 0; // Iteration counter to reset force calculations periodically

    void Start()
    {
        // Initialize particle data
        m_container = FlexComponent.GetContainer(); // We retrieve the particle container from the Flex component.
        
        // We get the particle data of the container by mapping and unmapping the container
        particleData.container = m_container;
        particleData.particleData = FlexExt.MapParticleData(m_container.handle);
        FlexExt.UnmapParticleData(m_container.handle);

        // We adjust the size of the particle array to the number of particles and store the particles of the container into the _particles array
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

            // We generate an array that will tell us the indices of each particle within the deformable object (Flex Component)
            int[] indices = new int[instance.numParticles];
            // We copy the previously mentioned indices
            if (instance.numParticles > 0) FlexUtils.FastCopy(instance.particleIndices, indices);

            if (FlexComponent is FlexSoftActor) // Can be omitted --> This is for only execute the code with deformable objects
            {
                // We generate a Buffer that will contain 
                m_indexBuffers = new ComputeBuffer[n_clusters]; // We create an array that will have the particles' indices for each region/cluster
                m_particleMaterials = new Material[n_clusters]; // We create an array that will have the materials' used for colorizing each region/cluster

                mesh.subMeshCount = n_clusters;                 // We generate as many submeshes of the original mesh as the number of clusters that we want to have

                // --------- Cluster Set Up ---------
                for (int i = 0; i < n_clusters; ++i)
                {
                    int start = i == 0 ? 0 : FlexComponent.asset.shapeOffsets[i - 1];   // We determine the starting index of the particleData based on the predefined clusterization generated by the asset
                    int count = FlexComponent.asset.shapeOffsets[i] - start;            // Number of particles within the cluster

                    // Should be used only for meshes with a high tessellation
                    mesh.SetIndices(new int[count], MeshTopology.Points, i);            // Sets the indices of the particles as individual vertices (points) of the mesh to modify the submesh (cluster) appearance
                    

                    m_indexBuffers[i] = new ComputeBuffer(count, 4);    // We create the cluster sub-array that will contain the list of indices of each particle within the indexBuffer

                    int[] shape = new int[count];                       // Auxiliar array to hold the particle indices of this cluster

                    for (int j = 0; j < count; ++j)
                        shape[j] = indices[FlexComponent.asset.shapeIndices[start + j]];    // We assign the corresponding index of the particle to the temporal array for later copying it to the buffer
                    
                    m_indexBuffers[i].SetData(shape);   // Loads the particle indices for the cluster within the buffer

                    m_particleMaterials[i] = new Material(Shader.Find(PARTICLES_SHADER));   // We create a new material using an specific shader to draw the particles on the mesh
                    m_particleMaterials[i].hideFlags = HideFlags.HideAndDontSave;           // We make the material temporal (which means that it will not be saved on disk)
                }
            }

            mesh.bounds = FlexComponent.bounds; // We ensure that the FlexComponent and the mesh have the same bounds


            // We update the mesh filter of the object adding the generated submeshes and materials, also we enable the shadowcasting of the renderer
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
