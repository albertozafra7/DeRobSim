using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.IO;
using System.Linq;

public class TetrahedralMeshTracking : MonoBehaviour
{
    #region Properties

    //--------- Public ---------

    // +++ File Managing +++ 
    public string mshFilePath;  // Relative path from Assets folder

    // +++ Mesh Managing ++
    public SkinnedMeshRenderer skinnedMeshRenderer; // Skinned Mesh Renderer to get the triangular mesh

    // +++ Compute Shaders for GPU optimization +++
    [Header("ComputeShaders")]
    [SerializeField]
	ComputeShader MappingcomputeShader;
    [SerializeField]
	ComputeShader InterpolationcomputeShader;
    
    // +++ Mapping +++
    public int n_NNNodes = 3; // Number of Nearest Neighbors to do the mapping

    // +++ Debugging +++
    [Header("Drawing")]
    public bool DrawTetMesh = false;
    public Color vertexColor = Color.red; // Color of the vertices
    public Color edgeColor = Color.blue; // Color of the edges
    public float vertexSize = 0.05f; // Size of the spheres representing vertices

    //--------- Private ---------

    // +++ Mesh file Managing +++
    private string absoluteFilePath; // Absolute file path to the .msh file

    // BoundingBox of the tetrahedral mesh
    private Vector3 tetraMinBounds;
    private Vector3 tetraMaxBounds;

    // Offset to locate the vertices in world coordinates
    private Vector3 meshOriginOffset;
    // Scale to transform from the local mesh coordinates of the file to Unity's world coordinates
    private float meshScaleFactor;

    // +++ Tetrahedra Managers +++
    private List<Vector3> tetrahedralVertices = new List<Vector3>(); // Tetrahedral vertices' positions (in world coordinates)
    private List<int[]> tetrahedra = new List<int[]>(); // Tetrahedron structure storing (Each tetrahedron is an array of 4 indices)
    private Dictionary<int, List<int>> tetraToSurfaceMapping = new Dictionary<int, List<int>>(); // Mapping between the tetrahedral mesh and the triangular surface mesh in unity

    // +++ Triangular Surface Mesh Manager +++
    private List<Vector3> surfaceVertices = new List<Vector3>(); // Positions of the Surface mesh' vertices (in local coordinates)
    private Mesh surfaceMesh; // Copy of the SkinnedMeshRenderer to access to the mesh' vertices and triangles

    #endregion Properties

    #region Main Methods

    // Awake is called before the first frame of the simulation
    void Awake()
    {
        if (skinnedMeshRenderer == null)
        {
            Debug.LogError("SkinnedMeshRenderer not assigned!");
            return;
        }

        absoluteFilePath = Path.Combine(Application.dataPath, mshFilePath);

        if (File.Exists(absoluteFilePath))
        {
            // Step 1: Load the tetrahedral mesh information
            ReadMshFile(absoluteFilePath);

            // Step 2: Aling the mesh information with Unity's mesh bounds
            AlignTetrahedralMeshToUnity();

            // Step 3: Initialize surface vertices
            surfaceMesh = new Mesh();
            skinnedMeshRenderer.BakeMesh(surfaceMesh);
            surfaceVertices.AddRange(surfaceMesh.vertices);

            // Step 4: Compute initial mapping
            ComputeTetraToSurfaceMappingWithGPU();
        }
        else
        {
            Debug.LogError("The specified .msh file does not exist.");
        }
    }

    // Update is called once per frame
    void Update()
    {
        // Step 1: Update surface vertices from Skinned Mesh Renderer
        skinnedMeshRenderer.BakeMesh(surfaceMesh);
        surfaceVertices.Clear();
        surfaceVertices.AddRange(surfaceMesh.vertices);

        // Step 2: Interpolate tetrahedral vertex positions
        InterpolateTetrahedralVerticesWithGPU();
    }

    #endregion MainMethods

    #region Custom Methods

    // Used to read the bounding box, the vertices and the tetrahedra contained in mesh files
    private void ReadMshFile(string filePath)
    {
        using (StreamReader reader = new StreamReader(filePath))
        {
            string line; // It is going to handle each line of the file

            // Booleans to determine in which section the program is reading
            bool entitiesSection = false;
            bool verticesSection = false;
            bool tetrahedraSection = false;

            while ((line = reader.ReadLine()) != null)
            {
                if (line.StartsWith("$Entities"))
                {
                    entitiesSection = true;
                    continue;
                }
                if (line.StartsWith("$EndEntities"))
                {
                    entitiesSection = false;
                    continue;
                }
                if (line.StartsWith("$Nodes"))
                {
                    verticesSection = true;
                    continue;
                }
                if (line.StartsWith("$EndNodes"))
                {
                    verticesSection = false;
                    continue;
                }
                if (line.StartsWith("$Elements"))
                {
                    tetrahedraSection = true;
                    continue;
                }
                if (line.StartsWith("$EndElements"))
                {
                    tetrahedraSection = false;
                    continue;
                }

                // Read Bounding Box
                if (entitiesSection)
                {
                    // Example line format: 1 -11 0 0 9 20 100 0 0
                    string[] parts = line.Split(' ');
                    if (parts.Length >= 7)
                    {
                        // Parse min and max bounds
                        float minX = float.Parse(parts[1]);
                        float minY = float.Parse(parts[2]);
                        float minZ = float.Parse(parts[3]);
                        float maxX = float.Parse(parts[4]);
                        float maxY = float.Parse(parts[5]);
                        float maxZ = float.Parse(parts[6]);

                        tetraMinBounds = new Vector3(minX, minY, minZ);
                        tetraMaxBounds = new Vector3(maxX, maxY, maxZ);

                        // Conversion to global coordinates
                        transform.TransformPoint(tetraMinBounds);
                        transform.TransformPoint(tetraMaxBounds);

                        // Debug.Log($"Parsed Tetrahedral Mesh Bounds: Min={tetraMinBounds}, Max={tetraMaxBounds}");
                    }
                }

                // Read vertices
                if (verticesSection)
                {
                    string[] parts = line.Split();
                    if (parts.Length == 3)
                    {
                        float x = float.Parse(parts[0]);
                        float y = float.Parse(parts[1]);
                        float z = float.Parse(parts[2]);
                        tetrahedralVertices.Add(new Vector3(x, y, z)); // Add to the surface list initially
                    }
                }

                // Read tetrahedra
                if (tetrahedraSection)
                {
                    string[] parts = line.Split();
                    if (parts.Length > 5)// && parts[1] == "4") // Tetrahedral elements
                    {
                        tetrahedra.Add(new int[]
                        {
                            int.Parse(parts[1]) - 1,
                            int.Parse(parts[2]) - 1,
                            int.Parse(parts[3]) - 1,
                            int.Parse(parts[4]) - 1
                        });
                    }
                }
            }
        }
    }

    // This method computes the scale factor and the offset from the local coordinates found in the mesh file to Unity's coordinate system
    private void AlignTetrahedralMeshToUnity()
    {
        if (skinnedMeshRenderer == null)
        {
            Debug.LogError("No SkinnedMeshRenderer assigned.");
            return;
        }

        // Get Unity mesh bounding box
        Bounds unityBounds = skinnedMeshRenderer.bounds;
        Vector3 unityMinBounds = unityBounds.min;
        Vector3 unityMaxBounds = unityBounds.max;

        // Compute scale factor
        Vector3 tetraSize = tetraMaxBounds - tetraMinBounds;
        Vector3 unitySize = unityMaxBounds - unityMinBounds;

        meshScaleFactor = Mathf.Min(unitySize.x / tetraSize.x, unitySize.y / tetraSize.y, unitySize.z / tetraSize.z);

        // Compute offset to align centers
        Vector3 tetraCenter = (tetraMinBounds + tetraMaxBounds) / 2;
        Vector3 unityCenter = (unityMinBounds + unityMaxBounds) / 2;

        meshOriginOffset = unityCenter - tetraCenter * meshScaleFactor;

        // Debug.Log($"Scale Factor: {meshScaleFactor}, Origin Offset: {meshOriginOffset}");

        // Apply transformation
        TransformTetrahedralMesh();
    }

    // Transforms each vertex local mesh file vertex position to Unity's world coordinates
    private void TransformTetrahedralMesh()
    {

        for (int i = 0; i < tetrahedralVertices.Count; i++)
        {
            tetrahedralVertices[i] = (tetrahedralVertices[i] * meshScaleFactor) + meshOriginOffset;

            // If this component is in the same gameObject as the skinnedMeshRender, the previous operation already transforms the tetrahedral vertex coordinates to world space
            if(skinnedMeshRenderer.gameObject != gameObject)
                tetrahedralVertices[i] = transform.TransformPoint(tetrahedralVertices[i]);
        }

        // Debug.Log("Tetrahedral mesh aligned with Unity mesh.");
    }

    // Computes the NN for mapping the tetrahedral mesh to the triangular Unity's surface mesh
    private void ComputeTetraToSurfaceMapping()
    {
        for (int i = 0; i < tetrahedralVertices.Count; i++)
        {
            // Transform the tetrahedral vertex to world coordinates
            // Vector3 tetraVertexWorld = transform.TransformPoint(tetrahedralVertices[i]);

            // Create a list to hold distances and indices of the nearest neighbors
            List<(float distance, int index)> nearestNeighbors = new List<(float, int)>();

            for (int j = 0; j < surfaceVertices.Count; j++)
            {
                // Transform the surface vertex to world coordinates
                Vector3 surfaceVertexWorld = skinnedMeshRenderer.gameObject.transform.TransformPoint(surfaceVertices[j]);

                // Calculate the distance
                // float distance = Vector3.Distance(tetraVertexWorld, surfaceVertexWorld);
                float distance = Vector3.Distance(tetrahedralVertices[i], surfaceVertexWorld);

                // Add the distance and index to the list
                nearestNeighbors.Add((distance, j));
            }

            // Sort the list by distance and take the three closest neighbors
            nearestNeighbors.Sort((a, b) => a.distance.CompareTo(b.distance));
            List<int> closestIndices = nearestNeighbors.Take(n_NNNodes).Select(n => n.index).ToList();

            // Debugging output
            // Debug.Log($"TetraVertex {i} -> Nearest SurfaceVertices: {string.Join(", ", closestIndices)}");

            // Store the mapping
            tetraToSurfaceMapping[i] = closestIndices;
        }

        // Debug.Log("Tetra-to-surface mapping (nearest neighbors) initialized.");
    }

    // Simmilar to the previous one but computes everything in GPU through a compute shader
    private void ComputeTetraToSurfaceMappingWithGPU()
    {
        // ComputeShader computeShader = Resources.Load<ComputeShader>("TetrahedralMeshMapping");
        // if(computeShader == null)
        //     Debug.LogError("Compute Shader TetrahedralMeshMapping not found");

        int kernel = MappingcomputeShader.FindKernel("CSMain");

        // Create buffers
        ComputeBuffer tetraBuffer = new ComputeBuffer(tetrahedralVertices.Count, sizeof(float) * 3);
        ComputeBuffer surfaceBuffer = new ComputeBuffer(surfaceVertices.Count, sizeof(float) * 3);
        ComputeBuffer neighborBuffer = new ComputeBuffer(tetrahedralVertices.Count * n_NNNodes, sizeof(int));

        // Set data
        tetraBuffer.SetData(tetrahedralVertices);
        surfaceBuffer.SetData(surfaceVertices);
        neighborBuffer.SetData(new int[tetrahedralVertices.Count * n_NNNodes]);

        // Set compute shader parameters
        MappingcomputeShader.SetBuffer(kernel, "tetrahedralVertices", tetraBuffer);
        MappingcomputeShader.SetBuffer(kernel, "surfaceVertices", surfaceBuffer);
        MappingcomputeShader.SetBuffer(kernel, "nearestNeighbors", neighborBuffer);
        MappingcomputeShader.SetInt("n_NNNodes", n_NNNodes);

        // Set other parameters
        MappingcomputeShader.SetMatrix("MeshTransform", skinnedMeshRenderer.transform.localToWorldMatrix);

        // Dispatch
        MappingcomputeShader.Dispatch(kernel, Mathf.CeilToInt(tetrahedralVertices.Count / 256f), 1, 1);

        // Retrieve data
        int[] nearestNeighbors = new int[tetrahedralVertices.Count * n_NNNodes];
        neighborBuffer.GetData(nearestNeighbors);

        // Cleanup
        tetraBuffer.Dispose();
        surfaceBuffer.Dispose();
        neighborBuffer.Dispose();

        // Convert the flat array into the dictionary
        tetraToSurfaceMapping.Clear();
        for (int i = 0; i < tetrahedralVertices.Count; i++)
        {
            List<int> neighbors = new List<int>();
            for (int j = 0; j < n_NNNodes; j++)
            {
                neighbors.Add(nearestNeighbors[i * n_NNNodes + j]);
            }
            tetraToSurfaceMapping[i] = neighbors;

        }


        // Debug.Log("Tetra-to-surface mapping computed with GPU.");
    }

    // Interpolates the tetrahedral mesh vertices' positions based on the mapping with the triangular surface mesh
    private void InterpolateTetrahedralVertices()
    {
        for (int i = 0; i < tetrahedralVertices.Count; i++)
        {
            List<int> mappedSurfaceVertices = tetraToSurfaceMapping[i];
            if (mappedSurfaceVertices.Count == 0)
                continue;

            // Initialize the interpolated position in world space
            Vector3 interpolatedPositionWorld = Vector3.zero;

            // Accumulate positions from mapped surface vertices in world space
            foreach (int surfaceIndex in mappedSurfaceVertices)
            {
                interpolatedPositionWorld += skinnedMeshRenderer.gameObject.transform.TransformPoint(surfaceVertices[surfaceIndex]);
            }

            // Average the position
            interpolatedPositionWorld /= mappedSurfaceVertices.Count;

            // Transform the interpolated position back to local space
            Vector3 interpolatedPositionLocal = transform.InverseTransformPoint(interpolatedPositionWorld);

            // Update the tetrahedral vertex position in local space
            tetrahedralVertices[i] = interpolatedPositionLocal;
        }

        // Debug.Log("Tetrahedral vertices updated with interpolated positions.");
    }

    // Similar to the previous method but does it in GPU through a compute shader
    private void InterpolateTetrahedralVerticesWithGPU()
    {
        // ComputeShader computeShader = Resources.Load<ComputeShader>("InterpolateTetrahedralVertices");
        int kernel = InterpolationcomputeShader.FindKernel("InterpolateVertices");
        // Flatten tetraToSurfaceMapping into two arrays for GPU compatibility
        List<int> surfaceIndices = new List<int>();
        List<int> mappingOffsets = new List<int>();
        mappingOffsets.Add(0); // First offset is zero

        foreach (var pair  in tetraToSurfaceMapping)
        {
            List<int> mappedList = pair.Value; // Access the list of mapped surface vertices
            surfaceIndices.AddRange(mappedList);
            mappingOffsets.Add(surfaceIndices.Count);
        }

        // Create GPU buffers
        ComputeBuffer tetrahedralBuffer = new ComputeBuffer(tetrahedralVertices.Count, sizeof(float) * 3);
        ComputeBuffer surfaceBuffer = new ComputeBuffer(surfaceVertices.Count, sizeof(float) * 3);
        ComputeBuffer mappingBuffer = new ComputeBuffer(surfaceIndices.Count, sizeof(int));
        ComputeBuffer offsetBuffer = new ComputeBuffer(mappingOffsets.Count, sizeof(int));

        // Set data for the buffers
        tetrahedralBuffer.SetData(tetrahedralVertices.ToArray());
        surfaceBuffer.SetData(surfaceVertices.ToArray());
        mappingBuffer.SetData(surfaceIndices.ToArray());
        offsetBuffer.SetData(mappingOffsets.ToArray());

        // Bind buffers to the compute shader
        InterpolationcomputeShader.SetBuffer(0, "TetrahedralVertices", tetrahedralBuffer);
        InterpolationcomputeShader.SetBuffer(0, "SurfaceVertices", surfaceBuffer);
        InterpolationcomputeShader.SetBuffer(0, "MappingIndices", mappingBuffer);
        InterpolationcomputeShader.SetBuffer(0, "MappingOffsets", offsetBuffer);

        // Set other parameters
        InterpolationcomputeShader.SetMatrix("MeshTransform", skinnedMeshRenderer.transform.localToWorldMatrix);
        // InterpolationcomputeShader.SetMatrix("InverseTransform", transform.worldToLocalMatrix);

        // Dispatch the compute shader
        int threadGroups = Mathf.CeilToInt(tetrahedralVertices.Count / 64.0f);
        InterpolationcomputeShader.Dispatch(0, threadGroups, 1, 1);

        // Retrieve the updated tetrahedral vertices
        Vector3[] updatedTetrahedralVertices = new Vector3[tetrahedralVertices.Count];
        tetrahedralBuffer.GetData(updatedTetrahedralVertices);
        tetrahedralVertices = updatedTetrahedralVertices.ToList();

        // Release GPU buffers
        tetrahedralBuffer.Release();
        surfaceBuffer.Release();
        mappingBuffer.Release();
        offsetBuffer.Release();

        // Debug.Log("Tetrahedral vertices updated using compute shader.");
    }
    #endregion Custom Methods

    #region Public Methods

    // Returns the number of tetrahedra
    public int GetTetrahedraNum()
    {
        return tetrahedra.Count;
    }

    // Converts the tetrahedron list to a Vector4 array to easily store it in the HDF5 debug file
    public Vector4[] tetrahedrons2Vector4()
    {
        Vector4[] tetrahedralElements = new Vector4[tetrahedra.Count];

        for(int i = 0; i < tetrahedra.Count; i++)
        {
            tetrahedralElements[i] = new Vector4((float)tetrahedra[i][0]+1.0f, (float)tetrahedra[i][1]+1.0f, (float)tetrahedra[i][2]+1.0f, (float)tetrahedra[i][3]+1.0f);
        }

        return tetrahedralElements;
    }

    // Returns the number of vertices found in the tetrahedral mesh
    public int GetTetVertsNum()
    {
        return tetrahedralVertices.Count;
    }

    // Returns the tetrahedral mesh vertices' positions (in world space)
    public Vector3[] GetTetVertsPosition()
    {
        return tetrahedralVertices.ToArray();
    }

    #endregion Public Methods

    #region Debug Methods
    void OnDrawGizmos()
    {
        if(DrawTetMesh)
        {
            // Draw vertices
            Gizmos.color = vertexColor;
            foreach (var vertex in tetrahedralVertices)
            {
                Gizmos.DrawSphere(vertex, vertexSize);
                // Gizmos.DrawSphere(transform.TransformPoint(vertex), vertexSize);
            }

            // Draw edges
            Gizmos.color = edgeColor;
            foreach (var tetrahedron in tetrahedra)
            {
                int edges = 3;
                int startVertex = 0;
                int endVertex = 1;
                // AddEdge(v1, v2);
                // AddEdge(v1, v3);
                // AddEdge(v1, v4);
                // AddEdge(v2, v3);
                // AddEdge(v2, v4);
                // AddEdge(v3, v4);

                for(int i = 0; i < 3; ++i)
                {
                    for(int j = 0; j < edges; ++j)
                    {
                        Vector3 start = tetrahedralVertices[tetrahedron[startVertex]];
                        Vector3 end = tetrahedralVertices[tetrahedron[endVertex]];
                        // Vector3 start = transform.TransformPoint(tetrahedralVertices[tetrahedron[startVertex]]);
                        // Vector3 end = transform.TransformPoint(tetrahedralVertices[tetrahedron[endVertex]]);
                        Gizmos.DrawLine(start, end);
                        endVertex++;
                    }
                    startVertex++;
                    endVertex = startVertex+1;
                    edges--;
                }
            }
        }
    }

    #endregion Debug Methods
}
