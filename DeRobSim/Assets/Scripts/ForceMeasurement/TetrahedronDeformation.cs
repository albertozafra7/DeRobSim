using System.Collections.Generic;
using UnityEngine;

public class TetrahedronDeformation : MonoBehaviour
{
    #region Properties
    // Mesh filter for the triangular mesh (assign the mesh in the inspector)
    public SkinnedMeshRenderer skin;

    // List to store tetrahedron vertices and their corresponding triangles
    public Vector3[] tetrahedronVertices;
    public Vector4[] tetrahedronTriangles;
    #endregion Properties

    #region Native Methods

    // Start is called before the first frame update
    void Start()
    {
        // Create tetrahedrons from the triangular mesh
        GenerateTetrahedronsFromMesh();

        // Deform the tetrahedrons based on the mesh deformation
        DeformTetrahedrons();
    }

    void Update(){
        // In each time-step we recompute the virtual vertex pose
        DeformTetrahedrons();
    }

    #endregion Native Methods

    #region Custom Methods

    // Generate tetrahedrons based on mesh triangles
    void GenerateTetrahedronsFromMesh()
    {
        Mesh mesh = new Mesh();
        skin.BakeMesh(mesh);
        Vector3[] vertices = mesh.vertices;
        int[] triangles = mesh.triangles;

        // We resize the arrays used to store the tetrahedra and its node positions
        System.Array.Resize(ref tetrahedronVertices, (int)(triangles.Length * 4/3));
        System.Array.Resize(ref tetrahedronTriangles, (int)(triangles.Length/3));


        // For each triangle, create a tetrahedron with a height vertex
        for (int i = 0; i < triangles.Length; i += 3)
        {
            int idx0 = triangles[i];
            int idx1 = triangles[i + 1];
            int idx2 = triangles[i + 2];
            int idx3 = triangles.Length + (int)(i/3);

            // Calculate the centroid (middle point) of the base triangle
            Vector3 v0 = vertices[idx0];
            Vector3 v1 = vertices[idx1];
            Vector3 v2 = vertices[idx2];
            Vector3 middleVertex = (v0 + v1 + v2) / 3;

            // Calculate the normal of the triangle to define the height direction
            Vector3 normal = Vector3.Cross(v1 - v0, v2 - v0).normalized;

            // Calculate the height offset (you can adjust this to scale the volume)
            float height = 0.1f; // This can be adjusted based on the specific needs
            Vector3 heightVertex = middleVertex + normal * height;

            // Add the three triangle vertices and the calculated height vertex
            tetrahedronVertices[idx0] = v0;
            tetrahedronVertices[idx1] = v1;
            tetrahedronVertices[idx2] = v2;
            tetrahedronVertices[idx3] = heightVertex;

            // Create a set of tetrahedron indices (4 vertices per tetrahedron)
            tetrahedronTriangles[(int)(i/3)] = new Vector4(idx0,  // Vertex 0
                                                           idx1,  // Vertex 1
                                                           idx2,  // Vertex 2
                                                           idx3); // Height vertex
        }
    }

    // Deform tetrahedrons based on mesh deformation
    void DeformTetrahedrons()
    {
        // Get the current mesh vertices (deformed state)
        Mesh mesh = new Mesh();
        skin.BakeMesh(mesh);
        Vector3[] deformedVertices = mesh.vertices;

        // For each tetrahedron, we would deform the vertices
        for (int i = 0; i < tetrahedronTriangles.Length; i++)
        {
            Vector4 tetrahedron = tetrahedronTriangles[i];
            Vector3[] tetrahedronVertexPositions = new Vector3[4];

            // Get the deformed positions for the tetrahedron's base triangle vertices
            for (int j = 0; j < 3; j++)
            {
                int vertexIndex = (int)tetrahedron[j];
                tetrahedronVertexPositions[j] = deformedVertices[vertexIndex];
            }

            // Calculate the new middle vertex position as the centroid of the deformed triangle
            Vector3 deformedMiddleVertex = (tetrahedronVertexPositions[0] + tetrahedronVertexPositions[1] + tetrahedronVertexPositions[2]) / 3;

            // Recalculate the height vertex, which is offset from the centroid in the direction of the triangle's normal
            Vector3 normal = Vector3.Cross(tetrahedronVertexPositions[1] - tetrahedronVertexPositions[0], tetrahedronVertexPositions[2] - tetrahedronVertexPositions[0]).normalized;
            float height = (tetrahedronVertexPositions[3] - deformedMiddleVertex).magnitude; // Maintain similar height
            Vector3 deformedHeightVertex = deformedMiddleVertex + normal * height;

            // Update the tetrahedron positions (in a real system, you'd also want to update your physics or collision system here)
            tetrahedronVertexPositions[3] = deformedHeightVertex;

            // Update the tetrahedron positions in the vertex list
            for (int j = 0; j < 4; j++)
            {
                tetrahedronVertices[(int)tetrahedron[j]] = tetrahedronVertexPositions[j];
            }
        }
    }

    #endregion Custom Methods

}
