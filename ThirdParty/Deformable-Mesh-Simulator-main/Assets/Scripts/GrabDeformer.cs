using UnityEngine;
using Unity.Jobs;
using Unity.Burst;
using Unity.Collections;
using Unity.Mathematics;
using static Unity.Mathematics.math;

namespace Deform
{
    [Deformer (Name = "Grab", Description = "Deforms an object depending on the grab", Type = typeof (MagnetDeformer))]
    public class GrabDeformer : Deformer
    {
        
        #region Custom Properties

        public float DetectRadius{
            get => detectRadius;
            set => detectRadius = value;
        }

        public float Strength{
            get => strength;
            set => strength = value;
        }

        public bool ActiveGrab{
            get => activeGrab;
            set => activeGrab = value;
        }


        public Transform Center
        {
            get
            {
                if (center == null)
                    center = transform;
                return center;
            }
            set => center = value;
        }

        [SerializeField, HideInInspector] private float detectRadius = 0.2f;
        [SerializeField, HideInInspector] private float strength = 1;
        [SerializeField, HideInInspector] private bool activeGrab = false;
        [SerializeField, HideInInspector] private Transform center;

        #endregion Custom Properties

        #region Deformer Pipeline

        public override DataFlags DataFlags => DataFlags.Vertices;


        public override JobHandle Process (MeshData data, JobHandle dependency = default (JobHandle))
		{
			var meshToAxis = DeformerUtils.GetMeshToAxisSpace (Center, data.Target.GetTransform ());

			return new GrabJob
			{
			}.Schedule (data.Length, DEFAULT_BATCH_COUNT, dependency);
		}


        //[BurstCompile]
		public struct GrabJob : IJobParallelFor
		{


            public float detectRadius;  // Detection radius to determine if the grabber object is close enough to grab the vertices of the mesh
            public float strength;      // Strength of the grab
            public bool activeGrab;     // Determines if the deformed object is being grabbed or not
            public float4x4 meshToAxis; // Homogeneous transformation matrix used to change from global to local space
            public float4x4 axisToMesh; // Inverse homogeneous transformation matrix used to change from local to global space
            public NativeArray<float3> vertices;    // Vertices of the mesh to be deformed


            public void Execute(int index)
            {
                // We change the vertice from World space to local space
                float3 vertex_local_position = mul(meshToAxis, float4(vertices[index], 1f)).xyz;

                float dist_to_grabber = pow(length(vertex_local_position),2f);

                if(dist_to_grabber < detectRadius)
                {
                    Debug.Log("Vertex " + index + " is close enough to be grabbed (dist =" + dist_to_grabber + ")");
                }
		    }

        }
        #endregion Deformer Pipeline
    }
}