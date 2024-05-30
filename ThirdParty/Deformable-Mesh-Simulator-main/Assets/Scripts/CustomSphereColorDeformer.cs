using UnityEngine;
using Deform;
using Unity.Jobs;
using Unity.Collections;
using Unity.Mathematics;

[Deformer(Category = Category.Normal, Name = "Custom Color Deformer", Type = typeof(CustomSphereColorDeformer))]
public class CustomSphereColorDeformer : Deformer
{
    public Color color = Color.red;

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

    [SerializeField] private Transform center;

    public override DataFlags DataFlags => DataFlags.Colors;

    void OnDrawGizmosSelected() {
        Gizmos.matrix = center.localToWorldMatrix;
        Gizmos.DrawWireSphere(Vector3.zero, 1f);
    }

    public override JobHandle Process(MeshData data, JobHandle dependency = default)
    {
        return new CustomSphereColorJob {
            color = new float4(color.r, color.g, color.b, color.a),
            transform = DeformerUtils.GetMeshToAxisSpace(center, data.Target.GetTransform()),
            positions = data.DynamicNative.VertexBuffer,
            colors = data.DynamicNative.ColorBuffer
        }.Schedule(data.Length, DEFAULT_BATCH_COUNT, dependency);
    }

    public struct CustomSphereColorJob : IJobParallelFor
    {
        public float4 color;
        public float4x4 transform;
        [ReadOnly]
        public NativeArray<float3> positions;
        public NativeArray<float4> colors;
        public void Execute(int index)
        {
            var point = math.mul(transform, new float4(positions[index], 1f)).xyz;
            // Debug.Log("Length?" + math.length(point));

            if (math.length(point) < 1) 
            {
                colors[index] = color;
            }
        }
    }
}
