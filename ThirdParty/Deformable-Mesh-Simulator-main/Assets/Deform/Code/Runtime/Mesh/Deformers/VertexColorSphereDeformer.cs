using UnityEngine;
using Deform;
using Unity.Jobs;
using Unity.Collections;
using Unity.Mathematics;

[Deformer(Category = Category.Normal, Name = "Vertex Color Sphere", Type = typeof(VertexColorSphereDeformer))]
public class VertexColorSphereDeformer : Deformer
{
    public Color color = Color.red;

    public override DataFlags DataFlags => DataFlags.Colors;

    void OnDrawGizmosSelected() {
        Gizmos.matrix = transform.localToWorldMatrix;
        Gizmos.DrawWireSphere(Vector3.zero, 1f);
    }

    public override JobHandle Process(MeshData data, JobHandle dependency = default)
    {
        return new VertexColorSphereJob {
            color = new float4(color.r, color.g, color.b, color.a),
            transform = DeformerUtils.GetMeshToAxisSpace(transform, data.Target.GetTransform()),
            //transform = Matrix4x4.TRS(data.Target.GetTransform().position, data.Target.GetTransform().rotation, data.Target.GetTransform().lossyScale).inverse* Matrix4x4.TRS(transform.position, transform.rotation, transform.lossyScale),
            positions = data.DynamicNative.VertexBuffer,
            colors = data.DynamicNative.ColorBuffer
        }.Schedule(data.Length, DEFAULT_BATCH_COUNT, dependency);
    }

    public struct VertexColorSphereJob : IJobParallelFor
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
