using UnityEngine;
using Unity.Jobs;
using Unity.Burst;
using Unity.Collections;
using System.Collections.Generic;
using Unity.Mathematics;
using static Unity.Mathematics.math;
public class VertexSaver : MonoBehaviour
{
    public NativeArray<bool> grabbedVert;
    // Start is called before the first frame update
    public void CreateArray(int n_elements){
        grabbedVert = new NativeArray<bool>(n_elements,  Allocator.Persistent);
    
    }
    

    void Start()
    {
    }

    // Update is called once per frame
    void Update()
    {
        
    }
    private void OnDisable ()
    {
        if (grabbedVert.IsCreated)
            grabbedVert.Dispose();
    }
}
