using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using NVIDIA;

public class Force : MonoBehaviour
{
    public float mul = 100f;
    public NVIDIA.Flex.FlexActor FlexComponet;
    public Vector3 dir = Vector3.up;
    public bool reset_pose = false;

    private Transform initial_pose;


    void Start()
    {
        initial_pose = transform;
        // FlexComponet = GetComponent<NVIDIA.Flex.FlexSoftActor>();
    }


    void Update()
    {  

        FlexComponet.ApplyImpulse(dir*mul);

        if(reset_pose){
            ResetTransform();
            reset_pose = false;
        }
    }

    void ResetTransform(){
        FlexComponet.Teleport(initial_pose.position, initial_pose.rotation);
        transform.position = initial_pose.position;
        transform.rotation = initial_pose.rotation;
    }
}