using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using NVIDIA.Flex;

public class setActiveGrab : MonoBehaviour
{
    public GameObject grabber;
    public FlexActor actor;
    public bool grabbed;
    // Start is called before the first frame update
    void Start()
    {
        grabbed = false;
        if (grabber != null)
            actor.setGrabber(grabber);

    }

    // Update is called once per frame
    void Update()
    {
        if(grabber != null)
            actor.setGrabber(grabber);

        actor.setActiveGrab(grabbed); 
        // if(grabbed)
        //     FlexActor.activeGrab = grabbed;
        // else
        //     FlexActor.relGrasp = true;
        
    }
}
