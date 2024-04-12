using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using NVIDIA.Flex;

public class setActiveGrab : MonoBehaviour
{
    public Grabber grabber;
    public FlexActor actor;
    public bool grabbed;
    // Start is called before the first frame update
    void Start()
    {
        grabbed = false;
        if (grabber != null)
            actor.addGrabber(grabber);

    }

    // Update is called once per frame
    void Update()
    {
        // if(grabber != null)
        //     actor.addGrabber(grabber);

        if(grabbed)
            grabber.grab();
        else
            grabber.release(); 

        //print(actor.getActiveGrab());
        // if(grabbed)
        //     FlexActor.activeGrab = grabbed;
        // else
        //     FlexActor.relGrasp = true;
        
    }
}
