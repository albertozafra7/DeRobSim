using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class AgentTranslator : MonoBehaviour
{
    public List<setActiveGrab> Agents = new List<setActiveGrab>();
    public List<Transform> TargetPositions = new List<Transform>();
    public List<float> times2Animate = new List<float>();
    public List<bool> grabbedAnimate = new List<bool>();
    public bool camera_move_enabled = false;
    private bool movementEnabled = false;

    private float startTime = 0;

    private setActiveGrab MainAgent;
    private Transform animation_target;
    private float animation_time;
    private bool grabbed = false;

    void Update()
    {
        if(!movementEnabled && camera_move_enabled){
            startTime = Time.realtimeSinceStartup;
            movementEnabled = true;

            if(TargetPositions.Count > 0){
                animation_target = TargetPositions[0];
                TargetPositions.Remove(animation_target);
                animation_time = times2Animate[0];
                times2Animate.Remove(animation_time);
                grabbed = grabbedAnimate[0];
                grabbedAnimate.Remove(grabbed);
                MainAgent = Agents[0];
                Agents.Remove(MainAgent);
            }
        }

        if (camera_move_enabled)
        {
           
            MainAgent.transform.position = Vector3.Lerp(MainAgent.transform.position, animation_target.position, (Time.realtimeSinceStartup - startTime)/animation_time);
            MainAgent.transform.rotation = Quaternion.Lerp(MainAgent.transform.rotation, animation_target.rotation, (Time.realtimeSinceStartup - startTime)/animation_time);
            
            MainAgent.grabbed = grabbed;
        }

        if(!camera_move_enabled && movementEnabled){
            movementEnabled = false;
        }

        if(movementEnabled && Vector3.Distance(MainAgent.transform.position,animation_target.position) < 0.01){
            movementEnabled = false;
        }

    }
}
