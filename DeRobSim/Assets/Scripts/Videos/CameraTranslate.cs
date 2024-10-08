using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CameraTranslate : MonoBehaviour
{
    public Camera MainCamera;
    public List<Transform> TargetPositions = new List<Transform>();
      public List<float> times2Animate = new List<float>();
    public bool camera_move_enabled = false;
    private bool movementEnabled = false;

    private float startTime = 0;

    private Transform animation_target;
    private float animation_time;

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
            }
        }

        if (camera_move_enabled)
        {
           
            MainCamera.transform.position = Vector3.Lerp(MainCamera.transform.position, animation_target.position, (Time.realtimeSinceStartup - startTime)/animation_time);
            MainCamera.transform.rotation = Quaternion.Lerp(MainCamera.transform.rotation, animation_target.rotation, (Time.realtimeSinceStartup - startTime)/animation_time);
        }

        if(!camera_move_enabled && movementEnabled){
            movementEnabled = false;
        }

        if((Time.realtimeSinceStartup - startTime)/animation_time >= 1.0f){
            camera_move_enabled = false;
            movementEnabled = false;
        }

    }
}
