using System.Collections;
using System.Collections.Generic;
using UnityEngine;

// Requires to have a component of type Grabber
[RequireComponent(typeof(Grabber))]
public class Agent : MonoBehaviour
{
    #region Custom Struct

       public struct pose
    {
        public pose (Vector3 P, Quaternion R)
        {
            position = P;
            rotation = R;
        }

        public Vector3 position { get; set;}
        public Quaternion rotation { get; set;}

        public override string ToString() => $"({position}, {rotation})";
    }

    #endregion Custom Struct


    #region Properties

    //--------- Public --------- 
    public float maxVel = 10.0f;       // m/s
    public float maxAccel = 10.0f;     // m/s^2
    public bool resetPose = false;     // Resets the agent from the inspector
    public bool stopAgent = false;     // Stops the agent movement from the inspector
    public bool activeGrab = false;    // Determines if the agent is grabbing the object

    //--------- Private ---------
    private pose restingPose;          // Starting pose
    public Vector3 currentVel;         // holonomic robot (3 degrees of freedom) m/s
    public Vector3 currentAngVel;      // Angular velocity degrees/s
    public Vector3 currentAccel;       // holonomic robot m/s^2
    public Vector3 currentAngAccel;    // Angular acceleration degrees/s^2
    private Grabber grabber;           // Grabber object
    public setActiveGrab grabber_act;  // Responsible of grabbing --> TODO: Correct
    
        
    #endregion Properties

    #region Main Methods
    void Start()
    {
        resetPose = false;
        set_restPose(new pose(transform.position,transform.rotation));
        grabber = gameObject.GetComponent<Grabber>();
        grabber_act = gameObject.GetComponent<setActiveGrab>(); // TODO: Correct

        // If the robot does not have any grabber component
        if (grabber == null){
            Debug.LogError("AGENT ERROR: NO GRABBER FOUND");
            stopAgent = true;
        }
    }

    void Update()
    {
        if(resetPose)
            ResetAgent();
        
        if(!stopAgent){
            if(activeGrab)
                GrabObject();
            else
                ReleaseObject();
                
            move();
            accelerate();
        }
    }

    #endregion Main Methods

    #region Custom Methods

    public void resetAgent(){
        // We reset its pose and its movement
        set_pose(restingPose);
        StopAgent();
    }

    public void teleport(pose pose){
        set_pose(pose);
    }

    public void teleport_N_stop(pose pose){
        teleport(pose);
        StopAgent();
    }

    public void move(){
        // Uniformly accelerated motion
        transform.position += currentVel*Time.deltaTime;
        transform.Rotate(currentAngVel*Time.deltaTime, Space.World);
    }

    public void accelerate(){
        // Increment of the velocity at each timestep
        currentVel += currentAccel*Time.deltaTime;
        currentAngVel += currentAngAccel*Time.deltaTime;

        // We restrict the current velocities
        currentVel = restrictValues(currentVel, maxVel);
        currentAngVel = restrictValues(currentAngVel, maxVel);
    }

    public void StopAgent(){
        set_vel(Vector3.zero);
        set_angVel(Vector3.zero);
        set_accel(Vector3.zero);
        set_angAccel(Vector3.zero);
        stopAgent = true;
    }

    public void ResetAgent(){
        teleport_N_stop(restingPose);
        resetPose = false;
    }

    public void GrabObject(){
        // grabber.grab();
        grabber_act.grabbed = true;         // TODO: CORRECT
        activeGrab = true;
    }

    public void ReleaseObject(){
        if(activeGrab){
            // grabber.release();
            grabber_act.grabbed = false;    // TODO: CORRECT
            activeGrab = false;
        }
    }

    private Vector3 restrictValues(Vector3 value, float maxValue){
        Vector3 new_value = new Vector3(value.x, value.y, value.z);
        
        if(value.x > maxValue)
            new_value.x = maxValue;
        if(value.y > maxValue)
            new_value.y = maxValue;
        if(value.z > maxValue)
            new_value.z = maxValue;
        
        return new_value;
    }


    #endregion Custom Methods


    #region Getters

    //--------- Public --------- 

    //++++++ Getters ++++++

    //*** Poses ***//
    public Vector3 get_position(){
        return transform.position;
    }

    public Quaternion get_rotation(){
        return transform.rotation;
    }

    public Vector3 get_restingPosition(){
        return restingPose.position;
    }

    public Quaternion get_restingRotation(){
        return restingPose.rotation;
    }

    public Transform get_pose(){
        return transform;
    }

    public pose get_restingPose(){
        return restingPose;
    }

    //*** Velocities/Accelerations ***//
    public Vector3 get_vel(){
        return currentVel;
    }

    public Vector3 get_angVel(){
        return currentAngVel;
    }

    public Vector3 get_accel(){
        return currentAccel;
    }

    public Vector3 get_angAccel(){
        return currentAngAccel;
    }

    public float get_vel_magnitude(){
        return currentVel.magnitude;
    }

    public float get_angVel_magnitude(){
        return currentAngVel.magnitude;
    }

    public float get_accel_magnitude(){
        return currentAccel.magnitude;
    }

    public float get_angAccel_magnitude(){
        return currentAngAccel.magnitude;
    }

    public float get_maxVel(){
        return maxVel;
    }

    public float get_maxAccel(){
        return maxAccel;
    }

    //*** Grabber ***//
    public Grabber get_grabber(){
        return grabber;
    }

    public bool get_activeGrab(){
        return activeGrab;
    }


    #endregion Getters


    #region Setters
    //--------- Public --------- 

    //++++++ Setters ++++++

    //*** Poses ***//
    public void set_position(Vector3 position){
        transform.position = position;
    }

    public void set_rotation(Quaternion rotation){
        transform.rotation = rotation;
    }

    public void set_pose(pose pose){
        transform.position = pose.position;
        transform.rotation = pose.rotation;
    }

    public void set_restPosition(Vector3 position){
        restingPose.position = position;
    }

    public void set_restOrientation(Quaternion rotation){
        restingPose.rotation = rotation;
    }

    public void set_restPose(pose pose){
        restingPose.position = pose.position;
        restingPose.rotation = pose.rotation;
    }

    //*** Velocities/Accelerations ***//
    public void set_vel(Vector3 vel){
        // We restrict the velocity values to the maximum velocity
        // vel = restrictValues(vel, maxVel);

        currentVel = vel;
    }

    public void set_angVel(Vector3 angVel){
        // We restrict the velocity values to the maximum velocity
        // angVel = restrictValues(angVel, maxVel);

        currentAngVel = angVel;
    }

    public void set_accel(Vector3 accel){
        // We restrict the velocity values to the maximum acceleration
        // accel = restrictValues(accel, maxAccel);

        currentAccel = accel;
    }

    public void set_angAccel(Vector3 angAccel){
        // We restrict the velocity values to the maximum velocity
        // angAccel = restrictValues(angAccel, maxAccel);

        currentAngAccel = angAccel;
    }

    public void set_maxVel(float vel){
        maxVel = vel;
    }

    public void set_maxAccel(float accel){
        maxAccel = accel;
    }

    //*** Grabber ***//
    public void set_grabber(Grabber new_grabber){
        if(new_grabber != null)
            grabber = new_grabber;
    }

    public void set_activeGrab(bool active){
        activeGrab = active;
    }

    #endregion Setters

}
