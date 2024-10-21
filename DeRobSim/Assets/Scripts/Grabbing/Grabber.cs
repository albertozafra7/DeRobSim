using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using NVIDIA.Flex;


public class Grabber : MonoBehaviour {
    
    #region Properties

    // ++++++++++++++ Public ++++++++++++++
    public float detectionRadius = 0.02f;   // Determines the detection ratius of the particles that are going to be attached to the GameObject
    public bool drawRadius = false; // Determines if the radius is going to be drawn for debugging

    // ++++++++++++++ Private ++++++++++++++
    private GameObject grabber_obj = null;    // The gameObject to which the component is attached to
    private bool activeGrab = false;    // Determines if the object is grabbing something
    private bool onDetecting = false;   // Determines if the object is going to grab something, so it is time to detect the closest particles to attach to the grabber object
    private bool relGrasp = false;      // Determines if the object has been released
    private bool onReleasing = true;    // Determines if the object is on the releasing processs
    private List<int> particlesUnderRadius = new List<int>();   // Determines the particles that have to be attached to the GameObject in order to perform the grasp
    public List<FlexActor> actorList = new List<FlexActor>();     // Determines the flexActors that are coliding with the object
    private SphereCollider grabberCollider;
    #endregion

    #region Public Methods

    // --------- Setters ---------
    // public void setActiveGrab(bool active) {
    //     if(active) {      
    //         if(!activeGrab)
    //             onDetecting = active;
    //         else
    //             activeGrab = active;
    //     } else{
    //         onDetecting = false;
    //         activeGrab = false;

    //         if(!relGrasp)
    //             onReleasing = true;
    //         else
    //             onReleasing = false;
    //     } 
    // }

    // public void setActiveGrab(bool active){
    //     if(!activeGrab)
    //         oneTimePick = active;
    //     else
    //         activeGrab = active;
            
    //     relGrasp = !activeGrab && !relGrasp;
    // }
    // public void setOnGrabbing(bool active){
    //     activeGrab = active;
    // }

    public void grab(){
        addGrabbers2Actors();
        if(!isGrabbing())
            setOnDetecting();
        else
            setOnGrabbing();
    }

    public void setOnGrabbing(){
        onReleasing = false;
        onDetecting = false;
        activeGrab = true;
        relGrasp = false;
    }

    public void setOnDetecting(){
        onReleasing = false;
        onDetecting = true;
        activeGrab = false;
        relGrasp = false;
    }

    public void setOnReleasing(){
        onDetecting = false;
        onReleasing = true;
        activeGrab = false;
        relGrasp = false;
    }

    public void setOnReleased(){
        onDetecting = false;
        onReleasing = false;
        activeGrab = false;
        relGrasp = true;
    }

    public void release(){
        // Commented because its update is so fast that gives no time for updating the FlexActor
        // if(isReleasing()){
        //     Debug.Log("###G --> Release");
        //     setOnReleased();
        //     removeGrabbersFromActors();
        // }
        if(!isReleased()){
            setOnReleasing();   // This lets the FlexActor handle the releasing
        }
    }

    public void setDetectionRadius(float radius){
        if(radius < 0)
            return;
        else
            detectionRadius = radius;
    }

    public void setDetectedParticles(List<int> particles){
        particlesUnderRadius.Clear();
        if(particles != null)
            particlesUnderRadius = particles;
    }

    public void reset(){
        activeGrab = false;
        onDetecting = false;
        relGrasp = true;
        onReleasing = false;
        //detectionRadius = 0.02f;
        particlesUnderRadius.Clear();

    }

    // --------- Getters ---------

    public Vector3 getGrabberpos(){
        return grabber_obj.transform.position;
    }

    public bool getActiveGrab(){
        return activeGrab;
    }

    public bool getonDetecting(){
        return onDetecting;
    }

    public bool getonReleasing(){
        return onReleasing;
    }

    public bool getReleased(){
        return relGrasp;
    }
    public bool isGrabbing(){
        return activeGrab && !onDetecting && !onReleasing && !relGrasp;
    }

    public bool isOnDetecting(){
        return onDetecting && !activeGrab && !onReleasing && !relGrasp;
    }

    public bool isReleasing(){
        return onReleasing && !activeGrab && !onDetecting && !relGrasp;
    }

    public bool isReleased(){
        return relGrasp && !activeGrab && !onDetecting && !onReleasing;
    }

    public GameObject getGrabber(){
        return grabber_obj;
    }

    public float getDetectionRadius(){
        return detectionRadius;
    }

    public List<int> getDetectedParticles(){
        return particlesUnderRadius;
    }

    public int getDetectedParticlesCount(){
        return particlesUnderRadius.Count;
    }

    // --------- Custom ---------
    public float[] GetClosestParticle(Vector4[] allParticles){
        float helpDistMin = 1000;
        float indicesMin = -1;
        float[] ret = new float[2];
        for (int i = 0; i < allParticles.Length; i++)
        {
            if (Vector3.Distance(grabber_obj.transform.position, allParticles[i]) < helpDistMin)
            {
                helpDistMin = Vector3.Distance(grabber_obj.transform.position, allParticles[i]);
                indicesMin = i;
            }

        }

        ret[0] = indicesMin;
        ret[1] = helpDistMin;

        return ret;
    }

    public float GetDistanceToClosestParticle(Vector4[] allParticles){
        return GetClosestParticle(allParticles)[1];
    }

    public void DetectParticles(Vector4[] allParticles){
        setDetectedParticles(FindParticlesInRadius(allParticles, detectionRadius));
    }

    #endregion

    #region Private Methods

    // Methods for detecting the particles
    private List<int> FindParticlesInRadius(Vector4[] allParticles, float radius){
        List<int> DetectedParticles = new List<int>();

        for (int i = 0; i < allParticles.Length; i++)
        {
            if (Vector3.Distance(grabber_obj.transform.position, allParticles[i]) <= radius)
            {
                DetectedParticles.Add(i);
            }
        }

        return DetectedParticles;

    }

    private void addGrabbers2Actors(){
        foreach(FlexActor actor in actorList){
            actor.addGrabber(this);
        }
    }

    public void removeGrabbersFromActors(){
        foreach (FlexActor actor in actorList){
            actor.removeGrabber(this);
        }
    }

    private void Awake(){
        // Set up of the grabber_obj
        grabber_obj = gameObject;

        // We create the collider that will detect the flex actors on the neighborhoud
        grabberCollider = gameObject.AddComponent<SphereCollider>();
        grabberCollider.center = Vector3.zero;
        grabberCollider.radius = detectionRadius;
        grabberCollider.isTrigger = true;

        // Intialize all its properties
        reset();

    }

    private void Update(){
        if(grabberCollider.radius != detectionRadius){
            grabberCollider.radius = detectionRadius;
        }
    }

    // +++++++++++ Drawing +++++++++++
    private void OnDrawGizmosSelected()
    {
        if(drawRadius){
            // Draw a yellow sphere at the transform's position with the detection radius
            Gizmos.color = Color.yellow;
            Gizmos.DrawSphere(transform.position, detectionRadius);
        }

    }

    // +++++++++++ Flex Actor Management +++++++++++

    private void OnTriggerEnter(Collider other)
    {   
        
        if(other.gameObject.name == "FlexDetectShapes"){

            if(!activeGrab){
                FlexActor[] DetectedActors = FindObjectsOfType<FlexActor>();
                FlexActor closestActor = null;
                foreach(FlexActor actor in DetectedActors){
                    if(actor != null && actorList.Find(a => a.gameObject.name == actor.gameObject.name) == null){
                        if(closestActor == null)
                            closestActor = actor;
                        else if(Vector3.Distance(closestActor.transform.position, other.transform.position) > Vector3.Distance(actor.transform.position, other.transform.position))
                            closestActor = actor;                 
                    }
                }

                if(closestActor != null && (Vector3.Distance(other.bounds.ClosestPoint(transform.position),transform.position) <= detectionRadius)){
                    actorList.Add(closestActor);
                    closestActor.addGrabber(this);
                }

            }  
        
        }
    }

    // This aims to add the FlexActors that have been touched with the object, meanwhile the object was grabbing something
    private void OnTriggerStay(Collider other)
    {   
        
        if(other.gameObject.name == "FlexDetectShapes"){

            if(!activeGrab){
                FlexActor[] DetectedActors = FindObjectsOfType<FlexActor>();
                FlexActor closestActor = null;
                foreach(FlexActor actor in DetectedActors){
                    if(actor != null){
                        if(closestActor == null)
                            closestActor = actor;
                        else if(Vector3.Distance(closestActor.transform.position, other.transform.position) > Vector3.Distance(actor.transform.position, other.transform.position))
                            closestActor = actor;                 
                    }
                }

                if(closestActor != null && actorList.Find(a => a.gameObject.name == closestActor.gameObject.name) == null){
                    if(Vector3.Distance(other.bounds.ClosestPoint(transform.position),transform.position) <= detectionRadius){
                        actorList.Add(closestActor);
                        closestActor.addGrabber(this);
                    }
                }

            }  
        
        }
    }

    private void OnTriggerExit(Collider other)
    {        
        if(other.gameObject.name == "FlexDetectShapes"){
            if(!activeGrab){
                FlexActor[] DetectedActors = FindObjectsOfType<FlexActor>();
                FlexActor closestActor = null;
                foreach(FlexActor actor in DetectedActors){
                    if(actor != null && actorList.Find(a => a.gameObject.name == actor.gameObject.name) != null){
                        if(closestActor == null)
                            closestActor = actor;
                        else if(Vector3.Distance(closestActor.transform.position, other.transform.position) > Vector3.Distance(actor.transform.position, other.transform.position))
                            closestActor = actor;                 
                    }
                }
                closestActor.removeGrabber(this);
                actorList.Remove(closestActor);
            }
        
        }
    }

 

    #endregion Private Methods
    
}