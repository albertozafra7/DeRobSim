using System.Collections;
using System.Collections.Generic;
using System; 
using System.Text; 
using UnityEngine;
using Unity.Mathematics;
// Matlab Libraries
using MathWorks.MATLAB.NET.Arrays; // import from MWArray.dll
using MatlabControlLib;            // import the custom control Matlab Library


public class TransportControl : MonoBehaviour
{
    #region Properties
    //--------- Public ---------
    [Header("Agents")]
    public List<Agent> listAgents = new List<Agent>();              // Agents for being controlled
    [Header("Agent Poses")]
    public List<Transform> agentPose = new List<Transform>();       // Current pose (position and rotation) of the agents
    public List<Transform> agentDest = new List<Transform>();       // Destination pose of the agents

    //+++ Control params +++
    // Control accuracy
    [Header("Destination Error Threshold")]
    public float dest_threshold = 0.25f;

    // Delta Time
    [Header("Delta Time")]
    public float dt =  0.033f;

    // Control gains
    [Header("Control Gains")]
    public float[] kh = new float[2] {6.0f,3.0f};                   // Deformation Gain
    public float[] kg = new float[2] {3.0f,2.0f};                   // Deformation Correction Gain 
    public float[] ks = new float[2] {5.0f,6.0f};                   // Scale control Gain
    public float[] kgm = new float[2] {2.0f,6.0f};                  // Position Gain
    public float[] kth = new float[2] {4.0f,4.0f};                  // Orientation Gain

    // Control weights
    [Header("Control Weights")]
    public float alpha_H = 4.0f;                                    // Deformation Control Weight
    public float alpha_G = 2.0f;                                    // Correction Weight

    // Control Initialization
    [Header("Control Initialization")]
    public bool start_control = false;
    public bool draw_destiny = true;                                // Boolean used for drawing the destination of the object

    //--------- Private ---------
    private int n_agents;
    private float2x2 Rot90M = new float2x2(0.0f, -1.0f, 1.0f, 0.0f); // 90 deg rotation matrix to be used in the control
    private List<Agent.pose> agentPrevPose = new List<Agent.pose>(); // Previous agent pose
    private MatlabControlLib.MatlabControlLib controllerLib = new MatlabControlLib.MatlabControlLib();  // Object that uses the Matlab Library for the control algorithm
    private Vector3[] agentAccel;
    private bool agentsGrabbed = false;
    private bool agentsActivated = true;
    private float currPose_error = float.PositiveInfinity; // Determine the system current error

    #endregion Properties

    #region Main Methods

    void Awake()
    {
        // Time.deltaTime --> returns the time (in seconds) it took to complete the last frame
        //dt = Time.deltaTime;
    }

    // Start is called before the first frame update
    void Start()
    {
        n_agents = listAgents.Count;

        for(int i = 0; i < n_agents; ++i){
            // Store the initial poses
            agentPrevPose.Add(new Agent.pose(agentPose[i].position,agentPose[i].rotation));
            // Modify the agent dt to coordinate the whole control
            listAgents[i].set_dt(dt);
        }

        if(draw_destiny)
            DrawDestination();

        agentAccel = new Vector3[n_agents];

        ActivateAgents();

    }

    // Update is called once per frame
    void Update()
    {
        // ------- Drawing part ------- 
        // We draw the destiny if it has not been drawn
        if(draw_destiny)
            DrawDestination();

        // ------- Destination Evaluation -------
        // If the agents are not close enough we continue with the control, otherwise, we stop the control
        if(start_control && isCloseEnough(dest_threshold))
            start_control = false;

        // ------- Agent Grab -------
        if(start_control && !agentsGrabbed)
            AllAgentsGrab();

        // ------- Control part -------
        if(start_control){
            if(!agentsActivated)
                ActivateAgents();
            // We call the main control algorithm
            // Debug.LogWarning("AGENT POSE:" + Transform2Positions(agentPose));
            // Debug.LogWarning("AGENT DEST:" + Transform2Positions(agentDest));
            // Debug.LogWarning("AGENT PREV POSE:" + AgentPose2Positions(agentPrevPose));
            MWNumericArray accelerations = (MWNumericArray)controllerLib.TransportationControl(1,Transform2Positions(agentPose),Transform2Positions(agentDest),AgentPose2Positions(agentPrevPose),kh[0],kh[1],kg[0],kg[1],ks[0],ks[1],kgm[0],kgm[1],kth[0],kth[1],alpha_H,alpha_G,dt).GetValue(0);
            // Debug.LogWarning("CONTROL::" + accelerations);
            
            // We convert the output to a Vector3 to use it as accelerations within the agents
            MWNumericArray2Vector3(accelerations);

            // We update the previous agentPose
            for(int i = 0; i < n_agents; ++i)
                // Store the initial poses
                agentPrevPose[i] = new Agent.pose(agentPose[i].position,agentPose[i].rotation);

            // We send the accelerations to the agents
            SendAccels();

        }

        if(!start_control && agentsGrabbed)
            AllAgentsRelease();
        
        // If we are not performing any control we stop all the agents
        if(!start_control)
            StopAgents();

    }
    #endregion Main Methods

    #region Custom methods

    // ------- Setting Up the Controller -------
    MWNumericArray Transform2Positions(List<Transform> Transformations){
        float[,] positions = new float[2,n_agents];

        for(int i = 0; i < n_agents; ++i){
            positions[0,i] = Transformations[i].position.x;
            positions[1,i] = Transformations[i].position.z;
            //positions[1,i] = Transformations[i].position.y;
            //positions[2,i] = Transformations[i].position.z;
        }

        return new MWNumericArray(positions);
    }

    MWNumericArray AgentPose2Positions(List<Agent.pose> Poses){
        float[,] positions = new float[2,n_agents];

        for(int i = 0; i < n_agents; ++i){
            positions[0,i] = Poses[i].position.x;
            positions[1,i] = Poses[i].position.z;
            //positions[1,i] = Poses[i].position.y;
            //positions[2,i] = Poses[i].position.z;
        }

        return new MWNumericArray(positions);
    }

    // ------- Controller output Handle -------
    private void MWNumericArray2Vector3(MWNumericArray accelerations){
        double[,] accels = (double[,])accelerations.ToArray(MWArrayComponent.Real);
        for(int i = 0; i < n_agents; ++i)
            agentAccel[i] = new Vector3((float)accels[0,i], 0.0f, (float)accels[1,i]);
    }

    // ------- Agents -------
    private void AllAgentsGrab(){
        foreach(Agent agent in listAgents)
            agent.set_activeGrab(true);
        
        agentsGrabbed = true;
    }
    
    private void AllAgentsRelease(){
        foreach(Agent agent in listAgents)
            agent.set_activeGrab(false);
        
        agentsGrabbed = false;
    }

    private void SendAccels(){
        for(int i = 0; i < n_agents; ++i)
            listAgents[i].set_accel(agentAccel[i]);
    }

    private void StopAgents(){
        foreach(Agent agent in listAgents){
            if(!agent.isAgentStopped())
                agent.StopAgent();
        }
        
        agentsActivated = false;
    }

    private void ActivateAgents(){
        foreach(Agent agent in listAgents){
            if(agent.isAgentStopped())
                agent.ActivateAgent();
        }

        agentsActivated = true;
    }

    // ------- Error Computation -------
    // Computes the overall error of the agents position
    private bool isCloseEnough(float threshold){
        float pose_error = 0.0f;
        bool closeEnough = false;

        for(int i = 0; i < n_agents; ++i)
            pose_error += Vector3.Distance(agentPose[i].position, agentDest[i].position);
        
        currPose_error = pose_error;
        if(pose_error < threshold)
            closeEnough = true;
        
        return closeEnough;
    } 

    // ------- Drawing -------
    private void DrawDestination(){
        for(int i = 0; i < n_agents; ++i){
            if(i+1 >= n_agents)
                Debug.DrawLine(agentDest[i].position, agentDest[0].position,Color.blue);
            else
                Debug.DrawLine(agentDest[i].position, agentDest[i+1].position,Color.blue);
        }
    }

    #endregion Custom methods
}
