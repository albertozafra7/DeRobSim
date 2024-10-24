using System.Collections;
using System.Collections.Generic;
using System; 
using System.Text; 
using UnityEngine;
using Unity.Mathematics;
// Matlab Libraries
using MathWorks.MATLAB.NET.Arrays; // import from MWArray.dll
using MatlabControlLib;            // import the custom control Matlab Library
// For debugging
using NVIDIA.Flex;

public class TransportControl2D : MonoBehaviour
{
    #region Properties
    //--------- Public ---------
    [Header("Agents")]
    public List<Agent> listAgents = new List<Agent>();              // Agents for being controlled
    [Header("Agent Poses")]
    public List<Transform> agentPose = new List<Transform>();       // Current pose (position and rotation) of the agents
    public List<Transform> agentDest = new List<Transform>();       // Destination pose of the agents

    //+++ Control params +++

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

    [Header("Destination Modifiers")]
    public float dest_scale = 1.0f;                                 // It scales up or down the destination shape
    public float dest_rotation = 0.0f;                              // It rotates the destination shape

    // Control Initialization
    [Header("Control Initialization")]
    public bool start_control = false;                              // Boolean used for starting the control

    // Debug variables
    [Header("Debug")]
    public bool draw_destiny = true;                                // Boolean used for drawing the destination of the object
    public bool log_simulation = false;                             // Boolean used for saving the control parameters
    public string log_fileName = "TransportControl2D_log";       // Filename used for the log file
    public string log_filePath = "SimulationLogs";                  // Filepath used for the log file

    public float saveInterval = 0.25F;

    //--------- Private ---------
    private int n_agents;
    private float2x2 Rot90M = new float2x2(0.0f, -1.0f, 1.0f, 0.0f); // 90 deg rotation matrix to be used in the control
    private List<Agent.pose> agentPrevPose = new List<Agent.pose>(); // Previous agent pose
    private MatlabControlLib.MatlabControlLib controllerLib = new MatlabControlLib.MatlabControlLib();  // Object that uses the Matlab Library for the control algorithm
    private Vector3[] agentAccel;
    private bool agentsGrabbed = false;
    private bool agentsActivated = true;
    private float currPose_error = float.PositiveInfinity; // Determine the system current error
    private DebugLogger simLogger;  // Object used for logging the simulation
    private int n_matlabParams = 16; // Number of outputs received from matlab when debugging

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
        // Prepare the control
        n_agents = listAgents.Count;

        for(int i = 0; i < n_agents; ++i){
            // Store the initial poses
            agentPrevPose.Add(new Agent.pose(agentPose[i].position,agentPose[i].rotation));
            // Modify the agent dt to coordinate the whole control
            listAgents[i].set_dt(dt);
        }

        // Drawing
        if(draw_destiny)
            DrawDestination();

        // Resize of the agents acceleration
        agentAccel = new Vector3[n_agents];

        // Agent activation for using them
        ActivateAgents();

        // Debug log
        if(simLogger != null)
            PrepareDebugLogger();
    }

    // Update is called once per frame
    void Update()
    {
        // ------- Drawing part ------- 
        // We draw the destiny if it has not been drawn
        if(draw_destiny)
            DrawDestination();

        // ------- Agent Grab -------
        if(start_control && !agentsGrabbed)
            AllAgentsGrab();

        // ------- Control part -------
        if(start_control){
            if(!agentsActivated)
                ActivateAgents();

            // We call the main control algorithm
            MWNumericArray accelerations;
            
            if(log_simulation){ // If we want to log the simulation
                // We get the whole list of results
                // MWArray[] control_results = controllerLib.TransportationControl2D_Debug(n_matlabParams,Transform2Positions(agentPose),Transform2Positions(agentDest),AgentPose2Positions(agentPrevPose),kh[0],kh[1],kg[0],kg[1],ks[0],ks[1],kgm[0],kgm[1],kth[0],kth[1],alpha_H,alpha_G,dt,dest_scale,dest_rotation);
                MWArray[] control_results = controllerLib.TransportationControl2D_Debug(n_matlabParams,Transform2Positions(agentPose),Transform2Positions(agentDest),AgentPose2Positions(agentPrevPose),Time.realtimeSinceStartup,kh[1],kg[0],kg[1],ks[0],ks[1],kgm[0],kgm[1],kth[0],kth[1],alpha_H,alpha_G,dt,dest_scale,dest_rotation);

                // First we filter from the control results the accelerations that are going to be sent to the agents
                accelerations = (MWNumericArray)control_results.GetValue(0);

                // If we do not have any debugLogger created we create it
                if(simLogger == null){
                    simLogger = gameObject.AddComponent<DebugLogger>();
                    PrepareDebugLogger();
                }

                // We update the Dictionary with the matlab params
                UpdateMatlabParams(control_results);

                // We save the logs on the file
                simLogger.saveData();
                

            } else // Else we only retrieve the accelerations
                accelerations = (MWNumericArray)controllerLib.TransportationControl2D(1,Transform2Positions(agentPose),Transform2Positions(agentDest),AgentPose2Positions(agentPrevPose),kh[0],kh[1],kg[0],kg[1],ks[0],ks[1],kgm[0],kgm[1],kth[0],kth[1],alpha_H,alpha_G,dt,dest_scale,dest_rotation).GetValue(0);
            
            // We convert the output to a Vector3 to use it as accelerations within the agents
            MWNumericArray2Accels(accelerations);

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
    private void MWNumericArray2Accels(MWNumericArray accelerations){
        double[,] accels = (double[,])accelerations.ToArray(MWArrayComponent.Real);
        for(int i = 0; i < n_agents; ++i)
            agentAccel[i] = new Vector3((float)accels[0,i], 0.0f, (float)accels[1,i]);
    }

    private double[,] MWNumericArray2UnityArray(MWNumericArray matlabArray){
        return (double[,])matlabArray.ToArray(MWArrayComponent.Real);
    }

    private Vector3[] MWNumericArray2Vector3(MWNumericArray matlabArray){
        double[,] array = (double[,])matlabArray.ToArray(MWArrayComponent.Real);
        Vector3[] vector3s = new Vector3[n_agents];
        for(int i = 0; i < n_agents; ++i)
            vector3s[i] = new Vector3((float)array[0,i], 0.0f, (float)array[1,i]);
        
        return vector3s;
    }

    private double MWNumericArray2DoubleScalar(MWNumericArray matlabArray){
        return matlabArray.ToScalarDouble();
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

    // ------- Debug -------
    private void PrepareDebugLogger(){
        // We prepare the debug logger
        // We set the filepath
        simLogger.fileName = log_fileName + "_" +DateTime.Now.ToString("yyyy-MM-dd_HH-mm-ss") + ".h5";
        simLogger.filePath = log_filePath;

        // We set the saveInterval
        simLogger.updateSaveInterval = saveInterval;

        // We intialize the debug logger
        simLogger.InitializeData();

        // We set the agents
        simLogger.registeredAgents = listAgents;
        // We set the agent destinations
        simLogger.registeredTransforms = agentDest;
        // We set the flexObjects
        simLogger.registeredFlexObjects = new List<FlexActor>(FindObjectsOfType<FlexActor>());

        // We add the dictionary with all the params
        
        // - accel
        simLogger.registeredMatlabParams.Add("acceleration", null);
        // - U_f
        simLogger.registeredMatlabParams.Add("U_f", null);
        // - U_D
        simLogger.registeredMatlabParams.Add("U_D", null);
        // - U_g
        simLogger.registeredMatlabParams.Add("U_g", null);
        // - U_th
        simLogger.registeredMatlabParams.Add("U_th", null);
        // - U_gamma
        simLogger.registeredMatlabParams.Add("U_gamma", null);
        // - U_s
        simLogger.registeredMatlabParams.Add("U_s", null);
        // - U_H
        simLogger.registeredMatlabParams.Add("U_H", null);
        // - U_G
        simLogger.registeredMatlabParams.Add("U_G", null);
        // - Positions
        simLogger.registeredMatlabParams.Add("positions", null);
        // - Destinations
        simLogger.registeredMatlabParams.Add("destinations", null);
        // - Prev_Positions
        simLogger.registeredMatlabParams.Add("prev_positions", null);
        // +++++++ ERRORS +++++++
        // - gamma
        simLogger.registeredMatlabScalars.Add("gamma", Double.NaN);
        // - eg
        simLogger.registeredMatlabScalars.Add("eg", Double.NaN);
        // - es
        simLogger.registeredMatlabScalars.Add("es", Double.NaN);
        // - eth
        simLogger.registeredMatlabScalars.Add("eth", Double.NaN);
    }

    private void UpdateMatlabParams(MWArray[] matlabParams){
        // - accel
        simLogger.registeredMatlabParams["acceleration"] = MWNumericArray2Vector3((MWNumericArray)matlabParams.GetValue(0));
        // - U_f
        simLogger.registeredMatlabParams["U_f"] = MWNumericArray2Vector3((MWNumericArray)matlabParams.GetValue(1));
        // - U_D
        simLogger.registeredMatlabParams["U_D"] = MWNumericArray2Vector3((MWNumericArray)matlabParams.GetValue(2));
        // - U_g
        simLogger.registeredMatlabParams["U_g"] = MWNumericArray2Vector3((MWNumericArray)matlabParams.GetValue(3));
        // - U_th
        simLogger.registeredMatlabParams["U_th"] = MWNumericArray2Vector3((MWNumericArray)matlabParams.GetValue(4));
        // - U_gamma
        simLogger.registeredMatlabParams["U_gamma"] = MWNumericArray2Vector3((MWNumericArray)matlabParams.GetValue(5));
        // - U_s
        simLogger.registeredMatlabParams["U_s"] = MWNumericArray2Vector3((MWNumericArray)matlabParams.GetValue(6));
        // - U_H
        simLogger.registeredMatlabParams["U_H"] = MWNumericArray2Vector3((MWNumericArray)matlabParams.GetValue(7));
        // - U_G
        simLogger.registeredMatlabParams["U_G"] = MWNumericArray2Vector3((MWNumericArray)matlabParams.GetValue(8));
        // - Positions
        simLogger.registeredMatlabParams["positions"] = MWNumericArray2Vector3((MWNumericArray)matlabParams.GetValue(9));
        // - Destinations
        simLogger.registeredMatlabParams["destinations"] = MWNumericArray2Vector3((MWNumericArray)matlabParams.GetValue(10));
        // - Prev_Positions
        simLogger.registeredMatlabParams["prev_positions"] = MWNumericArray2Vector3((MWNumericArray)matlabParams.GetValue(11));
        // +++++++ ERRORS +++++++
        // - gamma
        simLogger.registeredMatlabScalars["gamma"] = MWNumericArray2DoubleScalar((MWNumericArray)matlabParams.GetValue(12));
        // - eg
        simLogger.registeredMatlabScalars["eg"] = MWNumericArray2DoubleScalar((MWNumericArray)matlabParams.GetValue(13));
        // - es
        simLogger.registeredMatlabScalars["es"] = MWNumericArray2DoubleScalar((MWNumericArray)matlabParams.GetValue(14));
        // - eth
        simLogger.registeredMatlabScalars["eth"] = MWNumericArray2DoubleScalar((MWNumericArray)matlabParams.GetValue(15));        

    }


    #endregion Custom methods
}
