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
    public List<Agent.pose> agentOrigin = new List<Agent.pose>();   // Origin pose (position and rotation) of the agents (just for record)
    public List<Agent.pose> agentDest = new List<Agent.pose>();     // Destination pose of the agents

    //+++ Control params +++
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


    //--------- Private ---------
    private float dt;
    private int n_agents;
    private float2x2 Rot90M = new float2x2(0.0f, -1.0f, 1.0f, 0.0f); // 90 deg rotation matrix to be used in the control


    #endregion Properties

    #region Main Methods

    void Awake()
    {
        dt = Time.deltaTime;
    }

    // Start is called before the first frame update
    void Start()
    {
        n_agents = listAgents.Count;


    }

    // Update is called once per frame
    void Update()
    {
        
    }
    #endregion Main Methods

    #region Custom methods

    #endregion Custom methods
}
