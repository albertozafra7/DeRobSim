using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using PureHDF; // HDF5 Manager
using System.IO;
using NVIDIA.Flex;

public class DebugLogger : MonoBehaviour
{

    #region Properties
    //--------- Public ---------
    // +++ File Properties +++
    [Header("HDF5 File")]
    public string fileName = "simulation_data.h5";      // Name of file where the log is going to be stored
    public string filePath = "SimulationLogs";          // Assets/filePath where the file is going to be stored
    //public bool appendTimestamp2file = false;         // It determines if the current date and time will be appended to the filename
    public bool saveLog = true;                         // Boolean for storing the data

    [Header("Save Frequency")]
    public float updateSaveInterval = 0.25F;

    // +++ Objects to be logged +++
    [Header("GameObjects")]
    public List<Transform> registeredTransforms = new List<Transform>();    // List that will keep all the standard gameobjects that we want to save (It will use the GameObject name as the name of the group in the HDF5 file)
    
    [Header("Agents")]
    public List<Agent> registeredAgents = new List<Agent>();                // List that will keep track of all the agents present on the simulation

    [Header("Deformable Objects")]
    public List<FlexActor> registeredFlexObjects = new List<FlexActor>();   // List of deformable objects that we want to keep track (remember that under the name of the GameObject, the particles' properties will be stored)

    [Header("Control Params")]
    public Dictionary<string,Vector3[]> registeredMatlabParams = new Dictionary<string,Vector3[]>();    // Dictionary used to register the 3xn_agents control values
    public Dictionary<string,double> registeredMatlabScalars = new Dictionary<string, double>();        // Dictionary used to register the scalar control parameters

    // +++ Control sequence param +++
    [Header("File Reseting")]
    public bool ResetStoring = false;                   // If you set this boolean to true, the saving will be reset

    //--------- Private ---------
    private string absoluteFilePath;
    private H5File h5file = new H5File();
    private float lastSaveInterval;

    #endregion Properties

    #region Main Methods
    // Start is called before the first frame update
    void Start()
    {
        absoluteFilePath = Path.Combine(Application.dataPath, filePath, fileName);
        
        // We add the Unity Group
        h5file["Unity"] = new H5Group();
        // We add the Matlab Group
        h5file["Matlab"] = new H5Group();

        // We update the lastSaveInterval
        lastSaveInterval = float.MinValue; // We make it the minimum value to store the first frame information
    }

    // Update is called once per frame
    void Update()
    {
        if(saveLog)
            saveData();

        if(ResetStoring)
            resetData();
        
        if(!string.Equals(absoluteFilePath,Path.Combine(Application.dataPath, filePath, fileName)))
            absoluteFilePath = Path.Combine(Application.dataPath, filePath, fileName);
    }

    void OnDestroy(){
        // We write the information into the file
        h5file.Write(absoluteFilePath);
    }
    
    #endregion Main Methods

    #region Custom Methods

    public void InitializeData(){
        absoluteFilePath = Path.Combine(Application.dataPath, filePath, fileName);
        
        // We add the Unity Group
        h5file["Unity"] = new H5Group();
        // We add the Matlab Group
        h5file["Matlab"] = new H5Group();
    }

    public void saveData(){
        // If we have not saved for a long time
        float timeNow = Time.realtimeSinceStartup;
        if(timeNow - lastSaveInterval > updateSaveInterval){

            string currentTime = Time.realtimeSinceStartup.ToString();
            
            // We add a new group based on the current timestamp
            H5Group UnityRootGroup = (H5Group)h5file["Unity"];   // Pointer to the Unity's group in the HDF5
            UnityRootGroup[currentTime] = new H5Group();         // We create the new timestamp group

            // We add the different groups to the HDF5 file with the required information
            AddTransforms((H5Group)UnityRootGroup[currentTime]);
            AddAgents((H5Group)UnityRootGroup[currentTime]);
            AddDeformableObjects((H5Group)UnityRootGroup[currentTime]);

            // If we are logging the control params we add the Matlab info to the file
            if(registeredMatlabParams.Count > 0){
                // We get the pointer to the current group
                H5Group matlabRootGroup = (H5Group)h5file["Matlab"];
                matlabRootGroup[currentTime] = new H5Group();
                // We add the params to the HDF5 file
                AddMatlabParams((H5Group)matlabRootGroup[currentTime]);
            }

            lastSaveInterval = Time.realtimeSinceStartup;
            
        }
   
    }

    public void resetData(){
        // We reset the root group
        h5file["Unity"] = new H5Group();
        h5file["Matlab"] = new H5Group();
        ResetStoring = false;
    }


    private void AddTransforms(H5Group timestampGroup){
        // If we are tracking some standard gameObjects
        if(registeredTransforms.Count > 0){
            // We get the root group for the standard gameObjects
            timestampGroup["GameObjects"] = new H5Group();
            H5Group GameObjectGroup = (H5Group)timestampGroup["GameObjects"];

            // For each gameObject that we want to track we generate a new group that contains its Transform
            foreach(Transform standard_object in registeredTransforms){

                GameObjectGroup[standard_object.gameObject.name] = new H5Group()
                {
                    ["position"] = standard_object.position,
                    ["rotation"] = standard_object.rotation
                };
            }
        }
    }

    private void AddAgents(H5Group timestampGroup){
        // If we are tracking some agents present in the control
        if(registeredAgents.Count > 0){
            // We get the root group for the agents
            timestampGroup["Agents"] = new H5Group();
            H5Group AgentsGroup = (H5Group)timestampGroup["Agents"];

            // For each agent we generate a new group with its attributes
            foreach(Agent agent in registeredAgents){

                AgentsGroup[agent.gameObject.name] = new H5Group()
                {
                    ["position"] = agent.get_position(),
                    ["rotation"] = agent.get_rotation(),
                    ["velocity"] = agent.get_vel(),
                    ["angular velocity"] = agent.get_angVel(),
                    ["acceleration"] = agent.get_accel(),
                    ["angular acceleration"] = agent.get_angAccel(),
                    ["grabbed"] = agent.get_activeGrab()

                };
            }
        }
    }

    private void AddDeformableObjects(H5Group timestampGroup){
        // If we are tracking some deformable objects
        if(registeredFlexObjects.Count > 0){
            // We get the root group for the deformable objects
            timestampGroup["DeformableObjects"] = new H5Group();
            H5Group DeformableObjectGroup = (H5Group)timestampGroup["DeformableObjects"];

            // For each deformable object we get some standard information
            foreach(FlexActor defobject in registeredFlexObjects){

                // We get the mesh of the deformable object for storing its vertices
                SkinnedMeshRenderer defSkin = defobject.gameObject.GetComponent<SkinnedMeshRenderer>();
                Mesh defmesh = new Mesh();

                defSkin.BakeMesh(defmesh);

                // We get a copy of the vertices position
                Vector3[] defVerts = defmesh.vertices;
                // We transform them to the world coordinates
                defobject.gameObject.transform.TransformPoints(defVerts);

                // We get the tetrahedral mesh information (if any)
                TetrahedralMeshTracking deftets = defobject.gameObject.GetComponent<TetrahedralMeshTracking>();

                if(deftets != null)
                {
                    DeformableObjectGroup[defobject.gameObject.name] = new H5Group()
                    {
                        ["position"] = defobject.gameObject.transform.position,
                        ["rotation"] = defobject.gameObject.transform.rotation,
                        ["n_particles"] = defobject.GetParticleNum(),
                        ["particles_info"] = defobject.GetParticles(),
                        ["n_vertices"] = defmesh.vertexCount,
                        ["vertex_info"] = defVerts,
                        ["mesh_triangles"] = defobject.GetTriangles(),
                        ["n_triangles"] = defobject.GetTriangles().Length,
                        ["n_tetrahedra"] = deftets.GetTetrahedraNum(),
                        ["mesh_tetrahedrons"] = deftets.tetrahedrons2Vector4(),
                        ["n_tetverts"] = deftets.GetTetVertsNum(),
                        ["tetverts_info"] = deftets.GetTetVertsPosition()
                    };
                } else {
                    DeformableObjectGroup[defobject.gameObject.name] = new H5Group()
                    {
                        ["position"] = defobject.gameObject.transform.position,
                        ["rotation"] = defobject.gameObject.transform.rotation,
                        ["n_particles"] = defobject.GetParticleNum(),
                        ["particles_info"] = defobject.GetParticles(),
                        ["n_vertices"] = defmesh.vertexCount,
                        ["vertex_info"] = defVerts,
                        ["mesh_triangles"] = defobject.GetTriangles(),
                        ["n_triangles"] = defobject.GetTriangles().Length
                    };
                }

            }
        }
    }

    /*private void storeParticlesInfo(H5Group particlesGroup, FlexActor DefObj){


    }*/

    private void AddMatlabParams(H5Group timestampGroup){
        // If we are tracking some control params (double check)
        if(registeredMatlabParams.Count > 0){
            // We add a dataset per param
            foreach(KeyValuePair<string, Vector3[]> kvp in registeredMatlabParams){
                timestampGroup[kvp.Key] = kvp.Value;
            }
        }
        
        if(registeredMatlabScalars.Count > 0){
            // We add a dataset per param
            foreach(KeyValuePair<string, double> kvp in registeredMatlabScalars){
                timestampGroup[kvp.Key] = kvp.Value;
            }
        }
    }


    #endregion Custom Methods
}
