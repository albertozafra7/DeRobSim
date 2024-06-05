using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using NVIDIA.Flex;
using System.IO;
using UnityEditor;

public class Container_Creation : MonoBehaviour
{
    public bool create = false;
    private bool creating = false;
    public int n_particles = 1000;
    public string cont_name = "Default_container";

    private Queue<string> createdAssets = new Queue<string>();

    //private FlexEditor flexEditor;

    public string path = "Assets/Deformable_Objects/";


    void Start()
    {
        
    }


    void Update()
    {  

        if (create)
        {
            if(!creating){
                creating = true;
                // Create container
                CreateAsset(cont_name);

            }
        }
        else
            creating = false;
    }

    void OnDestroy()
    {

        while (createdAssets.Count > 0)
        {
            AssetDatabase.DeleteAsset(createdAssets.Dequeue());
        }

    }


    public FlexContainer CreateAsset(string _name = "")
    {
        FlexContainer asset = ScriptableObject.CreateInstance<FlexContainer>();

        asset.SetMaxParticles(n_particles);

        // string path = AssetDatabase.GetAssetPath(Selection.activeObject);
        if (path == "") path = "Assets";
        else if (Path.GetExtension(path) != "") path = path.Replace(Path.GetFileName(AssetDatabase.GetAssetPath(Selection.activeObject)), "");

        if (_name == "") _name = typeof(FlexContainer).ToString();
        string assetPathAndName = AssetDatabase.GenerateUniqueAssetPath(path + "/" + _name + ".asset");

        AssetDatabase.CreateAsset(asset, assetPathAndName);

        AssetDatabase.SaveAssets();
        EditorUtility.FocusProjectWindow();
        Selection.activeObject = asset;

        createdAssets.Enqueue(assetPathAndName);

        return asset;
    }

}