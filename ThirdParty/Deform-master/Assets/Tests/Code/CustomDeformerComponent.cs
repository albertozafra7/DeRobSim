using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace Deform.Deformers{

    public class CustomDeformerComponent : DeformerComponent
    {
        public bool data_to_be_cached;
        
        // If you need to cache any data you must use the PreModify method as it is going to be called from the main thread
        public override void PreModify(){

            // You cache your own specific data
    

        }
        // Main method of the class
        public override VertexData[] Modify (VertexData[] vertexData, TransformData transformData, Bounds meshBounds){
            
            // You apply your deformation here

            return vertexData;
        }

    }

}

