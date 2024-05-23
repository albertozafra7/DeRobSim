using System.Collections;
using System.Collections.Generic;
using UnityEngine;


namespace Deform.Deformers
{
    public class GrabbedObject : DeformerComponent
    {
        #region Public Properties

        public GameObject grabber;
        public bool active_grab = false;
        public float strength;
        public float grab_radius;

        #endregion Public Properties

        #region Private Properties

        private Matrix4x4 TransformMatrix;

        private Matrix4x4 invTransformMatrix;

        private TransformData cached_transform;


        #endregion Private Properties

        #region Public Methods

        public void set_grabber(GameObject new_grabber){
            grabber = new_grabber;
        }

        public void set_active_grab(bool new_active_grab){
            active_grab = new_active_grab;
        }

        #endregion Public Methods

        #region Deform Methods
        public override void PreModify(){

            base.PreModify();

            if(grabber == null){
                grabber = new GameObject ("Grabber_" + this.name);
            }

            cached_transform = new TransformData(grabber.transform);

            TransformMatrix = Matrix4x4.TRS(cached_transform.position,cached_transform.rotation,cached_transform.localScale);
            invTransformMatrix = TransformMatrix.inverse;

        }

        public override VertexData[] Modify (VertexData[] vertexData, TransformData transformData, Bounds meshBounds){

            if(grabber != null){
            
                float max_dist = 10000000.0f;
                for(int i = 0; i < vertexData.Length; i++){
                    //Debug.Log(i + " Vertex is at " + vertexData[i].position);
                    if(Vector3.Distance(vertexData[i].basePosition,cached_transform.position) < max_dist)
                        max_dist = Vector3.Distance(vertexData[i].basePosition,cached_transform.position);
                    if(Vector3.Distance(vertexData[i].position, cached_transform.position) <= grab_radius && active_grab){
                        //Debug.Log("Vertex " + i + " Within grabbing radius");
                        Vector3 vertex_pos_graberspc = TransformMatrix.MultiplyPoint3x4(vertexData[i].position);

                        float position_deform = Vector3.Distance(vertexData[i].position, cached_transform.position) + strength; // + strength???

                        vertex_pos_graberspc = vertex_pos_graberspc * position_deform;

                        Vector3 rotation_dir = cached_transform.position - vertexData[i].position;
                        Quaternion rotation_deform = Quaternion.LookRotation(rotation_dir, Vector3.up);

                        vertex_pos_graberspc = rotation_deform * vertex_pos_graberspc;

                        vertexData[i].position = invTransformMatrix.MultiplyPoint3x4(vertex_pos_graberspc);
                    }
                }
                Debug.Log("The closest vertex is at " + max_dist.ToString("F5") + " from the grabber");
                //Debug.Log("The grabber position is= " + cached_transform.position);
            }
            return vertexData;
        }

        #endregion Deform Methods
    }
}