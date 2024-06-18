// This code contains NVIDIA Confidential Information and is disclosed to you
// under a form of NVIDIA software license agreement provided separately to you.
//
// Notice
// NVIDIA Corporation and its licensors retain all intellectual property and
// proprietary rights in and to this software and related documentation and
// any modifications thereto. Any use, reproduction, disclosure, or
// distribution of this software and related documentation without an express
// license agreement from NVIDIA Corporation is strictly prohibited.
//
// ALL NVIDIA DESIGN SPECIFICATIONS, CODE ARE PROVIDED "AS IS.". NVIDIA MAKES
// NO WARRANTIES, EXPRESSED, IMPLIED, STATUTORY, OR OTHERWISE WITH RESPECT TO
// THE MATERIALS, AND EXPRESSLY DISCLAIMS ALL IMPLIED WARRANTIES OF NONINFRINGEMENT,
// MERCHANTABILITY, AND FITNESS FOR A PARTICULAR PURPOSE.
//
// Information and code furnished is believed to be accurate and reliable.
// However, NVIDIA Corporation assumes no responsibility for the consequences of use of such
// information or for any infringement of patents or other rights of third parties that may
// result from its use. No license is granted by implication or otherwise under any patent
// or patent rights of NVIDIA Corporation. Details are subject to change without notice.
// This code supersedes and replaces all information previously supplied.
// NVIDIA Corporation products are not authorized for use as critical
// components in life support devices or systems without express written approval of
// NVIDIA Corporation.
//
// Copyright (c) 2018 NVIDIA Corporation. All rights reserved.

using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace NVIDIA.Flex
{
    [ExecuteInEditMode]
    [DisallowMultipleComponent]
    [AddComponentMenu("")]

    public class FlexActor : MonoBehaviour
    {
        #region Properties

        public FlexContainer container
        {
            get { return m_container; }
            set { m_container = value; }
        }

        public FlexAsset asset
        {
            get { return m_currentAsset; }
        }

        public int particleGroup
        {
            get { return m_particleGroup; }
            set { m_particleGroup = value; }
        }

        public bool selfCollide
        {
            get { return m_selfCollide; }
            set { m_selfCollide = value; }
        }

        public bool fluid
        {
            get { return m_fluid; }
            set { m_fluid = value; }
        }

        public float massScale
        {
            get { return m_massScale; }
            set { m_massScale = value; }
        }

        public bool drawParticles
        {
            get { return m_drawParticles; }
            set { m_drawParticles = value; }
        }

        public FlexExt.Instance.Handle handle
        {
            get { return m_instanceHandle; }
        }

        public int[] indices
        {
            get { return m_indices; }
        }

        public int indexCount
        {
            get { return m_indexCount; }
        }

        public Bounds bounds { get { return m_bounds; } }

        #endregion

        #region Events

        public delegate void OnBeforeRecreateFn();
        public event OnBeforeRecreateFn onBeforeRecreate;

        public delegate void OnAfterRecreateFn();
        public event OnAfterRecreateFn onAfterRecreate;

        public delegate void OnFlexUpdateFn(FlexContainer.ParticleData _particleData);
        public event OnFlexUpdateFn onFlexUpdate;

        #endregion

        #region Methods

        public void Recreate()
        {
            if (onBeforeRecreate != null) onBeforeRecreate();

            if (m_recreateActor == 2)
            {
                DestroyActor();
                CreateActor();
            }
            else if (m_recreateActor == 1)
            {
                DestroyInstance();
                CreateInstance();
            }

            m_recreateActor = 0;

            if (onAfterRecreate != null) onAfterRecreate();
        }

        public void Teleport(Vector3 _targetPosition, Quaternion _targetRotation)
        {
            m_teleportPosition = _targetPosition;
            m_teleportRotation = _targetRotation;
            m_teleport = 5;
        }

        public void ApplyImpulse(Vector3 _impulse, int _particle = -1)
        {
            ImpulseInfo info;
            info.impulse = _impulse;
            info.particle = _particle;
            m_impulses.Add(info);
        }
        
        public void addGrabber(Grabber grabber){
            // If the object is not null and is not already within our class
            Debug.Log("The grabber " + grabber.getGrabber().name + " has been added");
            if(grabber != null && grabberList.Find(a => a.getGrabber().name == grabber.getGrabber().name) == null)
                grabberList.Add(grabber);
            
            // particle_start_id = indices[0];
            // particle_num = indices.Length;
        }

        public void removeGrabber(Grabber grabber){
            if(grabber != null)
                grabberList.Remove(grabber);
        }

        public void resetGrabberList(){
            grabberList.Clear();
        }

        public FlexContainer GetContainer(){
            return m_currentContainer;
        }

        public int GetParticleNum(){
            return particle_num;
        }

        public int GetParticleStartId(){
            return particle_start_id;
        }

        #endregion

        #region Messages

        void OnEnable()
        {
            CreateActor();
        }

        void OnDisable()
        {
            DestroyActor();
        }

        void OnValidate()
        {
            ValidateFields();
            if (m_recreateActor > 0) Recreate();
        }

        void Update()
        {
            if (transform.hasChanged && !Application.isPlaying)
            {
                m_recreateActor = 1;
                Recreate();
                transform.hasChanged = false;
            }
        }

        #endregion

        #region Protected

        protected static Color GIZMO_COLOR = new Color(1.0f, 0.5f, 0.0f);

        protected FlexContainer m_currentContainer;
        protected FlexAsset m_currentAsset;

        protected virtual FlexAsset subclassAsset { get { return null; } }

        protected int m_phase = 0;

        void AddToContainer()
        {
            if (m_container)
            {
                m_currentContainer = m_container;
                m_currentContainer.onFlexUpdate += OnFlexUpdate;
                m_currentContainer.onBeforeRecreate += OnBeforeRecreate;
                m_currentContainer.onAfterRecreate += OnAfterRecreate;
                m_currentContainer.onBeforeDestroy += OnBeforeDestroy;
                m_currentContainer.AddActor(this);
            }
        }
        

        void RemoveFromContainer()
        {
            if (m_currentContainer)
            {
                m_currentContainer.onFlexUpdate -= OnFlexUpdate;
                m_currentContainer.onBeforeRecreate -= OnBeforeRecreate;
                m_currentContainer.onAfterRecreate -= OnAfterRecreate;
                m_currentContainer.onBeforeDestroy -= OnBeforeDestroy;
                m_currentContainer.RemoveActor(this);
                m_currentContainer = null;
            }
        }

        void AcquireAsset()
        {
            if (subclassAsset)
            {
                m_currentAsset = subclassAsset;
                m_currentAsset.onBeforeRebuild += OnBeforeRecreate;
                m_currentAsset.onAfterRebuild += OnAfterRecreate;
            }
        }

        void ReleaseAsset()
        {
            if (m_currentAsset)
            {
                m_currentAsset.onBeforeRebuild -= OnBeforeRecreate;
                m_currentAsset.onAfterRebuild -= OnAfterRecreate;
                m_currentAsset = null;
            }
        }

        protected virtual void CreateInstance()
        {
            if (m_currentContainer && m_currentContainer.handle && m_currentAsset && m_currentAsset.handle)
            {
                int group = m_particleGroup < 0 ? ++sm_nextGroup : m_particleGroup;
                Flex.Phase flags = Flex.Phase.Default;
                if (m_selfCollide) flags |= Flex.Phase.SelfCollide;
                if (m_selfCollideFilter) flags |= Flex.Phase.SelfCollideFilter;
                if (m_fluid) flags |= Flex.Phase.Fluid;
                m_phase = Flex.MakePhase(group, flags);
                m_instanceHandle = m_currentContainer.CreateInstance(m_currentAsset.handle, transform.localToWorldMatrix, Vector3.zero, m_phase, m_massScale);
                if (m_instanceHandle)
                {
                    FlexExt.Instance instance = m_instanceHandle.instance;
                    m_indices = new int[m_currentAsset.maxParticles];
                    // Debug.LogError("Max Particles = " + m_currentContainer.maxParticles);
                    m_indexCount = instance.numParticles;
                    if (m_indexCount > 0){
                        FlexUtils.FastCopy(instance.particleIndices, 0, ref m_indices[0], 0, sizeof(int) * m_indexCount);
                        particle_start_id  = m_indices[0];
                        // particle_start_id = m_currentContainer.maxParticles - m_indices[0] - m_indices.Length;
                        particle_num = m_indices.Length;           // m_currentContainer.maxParticles;            
                    } 
                    // Debug.LogError("Start index =" + m_indices[0]);
                    // Debug.LogError("Last index =" + m_indices[m_indices.Length-1]);
                    // Debug.LogError("size =" + m_indices.Length);
                }
            }
        }

        protected virtual void DestroyInstance()
        {
            if (m_instanceHandle)
            {
                m_currentContainer.DestroyInstance(m_instanceHandle);
                m_instanceHandle.Clear();
                m_indices = new int[0];
            }

            if (m_drawParticlesHelper)
            {
                if (gameObject.activeInHierarchy) StartCoroutine("DestroyDelayed", m_drawParticlesHelper.gameObject);
                else DestroyImmediate(m_drawParticlesHelper.gameObject);
                m_drawParticlesHelper = null;
            }
        }

        IEnumerator DestroyDelayed(GameObject go)
        {
            yield return new WaitForEndOfFrame();
            DestroyImmediate(go);
        }

        protected virtual void ValidateFields()
        {
            m_massScale = Mathf.Max(m_massScale, 0.01f);
        }
        //---------------------------------------------------
        private float[] FindNextTo(Vector4[] allParticles, Vector3 pos)
        {
            float helpDistMin = 1000;
            float indicesMin = -1;
            float[] ret = new float[2];
            Vector3[] vec3 = new Vector3[allParticles.Length];
            for (int i = 0; i < allParticles.Length; i++)
            {
                vec3[i] = allParticles[i];
            }
            for (int i = 0; i < allParticles.Length; i++)
            {
                if (Vector3.Distance(pos, vec3[i]) < helpDistMin)
                {
                    helpDistMin = Vector3.Distance(pos, vec3[i]);
                    indicesMin = i;
                }

            }

            ret[0] = indicesMin;
            ret[1] = helpDistMin;

            return ret;
        }

        private List<int> FindParticleInRadius(Vector4[] allParticles, Vector3 pos, float radius)
        {
            List<int> particlesUnderRadius = new List<int>();

            Vector3[] vec3 = new Vector3[allParticles.Length];
            for (int i = 0; i < allParticles.Length; i++)
            {
                vec3[i] = allParticles[i];
            }

            for (int i = 0; i < allParticles.Length; i++)
            {
                if (Vector3.Distance(pos, vec3[i]) <= radius)
                {
                    particlesUnderRadius.Add(i);
                }
            }

            // GameObject[] arrayOfGameObjects = particlesUnderRadius.ToArray();

            return particlesUnderRadius;

        }

        // 10000
        Vector4[] allParticles = new Vector4[10000];

        // Custom Grabber
        List<Grabber> grabberList = new List<Grabber>();

        int idNextPart;

        int particle_start_id = 0;  // Starting particle id of the actor within the container
        int particle_num = 0;    // Ending particle id of the actor within the container

        private bool setRestPos = true;
        Vector4[] particlesInitPos = new Vector4[10000];
        public static bool resFatPos = false;
        protected virtual void OnFlexUpdate(FlexContainer.ParticleData _particleData)
        {
            //Debug.LogError("Start? " + m_indices[m_indices.Length - 1]);
            UpdateDrawParticles();
            //Debug.LogError("End " + particle_num);

            if (setRestPos)
            {
                _particleData.GetParticles(particle_start_id,particle_num,particlesInitPos);
                // _particleData.GetParticles(0,particle_num,particlesInitPos);
                setRestPos = false;
            }
            
            // if (m_currentAsset.name == "Fat_FlexSoftAsset")
            // {
                if (resFatPos)
                {
                    _particleData.SetParticles(particle_start_id,particle_num,particlesInitPos);
                    // _particleData.SetParticles(0,particle_num,particlesInitPos);
                    resFatPos = false;
                }
                
                

                //pick all particles
                _particleData.GetParticles(particle_start_id,particle_num, allParticles);
                // _particleData.GetParticles(0,particle_num, allParticles);

                //if grab move particle in cube position
                if(grabberList.Exists(a => a.isGrabbing() == true)){
                    List<Grabber> auxList = grabberList.FindAll(a => a.getActiveGrab());
                    Debug.Log("ActiveGrab");
                    foreach(Grabber g in auxList){
                        foreach (int idPart in g.getDetectedParticles())
                            {
                                // Debug.LogError("Picked particle at " + idPart + "");
                                _particleData.SetParticle(particle_start_id + idPart, new Vector4(g.getGrabberpos().x, g.getGrabberpos().y, g.getGrabberpos().z, 0));
                            }
                    }
                }
                

                //check if trigger is active and there is a particle next
                if(grabberList.Exists(a => a.isOnDetecting() == true)){
                    List<Grabber> auxList = grabberList.FindAll(a => a.isOnDetecting());
                    Debug.Log("OneTimePick");
                    foreach(Grabber g in auxList){
                        Debug.Log("Closest particle at " + g.GetDistanceToClosestParticle(allParticles));
                        if (g.GetDistanceToClosestParticle(allParticles) <= g.getDetectionRadius()) { // Maybe change for a method within the class
                            //particlesUnderRadius = FindParticleInRadius(allParticles, grabber_.transform.position, 0.06f);
                            g.DetectParticles(allParticles);
                            Debug.Log("Detected Particles Count " + g.getDetectedParticlesCount());
                            if(g.getDetectedParticlesCount() > 0)
                                g.setOnGrabbing();
                            // if (g.getDetectedParticlesCount() != 0)
                            // {

                            //     oneTimePick = false;
                            //     oneTimeRel = true;
                            //     activeGrab = true;
                            // }
                        }
                    }
                }
                    
                if(grabberList.Exists(a => a.isReleasing() == true)){
                    //release particle ( W component = 1 reactivate gravity)
                    //if ((oneTimeRel) && relGrasp)
                    //{
                    List<Grabber> auxList = grabberList.FindAll(a => a.isReleasing());
                    Debug.Log("Release");
                    foreach (Grabber g in auxList){
                        foreach (int idPart in g.getDetectedParticles())
                        {
                            _particleData.SetParticle(particle_start_id + idPart, new Vector4(g.getGrabberpos().x, g.getGrabberpos().y, g.getGrabberpos().z, 1));
                        }
                        g.DetectParticles(allParticles);

                        if(g.getDetectedParticlesCount() == 0)
                            g.setOnReleased();
                    }

                    //}  
                }

              

                if (transform.hasChanged && Application.isPlaying)
                {
                    MoveFixedParticles(_particleData);
                    //if (Application.isPlaying) MoveFixedParticles(_particleData);
                    //else { m_recreateActor = 1; Recreate(); }
                    transform.hasChanged = false;
                }
            //}
            /*
            if (transform.hasChanged && Application.isPlaying)
            {
                MoveFixedParticles(_particleData);
                //if (Application.isPlaying) MoveFixedParticles(_particleData);
                //else { m_recreateActor = 1; Recreate(); }
                transform.hasChanged = false;
            }
            */
            if (m_teleport > 0)
            {
                TeleportParticles(_particleData);
                --m_teleport;
            }

            if (m_impulses.Count > 0)
            {
                ApplyImpulses(_particleData);
                m_impulses.Clear();
            }

            UpdateBounds(_particleData);

            if (onFlexUpdate != null) onFlexUpdate(_particleData);
        }

        protected void SetIndices(int[] _indices, int _count)
        {
            m_indexCount = Mathf.Min(_count, m_currentAsset.maxParticles);
            Array.Copy(_indices, m_indices, m_indexCount);
            FlexExt.Instance instance = m_instanceHandle.instance;
            instance.numParticles = m_indexCount;
            if (m_indexCount > 0) FlexUtils.FastCopy(ref m_indices[0], 0, instance.particleIndices, 0, sizeof(int) * m_indexCount);
            m_instanceHandle.instance = instance;
        }

        #endregion

        #region Private

        void CreateActor()
        {
            AddToContainer();
            AcquireAsset();
            CreateInstance();

            if(allParticles.Length < particle_num)
                System.Array.Resize(ref allParticles, particle_num);
            if(particlesInitPos.Length < particle_num)
                System.Array.Resize(ref particlesInitPos, particle_num);
        }

        void DestroyActor()
        {
            DestroyInstance();
            ReleaseAsset();
            RemoveFromContainer();
        }

        void OnBeforeRecreate()
        {
            if (onBeforeRecreate != null) onBeforeRecreate();
            DestroyInstance();
        }

        void OnAfterRecreate()
        {
            CreateInstance();
            if (onAfterRecreate != null) onAfterRecreate();
        }

        void OnBeforeDestroy()
        {
            DestroyActor();
        }

        void MoveFixedParticles(FlexContainer.ParticleData _particleData)
        {
            if (Application.isPlaying)
            {
                if (m_currentAsset && m_currentAsset.fixedParticles.Length > 0 && m_instanceHandle)
                {
                    FlexExt.Instance instance = m_instanceHandle.instance;
                    Vector4[] particles = m_currentAsset.particles;
                    foreach (var index in m_currentAsset.fixedParticles)
                    {
                        if (index < particles.Length)
                        {
                            Vector4 particle = transform.TransformPoint(particles[index]); particle.w = 0.0f;
                            _particleData.SetParticle(indices[index], particle);
                        }
                    }
                }
            }
            else
            {
                if (m_currentAsset && m_instanceHandle)
                {
                    FlexExt.Instance instance = m_instanceHandle.instance;
                    Vector4[] particles = m_currentAsset.particles;
                    for (int i = 0; i < particles.Length; ++i)
                    {
                        Vector4 particle = transform.TransformPoint(particles[i]);
                        particle.w = _particleData.GetParticle(indices[i]).w;
                        _particleData.SetParticle(indices[i], particle);
                    }
                }
            }
        }

        void UpdateDrawParticles()
        {
            if (!m_drawParticles && m_drawParticlesHelper)
            {
                DestroyImmediate(m_drawParticlesHelper.gameObject);
                m_drawParticlesHelper = null;
            }
            if (m_drawParticles && !m_drawParticlesHelper)
            {
                GameObject drawParticlesObject = new GameObject("FlexDrawParticles");
                drawParticlesObject.hideFlags = HideFlags.DontSave | HideFlags.HideInHierarchy;
                drawParticlesObject.transform.parent = transform;
                drawParticlesObject.transform.localPosition = Vector3.zero;
                drawParticlesObject.transform.rotation = Quaternion.identity;
                m_drawParticlesHelper = drawParticlesObject.AddComponent<_auxFlexDrawParticles>();
            }
            if (m_drawParticlesHelper)
                m_drawParticlesHelper.UpdateMesh();
        }

        void UpdateBounds(FlexContainer.ParticleData _particleData)
        {
            Vector3 boundsMin = Vector3.one * float.MaxValue, boundsMax = Vector3.one * float.MaxValue;
            if (m_container != null && m_indices != null && m_indices.Length > 0)
                FlexUtils.ComputeBounds(_particleData.particleData.particles, ref m_indices[0], m_indices.Length, ref boundsMin, ref boundsMax);
            m_bounds.SetMinMax(boundsMin, boundsMax);
        }

        void TeleportParticles(FlexContainer.ParticleData _particleData)
        {
            Matrix4x4 offset = Matrix4x4.TRS(m_teleportPosition, Quaternion.identity, Vector3.one)
                                * Matrix4x4.TRS(Vector3.zero, m_teleportRotation, Vector3.one)
                                * Matrix4x4.TRS(Vector3.zero, Quaternion.Inverse(transform.rotation), Vector3.one)
                                * Matrix4x4.TRS(-transform.position, Quaternion.identity, Vector3.one);
            if (m_currentAsset && m_instanceHandle)
            {
                FlexExt.Instance instance = m_instanceHandle.instance;
                int[] indices = new int[instance.numParticles];
                FlexUtils.FastCopy(instance.particleIndices, indices);
                foreach (var index in indices)
                {
                    Vector4 particle = _particleData.GetParticle(index);
                    float storeW = particle.w;
                    particle = offset.MultiplyPoint3x4(particle);
                    particle.w = storeW;
                    _particleData.SetParticle(index, particle);
                    _particleData.SetVelocity(index, Vector3.zero);
                }
            }
            transform.position = m_teleportPosition;
            transform.rotation = m_teleportRotation;
            transform.hasChanged = false;
        }

       void ApplyImpulses(FlexContainer.ParticleData _particleData)
       {
            // Check if there is a current asset and instance handle
            if (m_currentAsset && m_instanceHandle)
            {
                // Get the instance and number of particles
                FlexExt.Instance instance = m_instanceHandle.instance;
                int[] indices = new int[instance.numParticles];
                
                // Copy the particle indices to a new array
                FlexUtils.FastCopy(instance.particleIndices, indices);
                
                // Iterate over each impulse in the list
                foreach (var info in m_impulses)
                {
                    // Skip impulses with negligible magnitude
                    if (info.impulse.sqrMagnitude < float.Epsilon) continue;

                    float mass = 0;
                    
                    // Calculate the total mass of the particles affected by this impulse
                    foreach (var index in indices)
                    {
                        if (info.particle == -1 || info.particle == index)
                        {
                            Vector4 particle = _particleData.GetParticle(index);
                            mass += 1.0f / particle.w;  // Sum the inverse of the particle's mass
                        }
                    }
                    
                    // Skip if the calculated mass is negligible
                    if (mass < float.Epsilon) continue;

                    // Calculate the change in velocity
                    Vector3 velocityChange = info.impulse / mass;
                    
                    // Apply the velocity change to each particle
                    foreach (var index in indices)
                    {
                        _particleData.SetVelocity(index, _particleData.GetVelocity(index) + velocityChange);
                    }
                }
            }
        }


        static int sm_nextGroup = 0;

        Vector3 m_teleportPosition = Vector3.zero;
        Quaternion m_teleportRotation = Quaternion.identity;
        int m_teleport = 0;
        struct ImpulseInfo { public Vector3 impulse; public int particle; }
        List<ImpulseInfo> m_impulses = new List<ImpulseInfo>();

        [NonSerialized]
        int[] m_indices = new int[0];
        [NonSerialized]
        int m_indexCount = 0;
        [NonSerialized]
        _auxFlexDrawParticles m_drawParticlesHelper;
        [NonSerialized]
        Bounds m_bounds = new Bounds();

        FlexExt.Instance.Handle m_instanceHandle;

        [SerializeField]
        FlexContainer m_container;
        [SerializeField]
        int m_particleGroup = -1;
        [SerializeField]
        bool m_selfCollide = true;
        [SerializeField]
        bool m_selfCollideFilter = false;
        [SerializeField]
        bool m_fluid = false;
        [SerializeField]
        float m_massScale = 1.0f;
        [SerializeField]
        bool m_drawParticles = false;

        [SerializeField]
        int m_recreateActor = 0; // 0 - nothing, 1 - re-create instance, 2 - re-add to container

        #endregion
    }
}
