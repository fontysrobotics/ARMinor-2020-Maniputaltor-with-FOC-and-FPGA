using RosSharp.Urdf;
using UnityEngine;
using Joint = UnityEngine.Joint;

namespace RosSharp.RosBridgeClient
{
    [RequireComponent(typeof(Joint)), RequireComponent(typeof(UrdfJointRevolute))]
    public class JointStateControlWriter : MonoBehaviour
    {
        private UrdfJointRevolute urdfJoint;

        private float velocity;
        // private float prevState; // rad or m
        private bool isNewStateReceived;

        private void Start()
        {
            urdfJoint = GetComponent<UrdfJointRevolute>();
        }

        private void Update()
        {
            if (isNewStateReceived)
            {
                WriteUpdate();
                isNewStateReceived = false;
            }
        }
        private void WriteUpdate()
        {
            urdfJoint.UpdateJointVelocity(velocity);
        }

        public void SetVelocity(float newVelocity)
        {
            velocity = newVelocity;
            isNewStateReceived = true;
        }


    }
}
