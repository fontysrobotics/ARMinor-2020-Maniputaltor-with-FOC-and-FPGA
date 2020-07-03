using UnityEngine;
using System.Collections.Generic;

namespace RosSharp.RosBridgeClient
{
    public class JointStateControlSubscriber : Subscriber<Messages.UnitySim.JointStateControl>
    {
        public List<string> JointNames;
        public List<JointStateControlWriter> JointStateWriters;

        protected override void ReceiveMessage(Messages.UnitySim.JointStateControl message)
        {
		Debug.Log("message received");
            int index;
            for (int i = 0; i < message.name.Length; i++)
            {
                index = JointNames.IndexOf(message.name[i]);
                if (index != -1) {
                  JointStateWriters[index].SetVelocity((float) message.velocity[i]);
                }
            }
        }
    }
}
