/*
This message class is generated automatically with 'SimpleMessageGenerator' of ROS#
*/

using Newtonsoft.Json;
using RosSharp.RosBridgeClient.Messages.Geometry;
using RosSharp.RosBridgeClient.Messages.Navigation;
using RosSharp.RosBridgeClient.Messages.Sensor;
using RosSharp.RosBridgeClient.Messages.Standard;
using RosSharp.RosBridgeClient.Messages.Actionlib;

namespace RosSharp.RosBridgeClient.Messages.UnitySim
{
  public class JointStateControl : Message
  {
    [JsonIgnore]
    public const string RosMessageName = "unity_msgs/speed_control";

    public Header header;
    public string[] name;
    public float[] velocity;
    public float[] effort;

    public JointStateControl()
    {
      header = new Header();
      name = new string[0];
      velocity = new float[0];
      effort = new float[0];
    }
  }
}
