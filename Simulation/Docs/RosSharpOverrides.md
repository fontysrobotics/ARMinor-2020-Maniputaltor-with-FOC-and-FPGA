RosSharp Asset override list
======

| File | Changes |
|------|---------|
| __Joint overrides__ | base joint management |
| `UrdfJoint` | Allowed the `Start()` function to be overridden |
| `UrdfJointRevolute` | - Added an encoder type system to track the angle of the joint as the build-in angle reporting is not always correct. <br /> - Added methods for motor control (velocity and effort control) |
| __Ros bridging__ | Overrides in message handling |
| `JointStateReader` | - Added calibration system so the zero angle can be changed / properly set. <br /> - Added normalization so values are between -180 and 180 degrees |
