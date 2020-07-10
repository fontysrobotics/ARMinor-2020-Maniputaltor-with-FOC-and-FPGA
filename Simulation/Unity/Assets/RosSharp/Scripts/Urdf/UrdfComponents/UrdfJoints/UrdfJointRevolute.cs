/*
© Siemens AG, 2018-2019
Author: Suzannah Smith (suzannah.smith@siemens.com)

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at
<http://www.apache.org/licenses/LICENSE-2.0>.
Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*/

using System;
using UnityEngine;

namespace RosSharp.Urdf
{
    public class UrdfJointRevolute : UrdfJoint
    {
        private bool calibrated;
        private float PrevPos;
        private float ActualPos;

        public override JointTypes JointType => JointTypes.Revolute;

        public static UrdfJoint Create(GameObject linkObject)
        {
            UrdfJointRevolute urdfJoint = linkObject.AddComponent<UrdfJointRevolute>();

            urdfJoint.UnityJoint = linkObject.AddComponent<HingeJoint>();
            urdfJoint.UnityJoint.autoConfigureConnectedAnchor = true;
            ((HingeJoint)urdfJoint.UnityJoint).useLimits = true;
            linkObject.AddComponent<HingeJointLimitsManager>();

            return urdfJoint;
        }

        #region Runtime

        public override void Start()
        {
          base.Start();
          Update();
        }

        // Encoder for the angle of the joint
        public void Update()
        {
          // current position calculation
          float CurPos = 0;
          // get the current position in the correct angle according the the joints axis
          //TODO figure out if this can be made to work with non axis alligned joints
          if (((HingeJoint)UnityJoint).axis.x != 0) {
            CurPos = ((HingeJoint)UnityJoint).transform.localRotation.eulerAngles.x * Mathf.Deg2Rad * -((HingeJoint)UnityJoint).axis.x;
          } else if(((HingeJoint)UnityJoint).axis.y != 0) {
            CurPos = ((HingeJoint)UnityJoint).transform.localRotation.eulerAngles.y * Mathf.Deg2Rad * -((HingeJoint)UnityJoint).axis.y;
          } else if(((HingeJoint)UnityJoint).axis.z != 0) {
            CurPos = ((HingeJoint)UnityJoint).transform.localRotation.eulerAngles.z * Mathf.Deg2Rad * -((HingeJoint)UnityJoint).axis.z;
          }

          // calibration / first time setting up current position
          // this part makes sure the angle is set correctly at start instead of zeroing at its starting location (that should be handled by a user settable calibration system to callibrate the reporting to other systems)
          if (!this.calibrated) {
            this.ActualPos = this.PrevPos = CurPos;
            this.calibrated = true;
            return;
          }

          // calculate the difference between updates
          float NewPos = CurPos - this.PrevPos;

          // get direction the movement should be in
          int direction = 0;
          if (((HingeJoint)UnityJoint).velocity < 0) {
            direction = 1;
          } else if (((HingeJoint)UnityJoint).velocity > 0) {
            direction = -1;
          }

          // if we pass over 0/360 degrees with at least a degree of difference to combat jitter
          // update the difference to be relative instead of taling the wrong direction and around
          if (direction == 1 && NewPos < -0.0174533) {
            NewPos = Mathf.Abs(NewPos + 2*Mathf.PI);
          } else if (direction == -1 && NewPos > 0.0174533) {
            NewPos = Mathf.Abs(NewPos - 2*Mathf.PI);
          }

          // add the difference to the current actual position
          this.ActualPos += Mathf.Abs(NewPos)*direction;

          // normalize the axis to be between 0 and 360 degrees
          while (this.ActualPos > (2*Mathf.PI)) {
            this.ActualPos -= 2*Mathf.PI;
          }
          while (this.ActualPos < 0) {
            this.ActualPos += 2*Mathf.PI;
          }

          // update previous position for the next update
          this.PrevPos = CurPos;
        }

        public override float GetPosition()
        {
            // Using the transformations localRotatation does not correctly work!
            // according to the docs it should report the angle relative to its parrent but it only reports based on an axis, so after 90 degrees from this axis it goes backwords, instead of continuing
            // This implementation also uses the axis as a multiplier for the angle (not perfect but does take into account the rotation direction)
            // if (((HingeJoint)UnityJoint).axis.x != 0) {
            //   return ((HingeJoint)UnityJoint).transform.localRotation.eulerAngles.x * Mathf.Deg2Rad * -((HingeJoint)UnityJoint).axis.x;
            // } else if(((HingeJoint)UnityJoint).axis.y != 0) {
            //   return ((HingeJoint)UnityJoint).transform.localRotation.eulerAngles.y * Mathf.Deg2Rad * -((HingeJoint)UnityJoint).axis.y;
            // } else if(((HingeJoint)UnityJoint).axis.z != 0) {
            //   return ((HingeJoint)UnityJoint).transform.localRotation.eulerAngles.z * Mathf.Deg2Rad * -((HingeJoint)UnityJoint).axis.z;
            // }
            // return 0;

            // Old implementation that works, but only as long as no parameters are changed in the Unity Editor, otherwise the angle will reset to 0
            // This value is resets to 0 whenever the hinge is reloaded (on startup or Unity Editor parameter changes!)
            // Upon further ispection, this method apperently has the same issues as the first part, but in a different axis / situation
            // return -((HingeJoint)UnityJoint).angle * Mathf.Deg2Rad;

            // time to redo all of it as its fully broken now, time to build our own encoder
            // as observed one of the 2 methods is correct but we do not know wich, also when moving multiple joints, this may become challenging
            // For now we build our own encoder using these values, they seem to be correct in terms of their difference, but not in their end value so we just use them as encoder differences and use the velocity to determine direction
            // get the position (encoded end value)
            return this.ActualPos;
        }

        public override float GetVelocity()
        {
            return ((HingeJoint)UnityJoint).velocity * Mathf.Deg2Rad;
        }

        public override float GetEffort()
        {
            return ((HingeJoint)UnityJoint).motor.force;
        }

        protected override void OnUpdateJointState(float deltaState)
        {
            Vector3 lea = ((HingeJoint)UnityJoint).transform.localEulerAngles;

            if (((HingeJoint)UnityJoint).axis.x != 0) {
              lea.x = deltaState * Mathf.Rad2Deg;
            } else if(((HingeJoint)UnityJoint).axis.y != 0) {
              lea.y = deltaState * Mathf.Rad2Deg;
            } else if(((HingeJoint)UnityJoint).axis.z != 0) {
              lea.z = deltaState * Mathf.Rad2Deg;
            }

            ((HingeJoint)UnityJoint).transform.localEulerAngles = lea;
            // Quaternion rot = Quaternion.AngleAxis(deltaState * Mathf.Rad2Deg, UnityJoint.axis);
            // transform.rotation = transform.rotation * rot;
        }

        #endregion

        protected override void ImportJointData(Joint joint)
        {
            UnityJoint.axis = (joint.axis != null) ? GetAxis(joint.axis) : GetDefaultAxis();

            if (joint.dynamics != null)
                ((HingeJoint)UnityJoint).spring = GetJointSpring(joint.dynamics);

            if (joint.limit != null)
                GetComponent<HingeJointLimitsManager>().InitializeLimits(joint.limit, (HingeJoint)UnityJoint);
        }

        protected override Joint ExportSpecificJointData(Joint joint)
        {
            joint.axis = GetAxisData(UnityJoint.axis);
            joint.dynamics = new Joint.Dynamics(((HingeJoint)UnityJoint).spring.damper, ((HingeJoint)UnityJoint).spring.spring);

            joint.limit = ExportLimitData();

            return joint;
        }

        public override bool AreLimitsCorrect()
        {
            HingeJointLimitsManager limits = GetComponent<HingeJointLimitsManager>();
            return limits != null && limits.LargeAngleLimitMin < limits.LargeAngleLimitMax;
        }

        protected override Joint.Limit ExportLimitData()
        {
            HingeJointLimitsManager hingeJointLimits = GetComponent<HingeJointLimitsManager>();
            return new Joint.Limit(
                Math.Round(hingeJointLimits.LargeAngleLimitMin * Mathf.Deg2Rad, RoundDigits),
                Math.Round(hingeJointLimits.LargeAngleLimitMax * Mathf.Deg2Rad, RoundDigits),
                EffortLimit,
                VelocityLimit);
        }

        public void UpdateJointVelocity(float newVelocity)
        {
          JointMotor motor = ((HingeJoint)UnityJoint).motor;

          motor.targetVelocity = newVelocity;
          ((HingeJoint)UnityJoint).motor = motor;
        }

        public void UpdateJointEffort(float newEffort)
        {
          JointMotor motor = ((HingeJoint)UnityJoint).motor;

          motor.force = newEffort;
          ((HingeJoint)UnityJoint).motor = motor;
        }

        public void Recallibrate() {
          this.calibrated = false;
        }
    }
}
