using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using ROS2;

namespace AWSIM
{
    /// <summary>
    /// Convert the data output from PredictedPerceptionResultRos2Publisher to ROS2 msg and Publish.
    /// </summary>
    [RequireComponent(typeof(PredictedPerceptionResultSensor))]
    public class PredictedPerceptionResultRos2Publisher : MonoBehaviour
    {
        /// <summary>
        /// Topic name in DetectedObject msg.
        /// </summary>
        public string objectTopic = "/awsim/ground_truth/perception/object_recognition/detection/objects";

        /// <summary>
        /// Object sensor frame id.
        /// </summary>
        public string frameId = "base_link";

        /// <summary>
        /// max distance that lidar can detect
        /// </summary>
        [Range(0, 200)]
        public float maxDistance = 200f;

        /// <summary>
        /// QoS settings.
        /// </summary>
        public QoSSettings qosSettings = new QoSSettings()
        {
            ReliabilityPolicy = ReliabilityPolicy.QOS_POLICY_RELIABILITY_BEST_EFFORT,
            DurabilityPolicy = DurabilityPolicy.QOS_POLICY_DURABILITY_VOLATILE,
            HistoryPolicy = HistoryPolicy.QOS_POLICY_HISTORY_KEEP_LAST,
            Depth = 1,
        };

        IPublisher<autoware_perception_msgs.msg.PredictedObjects> objectPublisher;
        autoware_perception_msgs.msg.PredictedObjects objectsMsg;
        PredictedPerceptionResultSensor objectSensor;

        void Start()
        {
            // Get ObjectSensor component.
            objectSensor = GetComponent<PredictedPerceptionResultSensor>();
            // Set callback.
            objectSensor.OnOutputData += Publish;

            // Create msg.
            objectsMsg = new autoware_perception_msgs.msg.PredictedObjects();

            // Create publisher.
            var qos = qosSettings.GetQoSProfile();
            objectPublisher = SimulatorROS2Node.CreatePublisher<autoware_perception_msgs.msg.PredictedObjects>(objectTopic, qos);
        }

        void Publish(PredictedPerceptionResultSensor.OutputData outputData)
        {
            if (outputData == null || outputData.objects == null || outputData.origin == null) return;
            var objectsList = new List<autoware_perception_msgs.msg.PredictedObject>();
            foreach (var detectedObject in outputData.objects)
            {
                if (detectedObject == null || detectedObject.rigidBody == null || detectedObject.dimension == null || detectedObject.bounds == null) continue;
                var rb = detectedObject.rigidBody;
                var dim = detectedObject.dimension;
                var bou = detectedObject.bounds;

                float distance = Vector3.Distance(outputData.origin.position, rb.transform.position);
                if (distance > maxDistance) continue;

                var obj = new autoware_perception_msgs.msg.PredictedObject();

                var intBytes = BitConverter.GetBytes(detectedObject.id); 
                for (int i = 0; i < 4; i++)
                {
                    obj.Object_id.Uuid[i] = intBytes[i];
                }
                for (int i = 4; i < 16; i++)
                {
                    obj.Object_id.Uuid[i] = 0;
                }

                obj.Existence_probability = 1.0f;

                var classification = new autoware_perception_msgs.msg.ObjectClassification();
                switch (detectedObject.classification)
                {
                    case ObjectClassification.ObjectType.UNKNOWN:
                        classification.Label = autoware_perception_msgs.msg.ObjectClassification.UNKNOWN;
                        break;
                    case ObjectClassification.ObjectType.CAR:
                        classification.Label = autoware_perception_msgs.msg.ObjectClassification.CAR;
                        break;
                    case ObjectClassification.ObjectType.TRUCK:
                        classification.Label = autoware_perception_msgs.msg.ObjectClassification.TRUCK;
                        break;
                    case ObjectClassification.ObjectType.BUS:
                        classification.Label = autoware_perception_msgs.msg.ObjectClassification.BUS;
                        break;
                    case ObjectClassification.ObjectType.TRAILER:
                        classification.Label = autoware_perception_msgs.msg.ObjectClassification.TRAILER;
                        break;
                    case ObjectClassification.ObjectType.MOTORCYCLE:
                        classification.Label = autoware_perception_msgs.msg.ObjectClassification.MOTORCYCLE;
                        break;
                    case ObjectClassification.ObjectType.BICYCLE:
                        classification.Label = autoware_perception_msgs.msg.ObjectClassification.BICYCLE;
                        break;
                    case ObjectClassification.ObjectType.PEDESTRIAN:
                        classification.Label = autoware_perception_msgs.msg.ObjectClassification.PEDESTRIAN;
                        break;
                    default:
                        Debug.LogWarning("Unknown classification type");
                        classification.Label = autoware_perception_msgs.msg.ObjectClassification.UNKNOWN;
                        break;
                }
                classification.Probability = 1.0f;
                obj.Classification = new[] { classification };

                var kinematics = new autoware_perception_msgs.msg.PredictedObjectKinematics();

                // Pose
                Vector3 relativePosition = rb.transform.position - outputData.origin.position;
                Vector3 transformedPosition = Quaternion.Inverse(outputData.origin.rotation) * relativePosition;
                var p = ROS2Utility.UnityToRosPosition(transformedPosition);
                kinematics.Initial_pose_with_covariance.Pose.Position.X = p.x;
                kinematics.Initial_pose_with_covariance.Pose.Position.Y = p.y;
                kinematics.Initial_pose_with_covariance.Pose.Position.Z = p.z;

                var r = ROS2Utility.UnityToRosRotation(Quaternion.Inverse(outputData.origin.rotation) * rb.transform.rotation);
                kinematics.Initial_pose_with_covariance.Pose.Orientation.X = r.x;
                kinematics.Initial_pose_with_covariance.Pose.Orientation.Y = r.y;
                kinematics.Initial_pose_with_covariance.Pose.Orientation.Z = r.z;
                kinematics.Initial_pose_with_covariance.Pose.Orientation.W = r.w;

                // Twist
                kinematics.Initial_twist_with_covariance.Twist.Linear.X = rb.velocity.magnitude;
                kinematics.Initial_twist_with_covariance.Twist.Linear.Y = 0.0;
                kinematics.Initial_twist_with_covariance.Twist.Linear.Z = 0.0;
                var a = ROS2Utility.UnityToRosPosition(rb.angularVelocity);
                kinematics.Initial_twist_with_covariance.Twist.Angular.X = 0.0;
                kinematics.Initial_twist_with_covariance.Twist.Angular.Y = 0.0;
                kinematics.Initial_twist_with_covariance.Twist.Angular.Z = a.z;

                // Covariance
                const int size = 6;
                for (int i = 0; i < size; i++)
                {
                    kinematics.Initial_pose_with_covariance.Covariance[i * size + i] = 1;
                    kinematics.Initial_twist_with_covariance.Covariance[i * size + i] = 1;
                }

                kinematics.Predicted_paths = new autoware_perception_msgs.msg.PredictedPath[0];

                obj.Kinematics = kinematics;

                // Shape & Footprint
                var shape = new autoware_perception_msgs.msg.Shape();
                shape.Type = autoware_perception_msgs.msg.Shape.BOUNDING_BOX;
                shape.Dimensions.X = dim.x;
                shape.Dimensions.Y = dim.y;
                shape.Dimensions.Z = dim.z;

                var footprints = new geometry_msgs.msg.Polygon();
                if (bou.Length > 0)
                {
                    footprints.Points = new[]
                    {
                        new geometry_msgs.msg.Point32() { X = bou[0].x, Y = bou[0].y, Z = 0 },
                        new geometry_msgs.msg.Point32() { X = bou[1].x, Y = bou[1].y, Z = 0 },
                        new geometry_msgs.msg.Point32() { X = bou[2].x, Y = bou[2].y, Z = 0 },
                        new geometry_msgs.msg.Point32() { X = bou[3].x, Y = bou[3].y, Z = 0 }
                    };
                }
                shape.Footprint = footprints;
                obj.Shape = shape;

                objectsList.Add(obj);
            }
            // Converts data output from ObjectSensor to ROS2 msg
            objectsMsg.Objects = objectsList.ToArray();
            // Update msg header.
            var header = objectsMsg as MessageWithHeader;
            SimulatorROS2Node.UpdateROSTimestamp(ref header);
            objectsMsg.Header.Frame_id = frameId;

            // Publish to ROS2.
            objectPublisher.Publish(objectsMsg);
        }

        void OnDestroy()
        {
            SimulatorROS2Node.RemovePublisher<autoware_perception_msgs.msg.PredictedObjects>(objectPublisher);
        }
    }
}
