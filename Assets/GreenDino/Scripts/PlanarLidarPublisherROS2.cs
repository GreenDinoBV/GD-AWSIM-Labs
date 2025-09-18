using System;
using UnityEngine;
using ROS2;
using sensor_msgs.msg;
using std_msgs.msg;

namespace AWSIM
{
    [RequireComponent(typeof(PlanarLidarSensor))]
    public class PlanarLidarPublisher : MonoBehaviour
    {
        public string Topic = "/scan";
        public string FrameID = "laser";
        public QoSSettings QosSettings = new QoSSettings()
        {
            ReliabilityPolicy = ReliabilityPolicy.QOS_POLICY_RELIABILITY_BEST_EFFORT,
            DurabilityPolicy = DurabilityPolicy.QOS_POLICY_DURABILITY_VOLATILE,
            HistoryPolicy = HistoryPolicy.QOS_POLICY_HISTORY_KEEP_LAST,
            Depth = 1,
        };

        private IPublisher<LaserScan> publisher;
        private LaserScan msg;
        private PlanarLidarSensor sensor;

        void Awake()
        {
            CreatePublisher();
        }

        void Start()
        {
            sensor = GetComponent<PlanarLidarSensor>();
            sensor.OnOutputData += Publish;
        }

        void CreatePublisher()
        {
            msg = new LaserScan()
            {
                Header = new Header() { Frame_id = FrameID },
            };

            var qos = QosSettings.GetQoSProfile();
            publisher = SimulatorROS2Node.CreatePublisher<LaserScan>(Topic, qos);
        }

        void Publish(PlanarLidarSensor.OutputData outputData)
        {
            msg.Angle_min = sensor.AngleMinRad;
            msg.Angle_max = sensor.AngleMaxRad;
            msg.Angle_increment = sensor.AngleIncrementRad;
            msg.Time_increment = 0.0f;
            msg.Scan_time = sensor.ScanTime;
            msg.Range_min = sensor.RangeMin;
            msg.Range_max = sensor.RangeMax;
            msg.Ranges = outputData.Ranges;
            msg.Intensities = new float[sensor.NumRays];

            var header = msg as MessageWithHeader;
            SimulatorROS2Node.UpdateROSTimestamp(ref header);

            publisher.Publish(msg);
        }

        void OnDestroy()
        {
            sensor.OnOutputData -= Publish;
            SimulatorROS2Node.RemovePublisher<LaserScan>(publisher);
            GC.Collect();
        }
    }
}
