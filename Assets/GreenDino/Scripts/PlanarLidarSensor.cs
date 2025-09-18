using System;
using UnityEngine;

namespace AWSIM
{
    public class PlanarLidarSensor : MonoBehaviour
    {
        public float angleMinDeg = -135f;
        public float angleMaxDeg = 135f;
        public float angleIncrementDeg = 0.5f;
        public float rangeMin = 0.05f;
        public float rangeMax = 12.0f;
        public float scanFrequency = 10.0f; // Hz

        public event Action<OutputData> OnOutputData;

        public struct OutputData
        {
            public float[] Ranges;
        }

        private float scanTimer;
        private int numRays;
        private float[] ranges;

        void Start()
        {
            numRays = Mathf.RoundToInt((angleMaxDeg - angleMinDeg) / angleIncrementDeg) + 1;
            ranges = new float[numRays];
        }

        void FixedUpdate()
        {
            scanTimer += Time.fixedDeltaTime;
            float scanPeriod = 1.0f / scanFrequency;

            if (scanTimer >= scanPeriod)
            {
                scanTimer -= scanPeriod;
                CaptureScan();
            }
        }

        void CaptureScan()
        {
            for (int i = 0; i < numRays; i++)
            {
                float angle = angleMinDeg + i * angleIncrementDeg;
                Vector3 dir = Quaternion.Euler(0, angle, 0) * transform.right;
                Debug.DrawRay(transform.position, dir * rangeMax, Color.red, 0.1f);

                if (Physics.Raycast(transform.position, dir, out RaycastHit hit, rangeMax))
                    ranges[i] = (hit.distance >= rangeMin) ? hit.distance : float.PositiveInfinity;
                else
                    ranges[i] = float.PositiveInfinity;
            }

            OnOutputData?.Invoke(new OutputData { Ranges = ranges });
        }

        public int NumRays => numRays;
        public float AngleMinRad => Mathf.Deg2Rad * angleMinDeg;
        public float AngleMaxRad => Mathf.Deg2Rad * angleMaxDeg;
        public float AngleIncrementRad => Mathf.Deg2Rad * angleIncrementDeg;
        public float RangeMin => rangeMin;
        public float RangeMax => rangeMax;
        public float ScanTime => 1.0f / scanFrequency;
    }
}
