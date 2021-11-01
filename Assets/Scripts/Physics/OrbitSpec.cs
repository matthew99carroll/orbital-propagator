namespace Physics
{
    public class OrbitSpec
    {
        public double Apoapsis;
        public double Periapsis;
        public double InitialTime;
        public double InitialTrueAnomaly;

        // 0 - 360
        public double Inclination;

        // 0 - 360
        public double LongitudeAscendingNode;

        // 0 - 360
        public double ArgumentOfPeriapsis;

        public OrbitSpec(double apoapsis, double periapsis, double inclination, double longitudeOfAscendingNode,
            double argumentOfPeriapsis, double initialTime, double initialTrueAnomaly)
        {
            Apoapsis = apoapsis;
            Periapsis = periapsis;
            Inclination = inclination;
            LongitudeAscendingNode = longitudeOfAscendingNode;
            ArgumentOfPeriapsis = argumentOfPeriapsis;
            InitialTime = initialTime;
            InitialTrueAnomaly = initialTrueAnomaly;
        }
    }
}