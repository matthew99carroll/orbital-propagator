namespace Physics
{
    public class OrbitSpec
    {
        public double Apoapsis;
        public double Periapsis;
        public double InitialTime;
        public double InitialMeanLongitude;

        // 0 - 360
        public double Inclination;

        // 0 - 360
        public double LongitudeAscendingNode;

        // 0 - 360
        public double ArgumentOfPeriapsis;

        public OrbitSpec(double apoapsis, double periapsis, double inclination, double longitudeOfAscendingNode,
            double argumentOfPeriapsis, double initialTime, double initialMeanLongitude)
        {
            Apoapsis = apoapsis;
            Periapsis = periapsis;
            Inclination = inclination;
            LongitudeAscendingNode = longitudeOfAscendingNode;
            ArgumentOfPeriapsis = argumentOfPeriapsis;
            InitialTime = initialTime;
            InitialMeanLongitude = initialMeanLongitude;
        }
    }
}