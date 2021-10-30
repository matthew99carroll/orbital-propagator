namespace Physics
{
    public class KeplerianOrbitalElements
    {
        // 0 - x
        public double SemiMajorAxis;
    
        // 0 - 1
        public double Eccentricity;

        // 0 - 360
        public double Inclination;
    
        // 0 - 360
        public double LongitudeAscendingNode;
    
        // 0 - 360
        public double ArgumentOfPeriapsis;

        public KeplerianOrbitalElements(double semiMajorAxis, double eccentricity, double inclination, double longitudeAscendingNode, double argumentOfPeriapsis)
        {
            SemiMajorAxis = semiMajorAxis;
            Eccentricity = eccentricity;
            Inclination = inclination;
            LongitudeAscendingNode = longitudeAscendingNode;
            ArgumentOfPeriapsis = argumentOfPeriapsis;
        }
    }
}