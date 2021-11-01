namespace Physics.OrbitManeuvers
{
    public class ManeuverRequirements
    {
        public double RequiredDeltaV;
        public double ExpendedPropellantMass;
        public double TimeOfFlight;
        public double _localHorizonAngle;

        public ManeuverRequirements(double requiredDeltaV, double expendedPropellantMass, double timeOfFlight)
        {
            RequiredDeltaV = requiredDeltaV;
            ExpendedPropellantMass = expendedPropellantMass;
            TimeOfFlight = timeOfFlight;
        }

        public ManeuverRequirements(double requiredDeltaV, double expendedPropellantMass)
        {
            RequiredDeltaV = requiredDeltaV;
            ExpendedPropellantMass = expendedPropellantMass;
        }
    }
    
    public class ApseRotationRequirements
    {
        public double RequiredDeltaV;
        public double ExpendedPropellantMass;
        public double LocalHorizonAngle;

        public ApseRotationRequirements(double requiredDeltaV, double expendedPropellantMass, double localHorizonAngle)
        {
            RequiredDeltaV = requiredDeltaV;
            ExpendedPropellantMass = expendedPropellantMass;
            LocalHorizonAngle = localHorizonAngle;
        }
    }
}