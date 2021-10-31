namespace Physics.OrbitManeuvers
{
    public class ManeuverRequirements
    {
        public double RequiredDeltaV;
        public double ExpendedPropellantMass;
        public double TimeOfFlight;

        public ManeuverRequirements(double requiredDeltaV, double expendedPropellantMass, double timeOfFlight)
        {
            RequiredDeltaV = requiredDeltaV;
            ExpendedPropellantMass = expendedPropellantMass;
            TimeOfFlight = timeOfFlight;
        }
    }
}