namespace Physics.OrbitManeuvers
{
    public class ManeuverRequirements
    {
        public double RequiredDeltaV;
        public double ExpendedPropellantMass;

        public ManeuverRequirements(double requiredDeltaV, double expendedPropellantMass)
        {
            RequiredDeltaV = requiredDeltaV;
            ExpendedPropellantMass = expendedPropellantMass;
        }
    }
}