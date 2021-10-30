using Math;

namespace Physics
{
    public static class BodyKinetics
    {

        public static Types.Vector3D CalculateGravitationalForce(Types.Vector3D position, float parentBodyMass,
            float orbitingBodyMass)
        {
            var forceX = (Constants.GRAVITATIONAL_CONSTANT * parentBodyMass * orbitingBodyMass) / position.X;
            var forceY = (Constants.GRAVITATIONAL_CONSTANT * parentBodyMass * orbitingBodyMass) / position.Y;
            var forceZ = (Constants.GRAVITATIONAL_CONSTANT * parentBodyMass * orbitingBodyMass) / position.Z;
            
            return new Types.Vector3D(forceX, forceY, forceZ);
        }

        public static Types.Vector3D CalculateGravitationalAcceleration(Types.Vector3D position, float parentBodyMass)
        {
            var accX = (Constants.GRAVITATIONAL_CONSTANT * parentBodyMass) / position.X;
            var accY = (Constants.GRAVITATIONAL_CONSTANT * parentBodyMass) / position.Y;
            var accZ = (Constants.GRAVITATIONAL_CONSTANT * parentBodyMass) / position.Z;
            
            return new Types.Vector3D(accX, accY, accZ);
        }
    }
}