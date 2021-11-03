using System.Data.SqlTypes;
using Math;
using Physics.OrbitDetermination;
using static System.Math;

namespace Physics.OrbitManeuvers
{
    public static class ChaseManeuver
    {

        private static double _angularMomentum;
        private static double _eccentricity;
        private static Types.Vector3D _radiusA, _radiusBPrime;
        private static Types.LambertsVelocityVectors _velocityTwo;

        private static double _deltaV;
        private static double _propellantMassExpended;
        
        /* Given two satellites in the exact same orbit, a chase maneuver can be used to catch this in a given time period
           The practical use of a chase maneuver is questionable, and is employed as a theoretical exercise
        */
        public static ManeuverRequirements CalculateChaseManeuver(OrbitSpec orbitSpec, 
            double trueAnomalyA, 
            double trueAnomalyB,
            double deltaT,
            double orbitalBodyMass,
            double propellantIsp)
        {
            _angularMomentum = KeplerianSolver.CalculateAngularMomentum(orbitSpec);
            _eccentricity = KeplerianSolver.CalculateEccentricity(orbitSpec);

            _radiusA = new Types.Vector3D(0, 0, 0);
            _radiusBPrime = new Types.Vector3D(0, 0, 0);
            
            _radiusA.X = (Pow(_angularMomentum, 2) / Constants.GRAVITATIONAL_PARAMETER) *
                       (1 / (1 + _eccentricity * Cos(trueAnomalyB))) * Cos(trueAnomalyB);
            
            _radiusA.Y = (Pow(_angularMomentum, 2) / Constants.GRAVITATIONAL_PARAMETER) *
                            (1 / (1 + _eccentricity * Cos(trueAnomalyB))) * Sin(trueAnomalyB);
            
            _radiusBPrime.X = (Pow(_angularMomentum, 2) / Constants.GRAVITATIONAL_PARAMETER) *
                                 (1 / (1 + _eccentricity * Cos(trueAnomalyA))) * Cos(trueAnomalyA);
            
            _radiusBPrime.Y = (Pow(_angularMomentum, 2) / Constants.GRAVITATIONAL_PARAMETER) *
                                 (1 / (1 + _eccentricity * Cos(trueAnomalyA))) * Sin(trueAnomalyA);

            Types.LambertsPositionVectors positions = new Types.LambertsPositionVectors(_radiusA, _radiusBPrime);

            _velocityTwo = Lambert.SolveLambertsProblem(positions, deltaT, Types.Direction.PROGRADE);

            _deltaV = Abs(Types.CalculateVectorNorm3D(_velocityTwo.V1)) +
                      Abs(Types.CalculateVectorNorm3D(_velocityTwo.V2));
            
            _propellantMassExpended = (1 - Exp(-_deltaV / (propellantIsp * Constants.EARTH_GRAVITY_0))) *
                                      orbitalBodyMass;
            
            return new ManeuverRequirements(_deltaV, _propellantMassExpended);

        }
    }
}