using UnityEditor;
using static System.Math;

namespace Physics.OrbitManeuvers
{
    public class NonHohmannTransferCommonApseLine
    {
        public static double _angularMomentumOrbitOne;
        public static double _radiusPointA;
        public static double _velocityTransverseOrbitOnePointA, _velocityRadialOrbitOnePointA;
        public static double _velocityOrbitOnePointA;
        public static double _flightPathAngleOrbitOne;

        public static double _eccentricityOrbitTwo;
        public static double _angularMomentumOrbitTwo;
        public static double _velocityTransverseOrbitTwoPointA, _velocityRadialOrbitTwoPointA;
        public static double _velocityOrbitTwoPointA;
        public static double _flightPathAngleOrbitTwo;

        public static double _deltaFlightPathAngle;
        public static double _deltaVPointA;

        public static double _deltaVPointAOrientation;

        private static double _propellantMassExpended;
        
        public static ManeuverRequirements CalculateNonHohmannTransferCommonApseLine(
            KeplerianOrbitalElements keplerianOrbitalElements,
            OrbitSpec orbitSpec,
            double trueAnomalyA,
            double finalTrueAnomaly,
            double finalPeriapsis,
            double finalApoapsis,
            double orbitalBodyMass,
            double propellantIsp)
        {
            _angularMomentumOrbitOne = KeplerianSolver.CalculateAngularMomentum(orbitSpec);
            
            _radiusPointA = (Pow(_angularMomentumOrbitOne, 2) / Constants.GRAVITATIONAL_PARAMETER) /
                            (1 + keplerianOrbitalElements.Eccentricity * Cos(trueAnomalyA));

            _velocityTransverseOrbitOnePointA = _angularMomentumOrbitOne / _radiusPointA;
            _velocityRadialOrbitOnePointA = (Constants.GRAVITATIONAL_PARAMETER / _angularMomentumOrbitOne) *
                                            keplerianOrbitalElements.Eccentricity * Sin(trueAnomalyA);

            _velocityOrbitOnePointA =
                Sqrt(Pow(_velocityTransverseOrbitOnePointA, 2) + Pow(_velocityRadialOrbitOnePointA, 2));

            _flightPathAngleOrbitOne = Atan(_velocityRadialOrbitOnePointA / _velocityTransverseOrbitOnePointA);

            _eccentricityOrbitTwo = (finalPeriapsis - _radiusPointA) /
                                    (finalPeriapsis * Cos(finalTrueAnomaly) - _radiusPointA * Cos(trueAnomalyA));

            keplerianOrbitalElements.Eccentricity = _eccentricityOrbitTwo;

            _angularMomentumOrbitTwo = Sqrt(Constants.GRAVITATIONAL_PARAMETER * _radiusPointA * finalPeriapsis) *
                                       Sqrt((Cos(finalTrueAnomaly) - Cos(trueAnomalyA)) / (finalPeriapsis * Cos(finalTrueAnomaly) - _radiusPointA * Cos(trueAnomalyA)));

            _velocityTransverseOrbitTwoPointA = _angularMomentumOrbitTwo / _radiusPointA;
            _velocityRadialOrbitTwoPointA = _velocityRadialOrbitOnePointA =
                (Constants.GRAVITATIONAL_PARAMETER / _angularMomentumOrbitTwo) * _eccentricityOrbitTwo * Sin(trueAnomalyA);
            
            _velocityOrbitTwoPointA = Sqrt(Pow(_velocityTransverseOrbitTwoPointA, 2) + Pow(_velocityRadialOrbitTwoPointA, 2));
            
            _flightPathAngleOrbitTwo = Atan(_velocityRadialOrbitTwoPointA / _velocityTransverseOrbitTwoPointA);

            _deltaFlightPathAngle = _flightPathAngleOrbitTwo - _flightPathAngleOrbitOne;

            _deltaVPointA = Sqrt(Pow(_velocityOrbitOnePointA, 2) + Pow(_velocityOrbitTwoPointA, 2) -
                                 2 * _velocityOrbitOnePointA * _velocityOrbitTwoPointA * Cos(_deltaFlightPathAngle));
            
            _propellantMassExpended = (1 - Exp(-_deltaVPointA / (propellantIsp * Constants.EARTH_GRAVITY_0))) *
                                      orbitalBodyMass;
            
            return new ManeuverRequirements(_deltaVPointA, _propellantMassExpended);
        }
    }
}