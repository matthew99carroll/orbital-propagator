using UnityEngine;

namespace Physics.OrbitManeuvers
{
    public static class BiellipticHohmannTransfer
    {
        private static double _radiusA, _radiusB, _radiusC, _radiusD;
        private static double _deltaVOrbitOnePointA;
        private static double _deltaVOrbitTwoPointA, _deltaVOrbitTwoPointB;
        private static double _deltaVOrbitThreePointB, _deltaVOrbitThreePointC;
        private static double _deltaVOrbitFourPointC, _deltaVOrbitFourPointD;
        private static double _angularMomentumOrbitTwo, _angularMomentumOrbitThree;
        private static double _deltaV;
        private static double _propellantMassExpended;
        private static double _semiMajorAxisOrbitTwo, _semiMajorAxisOrbitThree;
        private static double _timeOfFlight;

        public static ManeuverRequirements CalculateBiellipticHohmannTransfer(KeplerianOrbitalElements keplerianOrbitalElements, 
            OrbitSpec orbitSpec,
            double orbitOneApoapsis,
            double finalPeriapsis,
            double finalApoapsis,
            double orbitalBodyMass,
            double propellantIsp)
        {            
            _radiusA = orbitSpec.Periapsis + Constants.RADIUS_EARTH;
            _radiusB = orbitOneApoapsis + Constants.RADIUS_EARTH;
            _radiusC = finalPeriapsis + Constants.RADIUS_EARTH;
            _radiusD = finalApoapsis + Constants.RADIUS_EARTH;

            _deltaVOrbitOnePointA = System.Math.Sqrt(Constants.GRAVITATIONAL_PARAMETER / _radiusA);

            _angularMomentumOrbitTwo = System.Math.Sqrt(2 * Constants.GRAVITATIONAL_PARAMETER) * System.Math.Sqrt((_radiusA * _radiusB) / (_radiusA + _radiusB));

            _deltaVOrbitTwoPointA = _angularMomentumOrbitTwo / _radiusA;
            _deltaVOrbitTwoPointB = _angularMomentumOrbitTwo / _radiusB;

            _angularMomentumOrbitThree = System.Math.Sqrt(2 * Constants.GRAVITATIONAL_PARAMETER) * System.Math.Sqrt((_radiusC * _radiusB) / (_radiusC + _radiusB));

            _deltaVOrbitThreePointB = _angularMomentumOrbitThree / _radiusB;
            _deltaVOrbitThreePointC = _angularMomentumOrbitThree / _radiusC;

            _deltaVOrbitFourPointC = System.Math.Sqrt(Constants.GRAVITATIONAL_PARAMETER / _radiusC);
            _deltaVOrbitFourPointD = System.Math.Sqrt(Constants.GRAVITATIONAL_PARAMETER / _radiusD);

            _deltaV = System.Math.Abs(_deltaVOrbitTwoPointA - _deltaVOrbitOnePointA) + System.Math.Abs(_deltaVOrbitThreePointB - _deltaVOrbitTwoPointB) +
                System.Math.Abs(_deltaVOrbitFourPointC - _deltaVOrbitThreePointC);

            _propellantMassExpended = (1 - System.Math.Exp(-_deltaV / (propellantIsp * Constants.EARTH_GRAVITY_0))) *
                                         orbitalBodyMass;

            _semiMajorAxisOrbitTwo = (_radiusA + _radiusA) / 2.0;
            _semiMajorAxisOrbitThree = (_radiusB + _radiusC) / 2.0;

            _timeOfFlight = (1.0 / 2.0) * ((2 * System.Math.PI/(System.Math.Sqrt(Constants.GRAVITATIONAL_PARAMETER))) * System.Math.Pow(_semiMajorAxisOrbitTwo, 3/2) 
                            + (2 * System.Math.PI/(System.Math.Sqrt(Constants.GRAVITATIONAL_PARAMETER))) * System.Math.Pow(_semiMajorAxisOrbitThree, 3/2));

            orbitSpec.Apoapsis = _radiusD - Constants.RADIUS_EARTH;
            orbitSpec.Periapsis = _radiusC - Constants.RADIUS_EARTH;
            
            return new ManeuverRequirements(_deltaV, _propellantMassExpended, _timeOfFlight);
        }
    }
}