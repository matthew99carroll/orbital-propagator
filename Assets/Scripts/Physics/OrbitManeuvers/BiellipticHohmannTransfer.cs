using UnityEngine;
using static System.Math;

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

            _deltaVOrbitOnePointA = Sqrt(Constants.GRAVITATIONAL_PARAMETER / _radiusA);

            _angularMomentumOrbitTwo = Sqrt(2 * Constants.GRAVITATIONAL_PARAMETER) * Sqrt((_radiusA * _radiusB) / (_radiusA + _radiusB));

            _deltaVOrbitTwoPointA = _angularMomentumOrbitTwo / _radiusA;
            _deltaVOrbitTwoPointB = _angularMomentumOrbitTwo / _radiusB;

            _angularMomentumOrbitThree = Sqrt(2 * Constants.GRAVITATIONAL_PARAMETER) * Sqrt((_radiusC * _radiusB) / (_radiusC + _radiusB));

            _deltaVOrbitThreePointB = _angularMomentumOrbitThree / _radiusB;
            _deltaVOrbitThreePointC = _angularMomentumOrbitThree / _radiusC;

            _deltaVOrbitFourPointC = Sqrt(Constants.GRAVITATIONAL_PARAMETER / _radiusC);
            _deltaVOrbitFourPointD = Sqrt(Constants.GRAVITATIONAL_PARAMETER / _radiusD);

            _deltaV = Abs(_deltaVOrbitTwoPointA - _deltaVOrbitOnePointA) + Abs(_deltaVOrbitThreePointB - _deltaVOrbitTwoPointB) +
                Abs(_deltaVOrbitFourPointC - _deltaVOrbitThreePointC);

            _propellantMassExpended = (1 - Exp(-_deltaV / (propellantIsp * Constants.EARTH_GRAVITY_0))) *
                                         orbitalBodyMass;

            _semiMajorAxisOrbitTwo = (_radiusA + _radiusA) / 2.0;
            _semiMajorAxisOrbitThree = (_radiusB + _radiusC) / 2.0;

            _timeOfFlight = (1.0 / 2.0) * ((2 * PI/(Sqrt(Constants.GRAVITATIONAL_PARAMETER))) * Pow(_semiMajorAxisOrbitTwo, 3/2) 
                            + (2 * PI/(Sqrt(Constants.GRAVITATIONAL_PARAMETER))) * Pow(_semiMajorAxisOrbitThree, 3/2));

            orbitSpec.Apoapsis = _radiusD - Constants.RADIUS_EARTH;
            orbitSpec.Periapsis = _radiusC - Constants.RADIUS_EARTH;
            
            return new ManeuverRequirements(_deltaV, _propellantMassExpended, _timeOfFlight);
        }
    }
}