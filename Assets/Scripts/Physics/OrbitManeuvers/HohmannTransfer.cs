using UnityEngine;
using static System.Math;

namespace Physics.OrbitManeuvers
{
    public static class HohmannTransfer
    {
        private static double _periapsisFinal, _apoapsisFinal;
        private static double _periapsisOrbitOne, _apoapsisOrbitOne;
        private static double _angularMomentumOrbitOne, _angularMomentumOrbitTwo;
        private static double _angularMomentumFinal;
        private static double _velocityOrbitOnePointA, _velocityOrbitTwoPointA;
        private static double _velocityOrbitTwoPointB, _velocityOrbitThreePointB;
        private static double _deltaVPointA, _deltaVPointB;
        private static double _deltaV;
        private static double _propellantMassExpended;
        private static double _finalSemiMajorAxis;
        private static double _timeOfFlight;
        public static ManeuverRequirements CalculateHohmannTransfer(KeplerianOrbitalElements keplerianOrbitalElements, 
            OrbitSpec orbitSpec,
            double finalPeriapsis,
            double finalApoapsis,
            double orbitalBodyMass,
            double propellantIsp)
        {
            _periapsisFinal = finalPeriapsis + Constants.RADIUS_EARTH;
            _apoapsisFinal = finalApoapsis + Constants.RADIUS_EARTH;

            _periapsisOrbitOne = orbitSpec.Periapsis + Constants.RADIUS_EARTH;
            _apoapsisOrbitOne = orbitSpec.Apoapsis + Constants.RADIUS_EARTH;
            _angularMomentumOrbitOne = KeplerianSolver.CalculateAngularMomentum(orbitSpec);

            _velocityOrbitOnePointA = _angularMomentumOrbitOne / _periapsisOrbitOne;
            
            orbitSpec.Periapsis = _periapsisOrbitOne - Constants.RADIUS_EARTH;
            orbitSpec.Apoapsis = _apoapsisFinal - Constants.RADIUS_EARTH;
            keplerianOrbitalElements.Eccentricity = KeplerianSolver.CalculateEccentricity(orbitSpec);
            
            _angularMomentumOrbitTwo = KeplerianSolver.CalculateAngularMomentum(orbitSpec);

            _velocityOrbitTwoPointA = _angularMomentumOrbitTwo / _periapsisOrbitOne;
            _deltaVPointA = _velocityOrbitTwoPointA - _velocityOrbitOnePointA;

            _velocityOrbitTwoPointB = _angularMomentumOrbitTwo / _apoapsisFinal;

            orbitSpec.Periapsis = _periapsisFinal - Constants.RADIUS_EARTH;
            orbitSpec.Apoapsis = _apoapsisFinal - Constants.RADIUS_EARTH;
            keplerianOrbitalElements.Eccentricity = KeplerianSolver.CalculateEccentricity(orbitSpec);

            _angularMomentumFinal = KeplerianSolver.CalculateAngularMomentum(orbitSpec);

            _velocityOrbitThreePointB = _angularMomentumFinal / _apoapsisFinal;

            _deltaVPointB = _velocityOrbitThreePointB - _velocityOrbitTwoPointB;

            _deltaV = Abs(_deltaVPointA) + Abs(_deltaVPointB);

            _propellantMassExpended = (1 - Exp(-_deltaV / (propellantIsp * Constants.EARTH_GRAVITY_0))) *
                                         orbitalBodyMass;

            _finalSemiMajorAxis = (_periapsisOrbitOne + _apoapsisFinal) / 2.0;

            _timeOfFlight = (1.0 / 2.0) * ((2 * PI/(Sqrt(Constants.GRAVITATIONAL_PARAMETER))) * Pow(_finalSemiMajorAxis, 3/2));
            
            return new ManeuverRequirements(_deltaV, _propellantMassExpended, _timeOfFlight);
        }
    }
}