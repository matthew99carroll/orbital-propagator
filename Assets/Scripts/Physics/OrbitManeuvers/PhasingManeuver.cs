using UnityEngine;

namespace Physics.OrbitManeuvers
{
    public static class PhasingManeuver
    {
        private static double _orbitOnePeriapsis, _orbitOneApoapsis;
        private static double _orbitTwoApoapsis;
        private static double _angularMomentumOrbitOne, _angularMomentumOrbitTwo;
        private static double _periodOrbitOne, _periodOrbitTwo;
        private static double _semiMajorAxisOrbitTwo;
        private static double _eccentricAnomalyB;
        private static double _timeOfFlightAToB;
        private static double _deltaVOrbitOnePointA, _deltaVOrbitTwoPointA;
        private static double _deltaVPhasingStart, _deltaVPhasingEnd;
        private static double _deltaV;
        private static double _propellantMassExpended;

        public static ManeuverRequirements CalculatePhasingManeuver(KeplerianOrbitalElements keplerianOrbitalElements, 
            OrbitSpec orbitSpec,
            double trueAnomalyAToB,
            double numberOfRevolutions,
            double orbitalBodyMass,
            double propellantIsp)
        {

            _orbitOnePeriapsis = orbitSpec.Periapsis + Constants.RADIUS_EARTH;
            _orbitOneApoapsis = orbitSpec.Apoapsis + Constants.RADIUS_EARTH;

            _angularMomentumOrbitOne = KeplerianSolver.CalculateAngularMomentum(orbitSpec);
            
            _periodOrbitOne = KeplerianSolver.CalculateOrbitalPeriod(keplerianOrbitalElements);

            _eccentricAnomalyB =
                2 * System.Math.Atan(
                    System.Math.Sqrt((1 - keplerianOrbitalElements.Eccentricity) /
                                     (1 + keplerianOrbitalElements.Eccentricity)) *
                    System.Math.Tan(trueAnomalyAToB / 2));

            _timeOfFlightAToB = _periodOrbitOne / (2 * System.Math.PI) *
                                (_eccentricAnomalyB - keplerianOrbitalElements.Eccentricity *
                                    System.Math.Sin(_eccentricAnomalyB));

            _periodOrbitTwo = -_periodOrbitOne - (_timeOfFlightAToB / numberOfRevolutions);

            _semiMajorAxisOrbitTwo =
                System.Math.Pow(
                    (System.Math.Sqrt(Constants.GRAVITATIONAL_PARAMETER) * _periodOrbitTwo / (2 * System.Math.PI)),
                    (2.0 / 3.0));

            _orbitTwoApoapsis = (2 * _semiMajorAxisOrbitTwo) - _orbitOnePeriapsis;

            orbitSpec.Periapsis = _orbitOnePeriapsis - Constants.RADIUS_EARTH;
            orbitSpec.Apoapsis = _orbitTwoApoapsis - Constants.RADIUS_EARTH;


            _angularMomentumOrbitTwo = KeplerianSolver.CalculateAngularMomentum(orbitSpec);

            _deltaVOrbitOnePointA = _angularMomentumOrbitOne / _orbitOnePeriapsis;
            _deltaVOrbitTwoPointA = _angularMomentumOrbitTwo / _orbitOnePeriapsis;

            _deltaVPhasingStart = _deltaVOrbitTwoPointA - _deltaVOrbitOnePointA;
            _deltaVPhasingEnd = _deltaVOrbitOnePointA - _deltaVOrbitTwoPointA;

            _deltaV = System.Math.Abs(_deltaVPhasingStart) + System.Math.Abs(_deltaVPhasingEnd);
            
            _propellantMassExpended = (1 - System.Math.Exp(-_deltaV / (propellantIsp * Constants.EARTH_GRAVITY_0))) *
                                      orbitalBodyMass;
            
            return new ManeuverRequirements(_deltaV, _propellantMassExpended, _timeOfFlightAToB);
        }
    }
}