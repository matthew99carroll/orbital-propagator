using UnityEditor;
using static System.Math;

namespace Physics.OrbitManeuvers
{
    public static class RotateApseLine
    {
        private static double _eccentricityOrbitOne, _eccentricityOrbitTwo;
        private static double _angularMomentumOrbitOne, _angularMomentumOrbitTwo;
        private static double _apseLineRotation;
        private static double _a, _b, _c;
        private static double _phi;
        private static double _trueAnomalyOrbitIntersectionRootA, _trueAnomalyOrbitIntersectionRootB;
        private static double _radiusOrbitIntersectionA, _radiusOrbitIntersectionB;
        private static double _velocityTransverseIntersectionAOrbitOne, _velocityRadialIntersectionAOrbitOne;
        private static double _flightPathAngleIntersectionAOrbitOne;
        private static double _velocityOrbitOne;
        private static double _velocityTransverseIntersectionAOrbitTwo, _velocityRadialIntersectionAOrbitTwo;
        private static double _flightPathAngleIntersectionAOrbitTwo;
        private static double _velocityOrbitTwo;
        private static double _deltaV;
        private static double _propellantMassExpended;
        
        public static ManeuverRequirements CalculateApseLineRotation(KeplerianOrbitalElements keplerianOrbitalElements,
            OrbitSpec orbitSpec,
            double finalPeriapsis,
            double finalApoapsis,
            double trueAnomalyA,
            double trueAnomalyB,
            double orbitalBodyMass,
            double propellantIsp)
        {
            _eccentricityOrbitOne = keplerianOrbitalElements.Eccentricity;
            
            _angularMomentumOrbitOne = KeplerianSolver.CalculateAngularMomentum(orbitSpec, _eccentricityOrbitOne);

            orbitSpec.Periapsis = finalPeriapsis;
            orbitSpec.Apoapsis = finalApoapsis;

            _eccentricityOrbitTwo = KeplerianSolver.CalculateEccentricity(orbitSpec);
            _angularMomentumOrbitTwo = KeplerianSolver.CalculateAngularMomentum(orbitSpec, _eccentricityOrbitTwo);

            _apseLineRotation = trueAnomalyB - trueAnomalyA;


            _a = (_eccentricityOrbitOne * Pow(_angularMomentumOrbitTwo, 2)) -
                 (_eccentricityOrbitTwo * Pow(_angularMomentumOrbitOne, 2) * Cos(_apseLineRotation));

            _b = -_eccentricityOrbitTwo * Pow(_eccentricityOrbitOne, 2) * Sin(_apseLineRotation);

            _c = Pow(_angularMomentumOrbitOne, 2) - Pow(_angularMomentumOrbitTwo, 2);

            _phi = Atan(_b / _a);
            
            _trueAnomalyOrbitIntersectionRootA = _phi + Acos((_c / _a) * Cos(_phi));
            _trueAnomalyOrbitIntersectionRootB = _phi - Acos((_c / _a) * Cos(_phi));

            _radiusOrbitIntersectionA = Pow(_angularMomentumOrbitOne, 2) / (Constants.GRAVITATIONAL_PARAMETER) *
                                       (1.0 / (1 + _eccentricityOrbitOne * Cos(_trueAnomalyOrbitIntersectionRootA)));
            _radiusOrbitIntersectionB = Pow(_angularMomentumOrbitOne, 2) / (Constants.GRAVITATIONAL_PARAMETER) *
                                       (1.0 / (1 + _eccentricityOrbitOne * Cos(_trueAnomalyOrbitIntersectionRootB)));

            _velocityTransverseIntersectionAOrbitOne = _angularMomentumOrbitOne / _radiusOrbitIntersectionA;
            _velocityRadialIntersectionAOrbitOne = (Constants.GRAVITATIONAL_PARAMETER / _angularMomentumOrbitOne) *
                                                   _eccentricityOrbitOne * Sin(_trueAnomalyOrbitIntersectionRootA);
            _flightPathAngleIntersectionAOrbitOne =
                Atan(_velocityRadialIntersectionAOrbitOne / _velocityTransverseIntersectionAOrbitOne);
            _velocityOrbitOne = Sqrt(Pow(_velocityRadialIntersectionAOrbitOne, 2) +
                                     Pow(_velocityTransverseIntersectionAOrbitOne, 2));
            
            _velocityTransverseIntersectionAOrbitTwo = _angularMomentumOrbitTwo / _radiusOrbitIntersectionA;
            _velocityRadialIntersectionAOrbitTwo = (Constants.GRAVITATIONAL_PARAMETER / _angularMomentumOrbitTwo) *
                                                   _eccentricityOrbitTwo * Sin(_trueAnomalyOrbitIntersectionRootA - _apseLineRotation);
            _flightPathAngleIntersectionAOrbitOne =
                Atan(_velocityRadialIntersectionAOrbitTwo / _velocityTransverseIntersectionAOrbitTwo);
            _velocityOrbitTwo = Sqrt(Pow(_velocityRadialIntersectionAOrbitTwo, 2) +
                                     Pow(_velocityTransverseIntersectionAOrbitTwo, 2));

            _deltaV = Sqrt(Pow(_velocityOrbitOne, 2) + Pow(_velocityOrbitTwo, 2) - 2 * _velocityOrbitOne *
                _velocityOrbitTwo *
                Cos(_flightPathAngleIntersectionAOrbitTwo - _flightPathAngleIntersectionAOrbitOne));
            
            _propellantMassExpended = (1 - Exp(-_deltaV / (propellantIsp * Constants.EARTH_GRAVITY_0))) *
                                      orbitalBodyMass;
            
            return new ManeuverRequirements(_deltaV, _propellantMassExpended);
        }
    }
}