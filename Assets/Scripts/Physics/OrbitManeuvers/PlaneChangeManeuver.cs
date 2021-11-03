using UnityEditor;
using static System.Math;

namespace Physics.OrbitManeuvers
{
    public static class PlaneChangeManeuver
    {

        private static double _angularMomentum;
        private static double _eccentricity;
        private static double _radiusA;
        private static double _velocityTransverseA, _velocityRadialA;
        private static double _radiusB;
        private static double _velocityTransverseB, _velocityRadialB;

        private static double _deltaV;
        private static double _propellantMassExpended;
        
        public static ManeuverRequirements CalculateMinDeltaVPlaneChange(OrbitSpec orbitSpec, 
            double targetInclination,
            double trueAnomalyAscendingNode,
            double orbitalBodyMass,
            double propellantIsp)
        {
            _angularMomentum = KeplerianSolver.CalculateAngularMomentum(orbitSpec);
            _eccentricity = KeplerianSolver.CalculateEccentricity(orbitSpec);

            _radiusA = (Pow(_angularMomentum, 2) / Constants.GRAVITATIONAL_PARAMETER) *
                       (1 / (1 + _eccentricity * Cos(trueAnomalyAscendingNode)));

            _velocityTransverseA = _angularMomentum / _radiusA;
            _velocityRadialA = (Constants.GRAVITATIONAL_PARAMETER / _angularMomentum) * _eccentricity *
                               Sin(trueAnomalyAscendingNode);
            
            _radiusB = (Pow(_angularMomentum, 2) / Constants.GRAVITATIONAL_PARAMETER) *
                       (1 / (1 + _eccentricity * Cos(180.0 - trueAnomalyAscendingNode)));

            _velocityTransverseB = _angularMomentum / _radiusB;
            _velocityRadialB = (Constants.GRAVITATIONAL_PARAMETER / _angularMomentum) * _eccentricity *
                               Sin(180.0 - trueAnomalyAscendingNode);

            if (_velocityTransverseA < _velocityTransverseB)
            {
                _deltaV = Abs(2 * _velocityTransverseA * Sin(orbitSpec.Inclination - targetInclination) / 2);
            }
            else
            {
                _deltaV = Abs(2 * _velocityTransverseB * Sin(orbitSpec.Inclination - targetInclination) / 2);
            }
            
            _propellantMassExpended = (1 - Exp(-_deltaV / (propellantIsp * Constants.EARTH_GRAVITY_0))) *
                                      orbitalBodyMass;
            
            return new ManeuverRequirements(_deltaV, _propellantMassExpended);
        }
    }
}