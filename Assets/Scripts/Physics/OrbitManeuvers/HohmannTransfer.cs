using UnityEngine;

namespace Physics.OrbitManeuvers
{
    public static class HohmannTransfer
    {
        public static ManeuverRequirements CalculateHohmannTransfer(KeplerianOrbitalElements keplerianOrbitalElements, 
            OrbitSpec orbitSpec,
            double finalPeriapsis,
            double finalApoapsis,
            double orbitalBodyMass,
            double propellantIsp)
        {
            var periapsisFinal = finalPeriapsis + Constants.RADIUS_EARTH;
            var apoapsisFinal = finalApoapsis + Constants.RADIUS_EARTH;

            var periapsisOrbitOne = orbitSpec.Periapsis + Constants.RADIUS_EARTH;
            var apoapsisOrbitOne = orbitSpec.Apoapsis + Constants.RADIUS_EARTH;
            var angularMomentumOrbitOne = KeplerianSolver.CalculateAngularMomentum(orbitSpec);

            var velocityOrbitOnePointA = angularMomentumOrbitOne / periapsisOrbitOne;
            
            orbitSpec.Periapsis = periapsisOrbitOne - Constants.RADIUS_EARTH;
            orbitSpec.Apoapsis = apoapsisFinal - Constants.RADIUS_EARTH;
            keplerianOrbitalElements.Eccentricity = KeplerianSolver.CalculateEccentricity(orbitSpec);
            
            var angularMomentumOrbitTwo = KeplerianSolver.CalculateAngularMomentum(orbitSpec);

            var velocityOrbitTwoPointA = angularMomentumOrbitTwo / periapsisOrbitOne;
            var deltaVPointA = velocityOrbitTwoPointA - velocityOrbitOnePointA;

            var velocityOrbitTwoPointB = angularMomentumOrbitTwo / apoapsisFinal;

            orbitSpec.Periapsis = periapsisFinal - Constants.RADIUS_EARTH;
            orbitSpec.Apoapsis = apoapsisFinal - Constants.RADIUS_EARTH;
            keplerianOrbitalElements.Eccentricity = KeplerianSolver.CalculateEccentricity(orbitSpec);

            var angularMomentumFinal = KeplerianSolver.CalculateAngularMomentum(orbitSpec);

            var velocityOrbitThreePointB = angularMomentumFinal / apoapsisFinal;

            var deltaVPointB = velocityOrbitThreePointB - velocityOrbitTwoPointB;

            var deltaV = System.Math.Abs(deltaVPointA) + System.Math.Abs(deltaVPointB);

            var propellantMassExpended = (1 - System.Math.Exp(-deltaV / (propellantIsp * Constants.EARTH_GRAVITY_0))) *
                                         orbitalBodyMass;
            
            return new ManeuverRequirements(deltaV, propellantMassExpended);
        }
    }
}