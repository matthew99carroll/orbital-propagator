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
        private static double _radiusOrbitIntersection;
        
        public static ManeuverRequirements CalculateApseLineRotation(KeplerianOrbitalElements keplerianOrbitalElements,
            OrbitSpec orbitSpec,
            double finalPeriapsis,
            double finalApoapsis,
            double trueAnomalyA,
            double trueAnomalyB)
        {
            _eccentricityOrbitOne = keplerianOrbitalElements.Eccentricity;

            orbitSpec.Periapsis = finalPeriapsis;
            orbitSpec.Apoapsis = finalApoapsis;

            _eccentricityOrbitTwo = KeplerianSolver.CalculateEccentricity(orbitSpec);
            
            /* 1. Calculate angular momentum for orbit one and two
               2. Calculate apse line rotation = trueAnomalyA - trueAnomalyB
               3. Calculate a, b, c to plug into equation for the roots, which gives true anomaly at intersections I, J
               4. Calculate radius at intersection I or J
               5. Calculate the transverse and radial velocity and flight path angle in orbit 1 at intersection I
               6. Calculate the speed (magnitude) of the catellite in orbit 1 from the radial and transverse components
               7. Calculate the transverse and radial velocity and flight path angle in orbit 2 at intersection I
               8. Calculate the speed (magnitude) of the satellite in orbit 2 from the radial and transverse components
               9. Calculate delta V which is a ftn of velocity in orbit 1 and 2 and flight path in orbit 1 and 2
               10. Calculate mass of propellant expended
               11. Return a new maneuver requirement
             */
        }
    }
}