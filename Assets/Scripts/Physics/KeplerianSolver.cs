using System;
using Math;
using Types = Math.Types;

namespace Physics
{
    public class KeplerianSolver
    {
        public static KeplerianOrbitalElements CalculateKeplerianOrbitalElements(OrbitSpec orbitSpec)
        {
            var semiMajorAxis = CalculateSemiMajorAxis(orbitSpec);
            var eccentricity = CalculateEccentricity(orbitSpec);
            return new KeplerianOrbitalElements(
                semiMajorAxis,
                eccentricity,
                orbitSpec.Inclination,
                orbitSpec.LongitudeAscendingNode,
                orbitSpec.ArgumentOfPeriapsis
            );
        }

        public static double CalculateAngularMomentum(OrbitSpec orbitSpec, double eccentricity)
        {
            return System.Math.Sqrt((orbitSpec.Periapsis + Constants.RADIUS_EARTH) * Constants.GRAVITATIONAL_PARAMETER *
                                    (1 + eccentricity * System.Math.Cos(orbitSpec.InitialMeanLongitude)));
        }

        public static double CalculateAngularMomentum(OrbitSpec orbitSpec)
        {
            return System.Math.Sqrt(2 * Constants.GRAVITATIONAL_PARAMETER) *
                   System.Math.Sqrt((orbitSpec.Apoapsis * orbitSpec.Periapsis) /
                                    (orbitSpec.Apoapsis + orbitSpec.Periapsis));
        }
        
        public static double CalculateSemiMajorAxis(double angularMomentum, double eccentricity)
        {
            return (System.Math.Pow(angularMomentum, 2) / Constants.GRAVITATIONAL_PARAMETER) /
                   (1 - System.Math.Pow(eccentricity, 2));
        }

        public static double CalculateSemiMajorAxis(OrbitSpec orbitSpec)
        {
            return (orbitSpec.Periapsis + orbitSpec.Apoapsis + 2 * Constants.RADIUS_EARTH) / 2;
        }

        public static double CalculateOrbitalPeriod(KeplerianOrbitalElements keplerianOrbitalElements)
        {
            return (2 * System.Math.PI * System.Math.Pow(keplerianOrbitalElements.SemiMajorAxis, (3.0 / 2.0))) /
                   System.Math.Sqrt(Constants.GRAVITATIONAL_PARAMETER);
        }

        public static double CalculateEccentricity(OrbitSpec orbitSpec)
        {
            return 1 - (2 / ((orbitSpec.Apoapsis + Constants.RADIUS_EARTH) /
                (orbitSpec.Periapsis + Constants.RADIUS_EARTH) + 1));
        }

        public static double CalculateMeanAnomaly(KeplerianOrbitalElements keplerianOrbitalElements, double currentTime,
            double initialTime, double initialMeanLongitude)
        {
            var meanAngularMotion = System.Math.Sqrt(Constants.GRAVITATIONAL_PARAMETER /
                                                     System.Math.Pow(keplerianOrbitalElements.SemiMajorAxis, 3));
            return (meanAngularMotion * (currentTime - initialTime) + initialMeanLongitude);
        }

        public static (double, string) CalculateEccentricAnomaly(double meanAnomaly, double eccentricity)
        {
            var initialGuess = meanAnomaly;
            const double tolerance = 1E-6;
            const int maxIterations = 10;
            Func<double, double> function = guess => guess - eccentricity * System.Math.Sin(guess) - meanAnomaly;
            Func<double, double> derivative = guess => 1 - eccentricity * System.Math.Cos(guess);
            return NewtonsMethod.ConvergeSolution(initialGuess, function, derivative, tolerance, maxIterations);
        }

        public static double CalculateTrueAnomaly(double eccentricity, double eccentricAnomaly)
        {
            return 2 * System.Math.Atan(System.Math.Sqrt((1 + eccentricity) / (1 - eccentricity)) *
                                        System.Math.Tan(eccentricAnomaly / 2));
        }

        public static Types.Vector3D CalculatePerifocalPosition(double angularMomentum, double eccentricity,
            double trueAnomaly)
        {
            var xPosition = (((System.Math.Pow(angularMomentum, 2) / Constants.GRAVITATIONAL_PARAMETER) /
                              (1 + eccentricity * System.Math.Cos(trueAnomaly))) * System.Math.Cos(trueAnomaly));
            var yPosition = (((System.Math.Pow(angularMomentum, 2) / Constants.GRAVITATIONAL_PARAMETER) /
                              (1 + eccentricity * System.Math.Cos(trueAnomaly))) * System.Math.Sin(trueAnomaly));

            return new Types.Vector3D(xPosition, yPosition, 0.0);
        }

        public static Types.Vector3D CalculatePerifocalVelocity(double angularMomentum, double eccentricity,
            double trueAnomaly)
        {
            var xVelocity = (Constants.GRAVITATIONAL_PARAMETER / angularMomentum) * -System.Math.Sin(trueAnomaly);
            var yVelocity = (Constants.GRAVITATIONAL_PARAMETER / angularMomentum) *
                            (eccentricity + System.Math.Cos(trueAnomaly));

            return new Types.Vector3D(xVelocity, yVelocity, 0.0);
        }

        public static Types.Matrix3x3 CalculateTransformationMatrix(KeplerianOrbitalElements keplerianOrbitalElements)
        {
            var ra = keplerianOrbitalElements.LongitudeAscendingNode;
            var i = keplerianOrbitalElements.Inclination;
            var w = keplerianOrbitalElements.ArgumentOfPeriapsis;

            var xx = -System.Math.Sin(ra) * System.Math.Cos(i) * System.Math.Sin(w) 
                     + System.Math.Cos(ra) * System.Math.Cos(w);
            var xy = -System.Math.Sin(ra) * System.Math.Cos(i) * System.Math.Cos(w) 
                     - System.Math.Cos(ra) * System.Math.Sin(w);
            var xz = System.Math.Sin(ra) * System.Math.Sin(i);
            var yx = System.Math.Cos(ra) * System.Math.Cos(i) * System.Math.Sin(w) 
                     + System.Math.Sin(ra) * System.Math.Cos(w);
            var yy = System.Math.Cos(ra) * System.Math.Cos(i) * System.Math.Cos(w)
                     - System.Math.Sin(ra) * System.Math.Sin(w);
            var yz = -System.Math.Cos(ra) * System.Math.Sin(i);
            var zx = System.Math.Sin(i) * System.Math.Sin(w);
            var zy = System.Math.Sin(i) * System.Math.Cos(w);
            var zz = System.Math.Cos(i);

            return new Types.Matrix3x3(xx, xy, xz, yx, yy, yz, zx, zy, zz);
        }

        public static Types.Vector3D CalculateGeocentricEquatorialPosition(Types.Vector3D perifocalPosition,
            Types.Matrix3x3 transformationMatrix)
        {
            return Types.MatrixVectorProduct3D(transformationMatrix, perifocalPosition);
        }

        public static Types.Vector3D CalculateGeocentricEquatorialVelocity(Types.Vector3D perifocalVelocity,
            Types.Matrix3x3 transformationMatrix)
        {
            return Types.MatrixVectorProduct3D(transformationMatrix, perifocalVelocity);
        }

        public static double CalculateOblatenessRARate(KeplerianOrbitalElements keplerianOrbitalElements)
        {
            return -(0.6667 * System.Math.Sqrt(Constants.GRAVITATIONAL_PARAMETER) *
                     Constants.EARTH_SECOND_NODAL_HARMONIC * System.Math.Pow(Constants.RADIUS_EARTH, 2)
                     / (System.Math.Pow((1 - System.Math.Pow(keplerianOrbitalElements.Eccentricity, 2)), 2) *
                        System.Math.Pow(keplerianOrbitalElements.SemiMajorAxis, (7.0 / 2.0))))
                   * System.Math.Cos(keplerianOrbitalElements.Inclination);
        }

        public static double CalculateOblatenessAOPRate(KeplerianOrbitalElements keplerianOrbitalElements)
        {
            return -(0.6667 * System.Math.Sqrt(Constants.GRAVITATIONAL_PARAMETER) *
                     Constants.EARTH_SECOND_NODAL_HARMONIC * System.Math.Pow(Constants.RADIUS_EARTH, 2)
                     / (System.Math.Pow((1 - System.Math.Pow(keplerianOrbitalElements.Eccentricity, 2)), 2) *
                        System.Math.Pow(keplerianOrbitalElements.SemiMajorAxis, (7.0 / 2.0))))
                   * (2.5 * System.Math.Pow(System.Math.Sin(keplerianOrbitalElements.Inclination), 2) - 2);
        }

        public static double CalculateOrbitSpecificEnergy(KeplerianOrbitalElements keplerianOrbitalElements)
        {
            return -Constants.GRAVITATIONAL_PARAMETER / (2 * keplerianOrbitalElements.SemiMajorAxis);
        }
    }
}