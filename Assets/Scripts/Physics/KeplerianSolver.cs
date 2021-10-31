using System;
using Math;
using Types = Math.Types;
using static System.Math;

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
            return Sqrt((orbitSpec.Periapsis + Constants.RADIUS_EARTH) * Constants.GRAVITATIONAL_PARAMETER *
                                    (1 + eccentricity * Cos(orbitSpec.InitialMeanLongitude)));
        }

        public static double CalculateAngularMomentum(OrbitSpec orbitSpec)
        {
            return Sqrt(2 * Constants.GRAVITATIONAL_PARAMETER) *
                   Sqrt((orbitSpec.Apoapsis * orbitSpec.Periapsis) /
                                    (orbitSpec.Apoapsis + orbitSpec.Periapsis));
        }
        
        public static double CalculateSemiMajorAxis(double angularMomentum, double eccentricity)
        {
            return (Pow(angularMomentum, 2) / Constants.GRAVITATIONAL_PARAMETER) /
                   (1 - Pow(eccentricity, 2));
        }

        public static double CalculateSemiMajorAxis(OrbitSpec orbitSpec)
        {
            return (orbitSpec.Periapsis + orbitSpec.Apoapsis + 2 * Constants.RADIUS_EARTH) / 2;
        }

        public static double CalculateOrbitalPeriod(KeplerianOrbitalElements keplerianOrbitalElements)
        {
            return (2 * PI * Pow(keplerianOrbitalElements.SemiMajorAxis, (3.0 / 2.0))) /
                   Sqrt(Constants.GRAVITATIONAL_PARAMETER);
        }

        public static double CalculateEccentricity(OrbitSpec orbitSpec)
        {
            return 1 - (2 / ((orbitSpec.Apoapsis + Constants.RADIUS_EARTH) /
                (orbitSpec.Periapsis + Constants.RADIUS_EARTH) + 1));
        }

        public static double CalculateMeanAnomaly(KeplerianOrbitalElements keplerianOrbitalElements, double currentTime,
            double initialTime, double initialMeanLongitude)
        {
            var meanAngularMotion = Sqrt(Constants.GRAVITATIONAL_PARAMETER /
                                                     Pow(keplerianOrbitalElements.SemiMajorAxis, 3));
            return (meanAngularMotion * (currentTime - initialTime) + initialMeanLongitude);
        }

        public static (double, string) CalculateEccentricAnomaly(double meanAnomaly, double eccentricity)
        {
            var initialGuess = meanAnomaly;
            const double tolerance = 1E-6;
            const int maxIterations = 10;
            Func<double, double> function = guess => guess - eccentricity * Sin(guess) - meanAnomaly;
            Func<double, double> derivative = guess => 1 - eccentricity * Cos(guess);
            return NewtonsMethod.ConvergeSolution(initialGuess, function, derivative, tolerance, maxIterations);
        }

        public static double CalculateTrueAnomaly(double eccentricity, double eccentricAnomaly)
        {
            return 2 * Atan(Sqrt((1 + eccentricity) / (1 - eccentricity)) *
                                        Tan(eccentricAnomaly / 2));
        }

        public static Types.Vector3D CalculatePerifocalPosition(double angularMomentum, double eccentricity,
            double trueAnomaly)
        {
            var xPosition = (((Pow(angularMomentum, 2) / Constants.GRAVITATIONAL_PARAMETER) /
                              (1 + eccentricity * Cos(trueAnomaly))) * Cos(trueAnomaly));
            var yPosition = (((Pow(angularMomentum, 2) / Constants.GRAVITATIONAL_PARAMETER) /
                              (1 + eccentricity * Cos(trueAnomaly))) * Sin(trueAnomaly));

            return new Types.Vector3D(xPosition, yPosition, 0.0);
        }

        public static Types.Vector3D CalculatePerifocalVelocity(double angularMomentum, double eccentricity,
            double trueAnomaly)
        {
            var xVelocity = (Constants.GRAVITATIONAL_PARAMETER / angularMomentum) * -Sin(trueAnomaly);
            var yVelocity = (Constants.GRAVITATIONAL_PARAMETER / angularMomentum) *
                            (eccentricity + Cos(trueAnomaly));

            return new Types.Vector3D(xVelocity, yVelocity, 0.0);
        }

        public static Types.Matrix3x3 CalculateTransformationMatrix(KeplerianOrbitalElements keplerianOrbitalElements)
        {
            var ra = keplerianOrbitalElements.LongitudeAscendingNode;
            var i = keplerianOrbitalElements.Inclination;
            var w = keplerianOrbitalElements.ArgumentOfPeriapsis;

            var xx = -Sin(ra) * Cos(i) * Sin(w) 
                     + Cos(ra) * Cos(w);
            var xy = -Sin(ra) * Cos(i) * Cos(w) 
                     - Cos(ra) * Sin(w);
            var xz = Sin(ra) * Sin(i);
            var yx = Cos(ra) * Cos(i) * Sin(w) 
                     + Sin(ra) * Cos(w);
            var yy = Cos(ra) * Cos(i) * Cos(w)
                     - Sin(ra) * Sin(w);
            var yz = -Cos(ra) * Sin(i);
            var zx = Sin(i) * Sin(w);
            var zy = Sin(i) * Cos(w);
            var zz = Cos(i);

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
            return -(0.6667 * Sqrt(Constants.GRAVITATIONAL_PARAMETER) *
                     Constants.EARTH_SECOND_NODAL_HARMONIC * Pow(Constants.RADIUS_EARTH, 2)
                     / (Pow((1 - Pow(keplerianOrbitalElements.Eccentricity, 2)), 2) *
                        Pow(keplerianOrbitalElements.SemiMajorAxis, (7.0 / 2.0))))
                   * Cos(keplerianOrbitalElements.Inclination);
        }

        public static double CalculateOblatenessAOPRate(KeplerianOrbitalElements keplerianOrbitalElements)
        {
            return -(0.6667 * Sqrt(Constants.GRAVITATIONAL_PARAMETER) *
                     Constants.EARTH_SECOND_NODAL_HARMONIC * Pow(Constants.RADIUS_EARTH, 2)
                     / (Pow((1 - Pow(keplerianOrbitalElements.Eccentricity, 2)), 2) *
                        Pow(keplerianOrbitalElements.SemiMajorAxis, (7.0 / 2.0))))
                   * (2.5 * Pow(Sin(keplerianOrbitalElements.Inclination), 2) - 2);
        }

        public static double CalculateOrbitSpecificEnergy(KeplerianOrbitalElements keplerianOrbitalElements)
        {
            return -Constants.GRAVITATIONAL_PARAMETER / (2 * keplerianOrbitalElements.SemiMajorAxis);
        }
    }
}