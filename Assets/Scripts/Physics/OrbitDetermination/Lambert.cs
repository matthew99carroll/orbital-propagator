using System;
using System.Collections.Generic;
using System.Threading;
using UnityEngine;
using static System.Math;
using Types = Math.Types;

namespace Physics.OrbitDetermination
{
    public static class Lambert
    {
        private static double _radiusOneMagnitude, _radiusTwoMagnitude; // Magnitudes of R1 and R2
        private static Types.Vector3D _crossProductR1R2; // R1 Cross R2
        private static double _theta; // Angle between R1 and R2
        private static double _A; // Constant value used in calculations
        private static double  _z; // alpha * Pow(x, 2) where alpha is reciprocal of SMA and x is universal anomaly
        private static double _ratio; // Ratio of function F to derivative of F
        private static double _convergence_tolerance; // Tolerance on convergence precision
        private static double _f, _g; // Largrange Coefficients
        private static int _num_iterations; // Max number of iterations of Newtons method
        private static double _gdot; // Time derivative of _g
        private static double dummy; // Dummy variable
        private static int iteration_num; // Current iteration number
        
        // Simple function that calculates y(z) for use in solving lamberts problem.
        private static Func<double, double> _y = z => _radiusOneMagnitude + _radiusTwoMagnitude +
                                                      _A * (_z * CalculateStumpffS(z) - 1) / Sqrt(CalculateStumpffC(z));
        
        /*
         * Given two position vectors, a time of flight and direction it will calculate the corresponding velocity vectors
         */
        public static Types.LambertsVelocityVectors SolveLambertsProblem(Types.LambertsPositionVectors position,
            double timeOfFlight,
            Types.Direction direction)
        {
            // Initialise the lambert velocity vector
            Types.LambertsVelocityVectors velocityVectors = new Types.LambertsVelocityVectors(
                new Types.Vector3D(0, 0, 0),
                new Types.Vector3D(0, 0, 0));

            // Calculate the norm of the radius vectors
            _radiusOneMagnitude = Types.CalculateVectorNorm3D(position.R1);
            _radiusTwoMagnitude = Types.CalculateVectorNorm3D(position.R2);
            
            // Calculate the cross product of R1 x R2
            _crossProductR1R2 = Types.CalculateCrossProduct3D(position.R1, position.R2);

            // Calculate the angle between the two vectors
            _theta = Acos(Types.CalculateDotProduct3D(position.R1, position.R2) /
                          (_radiusOneMagnitude / _radiusTwoMagnitude));

            // Check if its a prograde or retrograde orbit, and set theta accordingly
            if (direction == Types.Direction.PROGRADE)
            {
                if (_crossProductR1R2.Z <= 0)
                {
                    _theta = (2 * PI) - _theta;
                }
            }
            else if (direction == Types.Direction.RETROGRADE)
            {
                if (_crossProductR1R2.Z >= 0)
                {
                    _theta = (2 * PI) - _theta;
                }
            }
            
            // Calculate the term A used to calculate in the function F
            _A = Sin(_theta) * Sqrt((_radiusOneMagnitude * _radiusTwoMagnitude) / (1 - Cos(_theta)));
            
            // Initial guess of F
            _z = -100;
            
            // Start doing broad strokes of the value of z until the output of F(z, t) becomes positive
            while (CalculateF(_z, timeOfFlight) < 0)
            {
                _z = _z + 0.1;
            }
            
            // Set the coonvergence tolerance and maximum number of iterations
            _convergence_tolerance = 1E-8;
            _num_iterations = 5000;
            
            // Initial value of the ratio of F/dFdz
            _ratio = 1;
            
            // Initialise iteration counter
            iteration_num = 0;
        
            // Iteration until convergence is reached
            while ((Abs(_ratio) > _convergence_tolerance) && (iteration_num <= _num_iterations))
            {
                iteration_num = iteration_num + 1;
                _ratio = CalculateF(_z, timeOfFlight) / CalculateFPrime(_z);
                _z = _z - _ratio;
            }
            
            // If no convergence occurred,log an error
            if (iteration_num >= _num_iterations)
            {
                Debug.LogError($"LAMBERT: Could not converge after {_num_iterations} iterations");
            }
            
            // Calculate the lagrange coefficients
            _f = 1 - _y(_z) / _radiusOneMagnitude;
            _g = _A * Sqrt(_y(_z) / Constants.GRAVITATIONAL_PARAMETER);
            _gdot = 1 - _y(_z) / _radiusTwoMagnitude;

            // FInally calculate the lambert velocity vectors
            velocityVectors.V1 = Types.ScalarMultipliedVector((1.0 / _g),
                Types.SubtractVectors(position.R2, Types.ScalarMultipliedVector(_f, position.R1)));

            velocityVectors.V2 = Types.ScalarMultipliedVector((1.0 / _g),
                Types.SubtractVectors(Types.ScalarMultipliedVector(_gdot, position.R2), position.R1));
            
            return velocityVectors;
        }
        
        /*
         * Calculate the S(z) value for the Stumpff function
         */
        public static double CalculateStumpffS(double z)
        {
            double s;

            if (z > 0)
            {
                s = (Sqrt(z) - Sin(Sqrt(z))) / Pow((Sqrt(z)), 3);
            }
            else if (z < 0)
            {
                s = (Sinh(Sqrt(-z)) - Sqrt(-z)) / Pow((Sqrt(-z)), 3);
            }
            else
            {
                s = 1.0 / 6.0;
            }

            return s;
        }

        /*
         * Calculate the C(z) value for the Stumpff function
         */
        public static double CalculateStumpffC(double z)
        {
            double c;

            if (z > 0)
            {
                c = (1 - Cos(Sqrt(z))) / z;
            }
            else if (z < 0)
            {
                c = (Cosh(Sqrt(-z)) - 1) / (-z);
            }
            else
            {
                c = 1.0 / 2.0;
            }

            return c;
        }
        
        /*
         * Calculate the function F used in solving the lambert problem
         */
        private static double CalculateF(double z, double t)
        {
            return Pow((_y(z) / CalculateStumpffC(z)), 1.5) * CalculateStumpffS(z)
                   + _A * Sqrt(_y(z))
                   - Sqrt(Constants.GRAVITATIONAL_PARAMETER) * t;
        }

        /*
         * Calculate the derivative of F used in solving the lambert problem
         */
        public static double CalculateFPrime(double z)
        {
            if (z == 0)
            {
                return (Sqrt(2) / 40 * Pow(_y(0), 1.5)) + (_A / 8) * (Sqrt(_y(0)) + _A * Sqrt(1 / (2 * _y(0))));
            }
            else
            {
                // This is a really gross equation, which is difficult to split up as the terms alone mean nothing
                return Pow((_y(z) / CalculateStumpffC(z)), 1.5) *
                       (1 / (2 * z) * (CalculateStumpffC(z) - 3 * CalculateStumpffS(z) / (2 * CalculateStumpffC(z)))
                        + (3.0 / 4.0) * (Pow(CalculateStumpffS(z), 2) / (CalculateStumpffC(z)))) +
                       (_A / 8) * (3 * (CalculateStumpffS(z) / CalculateStumpffC(z)) * Sqrt(_y(z)) +
                                   _A * Sqrt(CalculateStumpffC(z) / _y(z)));
            }
        }



    }
}