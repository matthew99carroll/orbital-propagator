using System;
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
        private static double _z; // alpha * Pow(x, 2) where alpha is reciprocal of semi-major axis and x is universal anomaly
        //private static Func<double> _y; // Equation that is a function of z
        //private static Func<double, double> _F; // Equation that is a function of z and constant t
        //private static Func<double> _dFdz; // Derivative of _F
        private static double _ratio; // Ratio of _F to Derivative of _F
        private static double _convergence_tolerance; // Tolerance on convergence precision
        private static double _f, _g; // Largrange Coefficients
        private static int _num_iterations; // Max number of iterations of Newtons method
        private static double _gdot; // Time derivative of _g
        private static Func<double> _C, _S; // Stumpff functions
        private static double dummy; // Dummy variable
        
        public static Types.LambertsVelocityVectors SolveLambertsProblem(Types.LambertsPositionVectors position, 
            double timeOfFlight, 
            Func<double, double> F,
            Func<double> dFdz,
            Func<double> y,
            Types.Direction direction)
        {
            /*
             * TODO: Write method to calculate Stumpff functions to determine S(z) and C(z).
             * Which are used for F(z, t), F'(z), y(z). These will be passed into this function to calculate lamberts
             * velocity vector
             */
            
            Types.LambertsVelocityVectors velocityVectors = new Types.LambertsVelocityVectors(
                new Types.Vector3D(0, 0, 0), 
                new Types.Vector3D(0, 0, 0));
            
            _radiusOneMagnitude = Types.CalculateVectorNorm3D(position.R1);
            _radiusTwoMagnitude = Types.CalculateVectorNorm3D(position.R2);

            _crossProductR1R2 = Types.CalculateCrossProduct3D(position.R1, position.R2);

            _theta = Acos(Types.CalculateDotProduct3D(position.R1, position.R2) / (_radiusOneMagnitude / _radiusTwoMagnitude));

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

            _A = Sin(_theta) * Sqrt((_radiusOneMagnitude * _radiusTwoMagnitude) / (1 - Cos(_theta)));

            _z = -100;
            
            // TODO: IMPLEMENT F(Z,T)
            // while (F(_z, timeOfFlight) < 0)
            // {
            //     _z = _z + 0.1;
            // }

            _convergence_tolerance = 1E-8;
            _num_iterations = 5000;

            _ratio = 1;
            int n = 0;

            while ((Abs(_ratio) > _convergence_tolerance) && (n <= _num_iterations))
            {
                n = n + 1;
                // TODO: IMPLEMENT F(Z,T) and F'(z)
                //_ratio = F(_z, timeOfFlight) / dFdz(_z);
                _z = _z - _ratio;
            }

            if (n >= _num_iterations)
            {
                /* Insert a return error type */
            }
            
            // TODO: IMPLEMENT y(z)
            //_f = 1 - y(_z) / _radiusOneMagnitude;
            //_g = _A * Sqrt(y(_z) / Constants.GRAVITATIONAL_PARAMETER);
            //_gdot = 1 - y(_z) / _radiusTwoMagnitude;

            velocityVectors.V1 = Types.ScalarMultipliedVector((1.0/_g), 
                Types.SubtractVectors(position.R2, Types.ScalarMultipliedVector(_f, position.R1)));
            
            velocityVectors.V2 = Types.ScalarMultipliedVector((1.0/_g), 
                Types.SubtractVectors(Types.ScalarMultipliedVector(_gdot, position.R2), position.R1));

            return velocityVectors;
        }
    }
}