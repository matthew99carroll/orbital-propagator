using System;
using UnityEngine;

namespace Math
{
    public static class NewtonsMethod
    {
        public static (double, string) ConvergeSolution(double initialGuess, Func<double, double> function, Func<double, double> derivative, double tolerance, int maxIterations)
        {
            var i = 0;
            var currentError = 1.0;
            var currentGuess = initialGuess;
            var previousGuess = 0.0;
            while (i <= maxIterations && currentError > tolerance)
            {
                previousGuess = currentGuess;
                currentGuess = previousGuess - (function(previousGuess) / derivative(previousGuess));
                currentError = System.Math.Abs(currentGuess - previousGuess);
                i++;
            }
            if (i >= maxIterations)
            {
                return (default, "Failed to converge");
            }
            
            return (currentGuess, null);
        }
    }
}