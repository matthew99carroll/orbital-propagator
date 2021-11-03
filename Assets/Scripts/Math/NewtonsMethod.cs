using System;
using UnityEngine;
using static System.Math;

namespace Math
{
    public static class NewtonsMethod
    {
        /*
         * Newtons method, which takes an initial guess, a single parameter function and derivative, and the convergence
         * tolerance and the maximum number of iterations.
         */
        public static (double, string) ConvergeSolution(double initialGuess, Func<double, double> function, Func<double, double> derivative, double tolerance, int maxIterations)
        {
            // Initialise current iteration counter
            var i = 0;
            
            // Initial error value
            var currentError = 1.0;
            
            // Initialise the current and previous guess
            var currentGuess = initialGuess;
            var previousGuess = 0.0;
            
            // Loop over until convergence is reached, else return a failed to converge message
            while (i <= maxIterations && currentError > tolerance)
            {
                previousGuess = currentGuess;
                currentGuess = previousGuess - (function(previousGuess) / derivative(previousGuess));
                currentError = Abs(currentGuess - previousGuess);
                i++;
            }
            if (i >= maxIterations)
            {
                return (default, "Failed to converge");
            }
            
            // null represents the fact that there is no error, so a null string is returned
            return (currentGuess, null);
        }
    }
}