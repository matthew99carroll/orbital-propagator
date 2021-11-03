using UnityEngine;
using static System.Math;

namespace Math
{
    public static class Types
    {
        // Use to indicate whether your orbit is prograde or retrograde
        public enum Direction
        {
            PROGRADE,
            RETROGRADE
        }
        
        /*
         * Custom 2D Vector data type
         * Not recommended to use if you want to do vector manipulation as they are more used as a storage format
         */
        public class Vector2D
        {
            public double X;
            public double Y;

            public Vector2D(double x, double y)
            {
                X = x;
                Y = y;
            }
        }

        /*
         * Custom 3D Vector data type
         * Not recommended to use if you want to do vector manipulation as they are more used as a storage format
         */
        public class Vector3D
        {
            public double X;
            public double Y;
            public double Z;

            public Vector3D(double x, double y, double z)
            {
                X = x;
                Y = y;
                Z = z;
            }
        }
        
        /*
         * Custom 3x3 Matrix data type
         * Not recommended to use if you want to do matrix manipulation as they are more used as a storage format,
         * although a basic matrix-vector multiplication can be undertaken
         */
        public class Matrix3x3
        {
            public double XX;
            public double XY;
            public double XZ;

            public double YX;
            public double YY;
            public double YZ;

            public double ZX;
            public double ZY;
            public double ZZ;

            public Matrix3x3(double xx, double xy, double xz,
                double yx, double yy, double yz,
                double zx, double zy, double zz)
            {
                XX = xx;
                XY = xy;
                XZ = xz;

                YX = yx;
                YY = yy;
                YZ = yz;

                ZX = zx;
                ZY = zy;
                ZZ = zz;
            }
        }
        
        /*
         * Contains the two position input vectors in the orbital frame of reference, and is inputted into the
         * lambert solver
         */
        public class LambertsPositionVectors
        {
            public Vector3D R1;
            public Vector3D R2;

            public LambertsPositionVectors(Vector3D r1, Vector3D r2)
            {
                R1 = r1;
                R2 = r2;
            }
        }
        
        /*
         * Contains the two velocity return vectors in the orbital frame of reference, and is outputted from the
         * lambert solver
         */
        public class LambertsVelocityVectors
        {
            public Vector3D V1;
            public Vector3D V2;

            public LambertsVelocityVectors(Vector3D v1, Vector3D v2)
            {
                V1 = v1;
                V2 = v2;
            }
        }
        
        /*
         * Calculates a vector given a matrix and vector to multiply
         */
        public static Vector3D MatrixVectorProduct3D(Matrix3x3 matrix, Vector3D vector)
        {
            var x = (vector.X * matrix.XX) + (vector.Y * matrix.XY) + (vector.Z * matrix.XZ);
            var y = (vector.X * matrix.YX) + (vector.Y * matrix.YY) + (vector.Z * matrix.YZ);
            var z = (vector.X * matrix.ZX) + (vector.Y * matrix.ZY) + (vector.Z * matrix.ZZ);
            
            return new Vector3D(x, y, z);
        }
        
        /*
         * Calculates the vector norm when passed a 3D vector, and returns a scalar
         */
        public static double CalculateVectorNorm3D(Vector3D vector)
        {
            return Sqrt(Pow(vector.X, 2) + Pow(vector.Y, 2) + Pow(vector.Z, 2));
        }

        /*
         * Calculates the cross product between two 3D vectors and returns a 3D vector
         */
        public static Vector3D CalculateCrossProduct3D(Vector3D vectorA, Vector3D vectorB)
        {
            Vector3D crossP = new Vector3D(0, 0, 0);
            
            crossP.X = vectorA.Y * vectorB.Z - vectorA.Z * vectorB.Y;
            crossP.Y= vectorA.Z * vectorB.X - vectorA.X * vectorB.Z;
            crossP.Z = vectorA.X * vectorB.Y - vectorA.Y * vectorB.X;

            return crossP;
        }

        /*
         * Calculates the dot product between two 3D vectors and returns a scalar
         */
        public static double CalculateDotProduct3D(Vector3D vectorA, Vector3D vectorB)
        {
            return (vectorA.X * vectorB.X) + (vectorA.Y * vectorB.Y) + (vectorA.Z * vectorB.Z);
        }

        /*
         * Calculates a vector scaled by a scalar value
         */
        public static Vector3D ScalarMultipliedVector(double scalar, Vector3D vector)
        {
            Vector3D vectorC = new Vector3D(0, 0, 0);

            vectorC.X = scalar * vector.X;
            vectorC.Y = scalar * vector.Y;
            vectorC.Z = scalar * vector.Z;

            return vectorC;
        }
        
        /*
         * Adds two vectors and returns the resultant vector
         */
        public static Vector3D AddVectors(Vector3D vectorA, Vector3D vectorB)
        {
            return new Vector3D(vectorA.X + vectorB.X, vectorA.Y + vectorB.Y, vectorA.Z + vectorB.Z);
        }
        
        /*
         * Subtracts two vectors and returns the resultant vector
         */
        public static Vector3D SubtractVectors(Vector3D vectorA, Vector3D vectorB)
        {
            return new Vector3D(vectorA.X - vectorB.X, vectorA.Y - vectorB.Y, vectorA.Z - vectorB.Z);
        }
    }
}