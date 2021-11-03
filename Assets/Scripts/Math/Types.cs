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

            public Vector3D()
            {
                X = 0;
                Y = 0;
                Z = 0;
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

            public Matrix3x3()
            {
                XX = 0;
                XY = 0;
                XZ = 0;

                YX = 0;
                YY = 0;
                YZ = 0;

                ZX = 0;
                ZY = 0;
                ZZ = 0;
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

        public class ClohessyWiltshireMatrices
        {
            public Matrix3x3 phirr;
            public Matrix3x3 phirv;
            public Matrix3x3 phivr;
            public Matrix3x3 phivv;

            public ClohessyWiltshireMatrices(Matrix3x3 _phirr, Matrix3x3 _phirv, Matrix3x3 _phivr, Matrix3x3 _phivv)
            {
                phirr = _phirr;
                phirv = _phirv;
                phivr = _phivr;
                phivv = _phivv;
            }

            public ClohessyWiltshireMatrices()
            {
                phirr = new Matrix3x3();
                phirv = new Matrix3x3();
                phivr = new Matrix3x3();
                phivv = new Matrix3x3(); 
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

        public static double CalculateDeterminant3x3(Matrix3x3 matrix)
        {
            return (matrix.XX * (matrix.YY * matrix.ZZ - matrix.YZ * matrix.ZY)) -
                   (matrix.XY * (matrix.YX * matrix.ZZ - matrix.YZ * matrix.ZX)) +
                   (matrix.XZ * (matrix.YX * matrix.ZY - matrix.YY * matrix.ZX));
        }

        public static Matrix3x3 ScalarMultipliedMatrix(double scalar, Matrix3x3 matrix)
        {
            Matrix3x3 matrixC = new Matrix3x3();

            matrixC.XX = scalar * matrix.XX;
            matrixC.XY = scalar * matrix.XY;
            matrixC.XZ = scalar * matrix.XZ;

            matrixC.YX = scalar * matrix.YX;
            matrixC.YY = scalar * matrix.YY;
            matrixC.YZ = scalar * matrix.YZ;

            matrixC.ZX = scalar * matrix.ZX;
            matrixC.ZY = scalar * matrix.ZY;
            matrixC.ZZ = scalar * matrix.ZZ;

            return matrixC;
        }

        public static Matrix3x3 CalculateInverseMatrix(Matrix3x3 matrix)
        {
            double determinant = CalculateDeterminant3x3(matrix);

            Matrix3x3 matrixInverse = new Matrix3x3();

            matrixInverse.XX = matrix.YY * matrix.ZZ - matrix.YZ * matrix.ZY;
            matrixInverse.XY = matrix.YZ * matrix.ZX - matrix.YX * matrix.ZZ;
            matrixInverse.XZ = matrix.YX * matrix.ZY - matrix.YY * matrix.ZX;

            matrixInverse.YX = matrix.ZY * matrix.XZ - matrix.ZX * matrix.XY;
            matrixInverse.YY = matrix.XX * matrix.ZZ - matrix.XZ * matrix.ZX;
            matrixInverse.YZ = matrix.XZ * matrix.XY - matrix.XX * matrix.YZ;

            matrixInverse.ZX = matrix.YX * matrix.ZY - matrix.YZ * matrix.YX;
            matrixInverse.ZY = matrix.YZ * matrix.ZX - matrix.YX * matrix.ZY;
            matrixInverse.ZZ = matrix.YX * matrix.XY - matrix.YY * matrix.ZX;

            matrixInverse = ScalarMultipliedMatrix(1 / determinant, matrixInverse);

            return matrixInverse;
        }

        public static Matrix3x3 MatrixMultiply(Matrix3x3 matrixA, Matrix3x3 matrixB)
        {
            Matrix3x3 matrixC = new Matrix3x3();

            matrixC.XX = matrixA.XX * matrixB.XX + matrixA.XY * matrixB.YX + matrixA.XZ * matrixB.ZX;
            matrixC.XY = matrixA.XX * matrixB.XY + matrixA.XY * matrixB.YY + matrixA.XZ * matrixB.ZY;
            matrixC.XZ = matrixA.XX * matrixB.XZ + matrixA.XY * matrixB.YZ + matrixA.XZ * matrixB.ZZ;

            matrixC.YX = matrixA.YX * matrixB.XX + matrixA.YY * matrixB.YX + matrixA.YZ * matrixB.ZX;
            matrixC.YY = matrixA.YX * matrixB.XY + matrixA.YY * matrixB.YY + matrixA.YZ * matrixB.ZY;
            matrixC.YZ = matrixA.YX * matrixB.XZ + matrixA.YY * matrixB.YZ + matrixA.YZ * matrixB.ZZ;

            matrixC.ZX = matrixA.ZX * matrixB.XX + matrixA.ZY * matrixB.YX + matrixA.ZZ * matrixB.ZX;
            matrixC.ZY = matrixA.ZX * matrixB.XY + matrixA.ZY * matrixB.YY + matrixA.ZZ * matrixB.ZY;
            matrixC.ZZ = matrixA.ZX * matrixB.XZ + matrixA.ZY * matrixB.YZ + matrixA.ZZ * matrixB.ZZ;

            return matrixC;
        }

        public static Matrix3x3 MatrixAddition(Matrix3x3 matrixA, Matrix3x3 matrixB)
        {
            Matrix3x3 matrixC = new Matrix3x3();

            matrixC.XX = matrixA.XX + matrixB.XX;
            matrixC.XY = matrixA.XY + matrixB.XY;
            matrixC.XZ = matrixA.XZ + matrixB.XZ;

            matrixC.YX = matrixA.YX + matrixB.YX;
            matrixC.YY = matrixA.YY + matrixB.YY;
            matrixC.YZ = matrixA.YZ + matrixB.YZ;

            matrixC.ZX = matrixA.ZX + matrixB.ZX;
            matrixC.ZY = matrixA.ZY + matrixB.ZY;
            matrixC.ZZ = matrixA.ZZ + matrixB.ZZ;

            return matrixC;
        }
    }
}