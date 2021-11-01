using UnityEngine;
using static System.Math;

namespace Math
{
    public static class Types
    {
        public enum Direction
        {
            PROGRADE,
            RETROGRADE
        }
        
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
        
        public static Vector3D MatrixVectorProduct3D(Matrix3x3 matrix, Vector3D vector)
        {
            var x = (vector.X * matrix.XX) + (vector.Y * matrix.XY) + (vector.Z * matrix.XZ);
            var y = (vector.X * matrix.YX) + (vector.Y * matrix.YY) + (vector.Z * matrix.YZ);
            var z = (vector.X * matrix.ZX) + (vector.Y * matrix.ZY) + (vector.Z * matrix.ZZ);
            
            return new Vector3D(x, y, z);
        }
        
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

        public static double CalculateVectorNorm3D(Vector3D vector)
        {
            return Sqrt(Pow(vector.X, 2) + Pow(vector.Y, 2) + Pow(vector.Z, 2));
        }

        public static Vector3D CalculateCrossProduct3D(Vector3D vectorA, Vector3D vectorB)
        {
            Vector3D crossP = new Vector3D(0, 0, 0);
            
            crossP.X = vectorA.Y * vectorB.Z - vectorA.Z * vectorB.Y;
            crossP.Y= vectorA.Z * vectorB.X - vectorA.X * vectorB.Z;
            crossP.Z = vectorA.X * vectorB.Y - vectorA.Y * vectorB.X;

            return crossP;
        }

        public static double CalculateDotProduct3D(Vector3D vectorA, Vector3D vectorB)
        {
            return (vectorA.X * vectorB.X) + (vectorA.Y * vectorB.Y) + (vectorA.Z * vectorB.Z);
        }

        public static Vector3D ScalarMultipliedVector(double scalar, Vector3D vector)
        {
            Vector3D vectorC = new Vector3D(0, 0, 0);

            vectorC.X = scalar * vector.X;
            vectorC.Y = scalar * vector.Y;
            vectorC.Z = scalar * vector.Z;

            return vectorC;
        }
        
        public static Vector3D AddVectors(Vector3D vectorA, Vector3D vectorB)
        {
            return new Vector3D(vectorA.X + vectorB.X, vectorA.Y + vectorB.Y, vectorA.Z + vectorB.Z);
        }
        
        public static Vector3D SubtractVectors(Vector3D vectorA, Vector3D vectorB)
        {
            return new Vector3D(vectorA.X - vectorB.X, vectorA.Y - vectorB.Y, vectorA.Z - vectorB.Z);
        }
    }
}