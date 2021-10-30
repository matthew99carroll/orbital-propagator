using UnityEngine;

namespace Math
{
    public static class Types
    {
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
    }
}