using Math;
using static Math.Types;
using static System.Math;

namespace Physics.OrbitManeuvers
{
    public static class TwoImpulseRendezvousManeuver
    {
        private static Vector3D _directionCosineMatrixGefToChaserI,
                                _directionCosineMatrixGefToChaserJ,
                                _directionCosineMatrixGefToChaserK;

        private static Matrix3x3 _directionCosineMatrix;
        private static Vector3D _relPositionChaserToTargetGef;
        private static double _meanMotionTarget;
        private static Vector3D _angularVelocityTarget;
        private static Vector3D _relVelocityChaserToTargetGef;

        private static Vector3D _initRelPositionTargetsRefFrame, _initRelVelocityTargetsRefFrameMinus;
        private static Vector3D _initRelVelocityTargetsRefFramePlus;
        private static Vector3D _finalRelVelocityTargetsRefFrameMinus;

        private static ClohessyWiltshireMatrices _matrices;
        
        private static Vector3D _initialDeltaV, _finalDeltaV;
        private static double _deltaV;
        private static double _propellantMassExpended;
        
        public static ManeuverRequirements CalculateRendezvousManeuver(
            Vector3D positionGeoEqChaser,
            Vector3D velocityGeoEqChaser,
            Vector3D positionGeoEqTarget,
            Vector3D velocityGeoEqTarget,
            double rendezvousTime,
            double chaserMass,
            double chaserPropellantIsp)
        {
            _directionCosineMatrixGefToChaserI = new Vector3D();
            _directionCosineMatrixGefToChaserJ = new Vector3D();
            
            // Calculate row 1, column i, j, k of the direction cosine matrix
            _directionCosineMatrixGefToChaserI.X =
                positionGeoEqTarget.X / CalculateVectorNorm3D(positionGeoEqTarget);
            _directionCosineMatrixGefToChaserI.Y =
                positionGeoEqTarget.Y / CalculateVectorNorm3D(positionGeoEqTarget);
            _directionCosineMatrixGefToChaserI.Z =
                positionGeoEqTarget.Z / CalculateVectorNorm3D(positionGeoEqTarget);

            // Calculate row 2, column i, j, k of the direction cosine matrix
            _directionCosineMatrixGefToChaserJ.X =
                velocityGeoEqTarget.X / CalculateVectorNorm3D(velocityGeoEqTarget);
            _directionCosineMatrixGefToChaserJ.Y =
                velocityGeoEqTarget.Y / CalculateVectorNorm3D(velocityGeoEqTarget);
            _directionCosineMatrixGefToChaserJ.Z =
                velocityGeoEqTarget.Z / CalculateVectorNorm3D(velocityGeoEqTarget);

            // Calculate row 3 of the direction cosine matrix by getting the cross product of row 1 and 2
            _directionCosineMatrixGefToChaserK = CalculateCrossProduct3D(_directionCosineMatrixGefToChaserI,
                _directionCosineMatrixGefToChaserJ);

            _directionCosineMatrix = new Matrix3x3();
            
            // Assign the three vectors calculated above into the 3x3 matrix
            _directionCosineMatrix.XX = _directionCosineMatrixGefToChaserI.X;
            _directionCosineMatrix.XY = _directionCosineMatrixGefToChaserI.Y;
            _directionCosineMatrix.XZ = _directionCosineMatrixGefToChaserI.Z;
            _directionCosineMatrix.YX = _directionCosineMatrixGefToChaserJ.X;
            _directionCosineMatrix.YY = _directionCosineMatrixGefToChaserJ.Y;
            _directionCosineMatrix.YZ = _directionCosineMatrixGefToChaserJ.Z;
            _directionCosineMatrix.ZX = _directionCosineMatrixGefToChaserK.X;
            _directionCosineMatrix.ZY = _directionCosineMatrixGefToChaserK.Y;
            _directionCosineMatrix.ZZ = _directionCosineMatrixGefToChaserK.Z;

            // Calculate the relative position of the chase vehicle to the target in the Geocentric Equatorial Frame of reference
            _relPositionChaserToTargetGef = SubtractVectors(positionGeoEqChaser, positionGeoEqTarget);

            // Calculate the mean motion of the target, which is used to calculate the angular velocity of the target
            _meanMotionTarget = CalculateVectorNorm3D(velocityGeoEqTarget) / CalculateVectorNorm3D(positionGeoEqTarget);

            // Angular velocity of target
            _angularVelocityTarget = new Vector3D(0, 0, _meanMotionTarget);

            // Calculate the relative velocity of the chase vehicle to the target in the Geocentric Equatorial Frame of reference
            _relVelocityChaserToTargetGef = SubtractVectors(SubtractVectors(velocityGeoEqChaser, velocityGeoEqTarget),
                CalculateCrossProduct3D(_angularVelocityTarget,
                    _relPositionChaserToTargetGef));

            // Calculate the initial position vector at the beginning of the rendezvous in the targets frame of reference
            _initRelPositionTargetsRefFrame =
                MatrixVectorProduct3D(_directionCosineMatrix, _relPositionChaserToTargetGef);

            // Calculate the initial velocity vector just before launch into the rendezvous trajectory
            _initRelVelocityTargetsRefFrameMinus =
                MatrixVectorProduct3D(_directionCosineMatrix, _relVelocityChaserToTargetGef);
            
            // Initialise empty Clohessy-Wiltshire matrix datatype
            _matrices = new ClohessyWiltshireMatrices();
            
            // Clohessy-Wiltshire Matrix Phi rr
            _matrices.phirr = new Matrix3x3((4 - 3 * Cos(_meanMotionTarget * rendezvousTime)), 0.0, 0.0,
                (6 * (Sin(_meanMotionTarget * rendezvousTime) - _meanMotionTarget * rendezvousTime)), 1.0, 0.0,
                0.0, 0.0, Cos(_meanMotionTarget * rendezvousTime));
            
            // Clohessy-Wiltshire Matrix Phi rv
            _matrices.phirv = new Matrix3x3((Sin(_meanMotionTarget * rendezvousTime) / _meanMotionTarget),
                ((2 * (1 - Cos(_meanMotionTarget * rendezvousTime))) / _meanMotionTarget), 0.0,
                ((2 * (Cos(_meanMotionTarget * rendezvousTime) - 1)) / _meanMotionTarget),
                ((4 * Sin(_meanMotionTarget * rendezvousTime) - 3 * _meanMotionTarget * rendezvousTime) /
                 _meanMotionTarget), 0.0,
                0.0, 0.0, Sin(_meanMotionTarget * rendezvousTime) / _meanMotionTarget);
            
            // Clohessy-Wiltshire Matrix Phi vr
            _matrices.phivr = new Matrix3x3((3 * _meanMotionTarget * Sin(_meanMotionTarget * rendezvousTime)), 0.0, 0.0,
                (6 * _meanMotionTarget * (Cos(_meanMotionTarget * rendezvousTime) - 1.0)), 0.0, 0.0,
                0.0, 0.0, -_meanMotionTarget * Sin(_meanMotionTarget * rendezvousTime));
            
            // Clohessy-Wiltshire Matrix Phi vv
            _matrices.phivv = new Matrix3x3(Cos(_meanMotionTarget * rendezvousTime),
                2 * Sin(_meanMotionTarget * rendezvousTime), 0.0,
                -2 * Sin(_meanMotionTarget * rendezvousTime), 4 * Cos(_meanMotionTarget * rendezvousTime) - 3, 0.0,
                0.0, 0.0, Cos(_meanMotionTarget * rendezvousTime));
            
            // Calculate the initial relative velocity just inside the rendezvous window in the targets frame of reference
            _initRelVelocityTargetsRefFramePlus = MatrixVectorProduct3D(
                MatrixMultiply(ScalarMultipliedMatrix(-1, CalculateInverseMatrix(_matrices.phirv)), _matrices.phirr),
                _initRelPositionTargetsRefFrame);
            
            // Calculate the final relative velocity just inside the rendezvous window in the targets frame of reference
            _finalRelVelocityTargetsRefFrameMinus = AddVectors(
                MatrixVectorProduct3D(_matrices.phivr, _initRelPositionTargetsRefFrame),
                MatrixVectorProduct3D(_matrices.phivv, _initRelVelocityTargetsRefFramePlus));
            
            // Calculate the initial delta V at the start of the rendezvous window
            _initialDeltaV =
                SubtractVectors(_initRelVelocityTargetsRefFramePlus, _initRelVelocityTargetsRefFrameMinus);
            
            // Calculate the final delta V at the end of the rendezvous window
            _finalDeltaV = SubtractVectors(new Vector3D(0, 0, 0), _finalRelVelocityTargetsRefFrameMinus);
            
            // Calulate the delta V used in the maneuver
            _deltaV = Abs(CalculateVectorNorm3D(_initialDeltaV)) + Abs(CalculateVectorNorm3D(_finalDeltaV));
            
            // Calculate the propellant mass expended doing the maneuver
            _propellantMassExpended = (1 - Exp(-_deltaV / (chaserPropellantIsp * Constants.EARTH_GRAVITY_0))) *
                                      chaserMass;

            return new ManeuverRequirements(_deltaV, _propellantMassExpended);
        }
    }
}