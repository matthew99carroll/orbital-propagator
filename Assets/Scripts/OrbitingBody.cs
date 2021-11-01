using System;
using System.Runtime.InteropServices;
using Physics;
using UnityEngine;
using Types = Math.Types;

public class OrbitingBody : IBody
{
    public string Name { get; set; }
    public double Mass { get; set; }
    public double MeanAnomaly { get; set; }
    public double EccentricAnomaly { get; set; }
    public double TrueAnomaly { get; set; }
    public double EccentricRadius { get; set; }
    public double AngularMomentum { get; set; }
    public double OrbitalPeriod { get; set; }
    
    public KeplerianSolver Solver2D { get; set; }
    public KeplerianOrbitalElements OrbitalElements { get; set; }

    public OrbitSpec OrbitSpec { get; set; }
    
    public Types.Vector3D PerifocalPosition { get; set; }
    public Types.Vector3D PerifocalVelocity { get; set; }
    public Types.Matrix3x3 TransformationMatrix { get; set; }
    
    public Types.Vector3D GeocentricEquatorialPosition { get; set; }
    public Types.Vector3D GeocentricEquatorialVelocity { get; set; }

    public OrbitingBody(string name, double mass, OrbitSpec orbitSpec)
    {
        Name = name;
        Mass = mass;
        OrbitSpec = orbitSpec;
        OrbitalElements = KeplerianSolver.CalculateKeplerianOrbitalElements(OrbitSpec);
        AngularMomentum = KeplerianSolver.CalculateAngularMomentum(OrbitSpec, OrbitalElements.Eccentricity);
        OrbitalPeriod = KeplerianSolver.CalculateOrbitalPeriod(OrbitalElements);
        MeanAnomaly = KeplerianSolver.CalculateMeanAnomaly(OrbitalElements, OrbitSpec.InitialTime,
            OrbitSpec.InitialTime, OrbitSpec.InitialTrueAnomaly);
    }

    public void ComputeNextPosition(double currentTime)
    {
        OrbitalElements = KeplerianSolver.CalculateKeplerianOrbitalElements(OrbitSpec);
        
        MeanAnomaly = KeplerianSolver.CalculateMeanAnomaly(OrbitalElements, currentTime, OrbitSpec.InitialTime,
            OrbitSpec.InitialTrueAnomaly);
        
        var (solution, errorMessage) = KeplerianSolver.CalculateEccentricAnomaly(MeanAnomaly, OrbitalElements.Eccentricity);
        if (errorMessage != null)
        {
            Debug.LogError($"ERROR: {errorMessage}");
        }
        EccentricAnomaly = solution;
        
        TrueAnomaly = KeplerianSolver.CalculateTrueAnomaly(OrbitalElements.Eccentricity, EccentricAnomaly);

        PerifocalPosition =
            KeplerianSolver.CalculatePerifocalPosition(AngularMomentum, OrbitalElements.Eccentricity, TrueAnomaly);
        
        PerifocalVelocity =
            KeplerianSolver.CalculatePerifocalVelocity(AngularMomentum, OrbitalElements.Eccentricity, TrueAnomaly);
        
        PerifocalVelocity =
            KeplerianSolver.CalculatePerifocalVelocity(AngularMomentum, OrbitalElements.Eccentricity, TrueAnomaly);
        
        TransformationMatrix = KeplerianSolver.CalculateTransformationMatrix(OrbitalElements);
        
        GeocentricEquatorialPosition =
            KeplerianSolver.CalculateGeocentricEquatorialPosition(PerifocalPosition, TransformationMatrix);
        
        GeocentricEquatorialVelocity =
            KeplerianSolver.CalculateGeocentricEquatorialVelocity(PerifocalVelocity, TransformationMatrix);
    }
}