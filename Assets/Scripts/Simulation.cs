using System;
using System.Collections;
using System.Collections.Generic;
using Physics;
using UnityEngine;

public class Simulation : MonoBehaviour
{
    public GameObject ReferenceBody;
    public GameObject SatellitePrefab;
    private readonly List<GameObject> _satelliteGameObjects = new List<GameObject>();
    private readonly List<OrbitingBody> _orbitingBodies = new List<OrbitingBody>();

    private double _currentTime = 1.0;

    public double ApoapsisSatelliteOne = 15000000;
    public double PeriapsisSatelliteOne = 150000;
    public double InclinationSatelliteOne = 0;
    public double LongitudeAscendingNodeSatelliteOne = 0;
    public double ArgumentOfPeriapsisSatelliteOne = 0;

    public double ApoapsisSatelliteTwo = 30000000;
    public double PeriapsisSatelliteTwo = 600000;
    public double InclinationSatelliteTwo = 0;
    public double LongitudeAscendingNodeSatelliteTwo = 0;
    public double ArgumentOfPeriapsisSatelliteTwo = 0;
    private double _prevTime;
    
    private void Awake()
    {
        var orbitSpecSatelliteOne = new OrbitSpec(ApoapsisSatelliteOne, PeriapsisSatelliteOne,
            InclinationSatelliteOne * (System.Math.PI/180),
            LongitudeAscendingNodeSatelliteOne * (System.Math.PI/180),
            ArgumentOfPeriapsisSatelliteOne * (System.Math.PI/180),
            0,
            0);
        var satelliteOne = new OrbitingBody("Deez 1", 100, orbitSpecSatelliteOne);
        _orbitingBodies.Add(satelliteOne);

        var orbitSpecSatelliteTwo = new OrbitSpec(ApoapsisSatelliteTwo, PeriapsisSatelliteTwo,
            InclinationSatelliteTwo * (System.Math.PI/180),
            LongitudeAscendingNodeSatelliteTwo * (System.Math.PI/180),
            ArgumentOfPeriapsisSatelliteTwo * (System.Math.PI/180),
            0,
            0);
        var satelliteTwo = new OrbitingBody("Deez 2", 100, orbitSpecSatelliteTwo);
        _orbitingBodies.Add(satelliteTwo);

        InitVisualisation();
    }

    private void InitVisualisation()
    {
        foreach (var orbitingBody in _orbitingBodies)
        {
            var satelliteGameObject = Instantiate(SatellitePrefab);
            satelliteGameObject.name = orbitingBody.Name;
            _satelliteGameObjects.Add(satelliteGameObject);
        }
    }

    private void Update()
    {

        if (Time.time > _prevTime + 0.2f)
        {
            foreach (var orbitingBody in _orbitingBodies)
            {
                if (orbitingBody.Name == "Deez 1")
                {
                    orbitingBody.OrbitSpec.Apoapsis = ApoapsisSatelliteOne;
                    orbitingBody.OrbitSpec.Periapsis = PeriapsisSatelliteOne;
                    orbitingBody.OrbitSpec.Inclination = InclinationSatelliteOne * (System.Math.PI/180);
                    orbitingBody.OrbitSpec.LongitudeAscendingNode = LongitudeAscendingNodeSatelliteOne * (System.Math.PI/180);
                    orbitingBody.OrbitSpec.ArgumentOfPeriapsis = ArgumentOfPeriapsisSatelliteOne * (System.Math.PI/180);
                }
                
                if (orbitingBody.Name == "Deez 2")
                {
                    orbitingBody.OrbitSpec.Apoapsis = ApoapsisSatelliteTwo;
                    orbitingBody.OrbitSpec.Periapsis = PeriapsisSatelliteTwo;
                    orbitingBody.OrbitSpec.Inclination = InclinationSatelliteTwo * (System.Math.PI/180);
                    orbitingBody.OrbitSpec.LongitudeAscendingNode = LongitudeAscendingNodeSatelliteTwo * (System.Math.PI/180);
                    orbitingBody.OrbitSpec.ArgumentOfPeriapsis = ArgumentOfPeriapsisSatelliteTwo * (System.Math.PI/180);
                }
            }
            
            _prevTime = Time.time;
        }
        
        ComputeNextStep();
    }

    private void ComputeNextStep()
    {
        foreach (var orbitingBody in _orbitingBodies)
        {
            orbitingBody.ComputeNextPosition(_currentTime++);

            // Update visualisation
            UpdateOrbitingBodyVisualisation(orbitingBody);
        }
    }

    private void UpdateOrbitingBodyVisualisation(OrbitingBody orbitingBody)
    {
        var orbitingBodyGameObject = _satelliteGameObjects.Find(ob => ob.name == orbitingBody.Name);

        var scaledXPosition = (float) (orbitingBody.GeocentricEquatorialPosition.X * Constants.VISUALISATION_SCALE);
        var scaledYPosition = (float) (orbitingBody.GeocentricEquatorialPosition.Y * Constants.VISUALISATION_SCALE);
        var scaledZPosition = (float) (orbitingBody.GeocentricEquatorialPosition.Z * Constants.VISUALISATION_SCALE);

        orbitingBodyGameObject.transform.position = new Vector3(scaledXPosition, scaledYPosition, scaledZPosition);
    }
}