# Orbital Propagator
The following is a program written in C# to interface with Unity 3D for visualisation of orbits based on a propagator.

## How it works
The True Anomaly is calculated by first calculating the Mean Anomaly, and then using this to calculate the Eccentric Anomaly. The equation for Eccentric Anomaly is implicit and the Eccentric Anomaly term appears twice, thus it must be numerically calculated. In this case I used a basic Newton's method approach to calculate this. From here the True Anomaly is calculated, which is in turn used to calculate a state vector in the Perifocal frame of reference, and using a transformation matrix, this state vector is taken into the Geocentric Equatorial frame of reference. The J2 pertubation is taken into account to calculate the time varying longitude of ascending node and argument of periapsis with time.

Basic Hohmann transfers have also been implemented, and given a final orbit, satellite mass and specific impulse of the satellite's fuel, will return the required delta V and expended mass.

## Where to find the code
All code can be found within the Assets/Scripts folders.

- ReferenceBody.cs simply contains the mass and name of the parent body the satellite is orbiting.
- OrbitingBody.cs contains all of the parameters used to calculate the state vector and orbital elements of a satellite.
- IBody.cs is a basic interface defining the name of the satellite, mass and its solver.
- Simulation.cs contains the main runtime code that interfaces with Unity3D.

Inside this folder you will see folders containing the following:

### Math
Contains custom classes for vector, matrix and other data structures used in common calculations. A general implemntation of numerical methods is also contained in this folder.

### Physics
Contains some basic rigidbody dynamics and all of the code for determining the oribtal elements and state vectors of a satellite.

## Future Additions
I hope to add the following when I have spare time:
- Orbital maneuvers such as Chase, Plane change and other maneuvers around a parent body.
- Lunar trajectories using coplanar patched conics and numerical integration techniques.
- Interplanetary trajectories which includes implementation of sphere of influence, optimal planetary departures, sensitivity analysis, interplanetary Hohmann transfers, Non-Hohmann interplanetary trajectories and Rendezvous opportunities.
- Extended orbital pertubations such as Atomospheric drag, solar radiation pressure, lunar gravity, solar gravity etc.
