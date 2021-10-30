using System.Collections;
using System.Collections.Generic;
using Physics;
using UnityEngine;

public class ReferenceBody: IBody
{
    public string Name { get; set; }
    public double Mass { get; set; }
    public KeplerianSolver Solver2D { get; set; }

    public ReferenceBody(string name, float mass, KeplerianOrbitalElements orbitalElements)
    {
        Name = name;
        // Solver = new KeplerianSolver()
    }
}
