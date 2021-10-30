using JetBrains.Annotations;
using Physics;

public interface IBody
{
    public string Name { get; set; }
    public double Mass { get; set; }
    public KeplerianSolver Solver2D { get; set; }
}